package internal

import (
	"fmt"
	"norma_core/software/station/shared/station"
	"norma_core/target/generated-sources/protobuf/station/inference"
	"sync"

	"github.com/norma-core/normfs/normfs_go/uintn"

	"github.com/rs/zerolog/log"
)

const InferenceStatesQueue = "inference-states"
const searchStep = 10000 // Move by 10k entries when searching

type QueueBounds struct {
	From uintn.UintN
	To   uintn.UintN
}

// tryFindQueueAtPtr tries to find the target queue in a single inference-states entry.
// Returns the queue pointer if found, or nil if not found (without error).
func tryFindQueueAtPtr(client station.StationClient, inferencePtr uintn.UintN, targetQueue string) (uintn.UintN, error) {
	res := client.ReadFromOffset(InferenceStatesQueue, inferencePtr, 1, 1, 10)

	var entry, ok = <-res.Data
	if !ok {
		if res.Err != nil {
			return nil, fmt.Errorf("failed to read inference-states: %w", res.Err)
		}
		return nil, nil // No entry found, not an error
	}

	var rxReader = inference.NewInferenceRxReader()
	if err := rxReader.Unmarshal(entry.Data); err != nil {
		return nil, nil // Failed to unmarshal, not an error
	}

	// Search for queue matching by postfix
	for _, e := range rxReader.GetEntries() {
		queueName := e.GetQueue()
		if queueMatchesByPostfix(queueName, targetQueue) {
			ptr, err := uintn.FromLEBytes(e.GetPtr())
			if err != nil {
				return nil, fmt.Errorf("failed to parse ptr for queue %s: %w", queueName, err)
			}
			return ptr, nil
		}
	}

	return nil, nil // Queue not found in this entry
}

// searchForwardForQueue searches forward from startPtr in batches of 10k entries
// until it finds the target queue. Returns the queue pointer.
func searchForwardForQueue(client station.StationClient, startPtr, endPtr uintn.UintN, targetQueue string) (uintn.UintN, error) {
	currentPtr := startPtr

	for {
		// Check if we've reached the end
		if currentPtr.Greater(endPtr) || currentPtr.Equal(endPtr) {
			return nil, fmt.Errorf("target queue %s not found between %v and %v", targetQueue, startPtr, endPtr)
		}

		log.Info().Msgf("Searching forward for queue %s from %v (batch of %d)", targetQueue, currentPtr, searchStep)

		// Read 10k entries at a time
		res := client.ReadFromOffset(InferenceStatesQueue, currentPtr, searchStep, 1, 30)

		entriesRead := 0
		var lastPtr uintn.UintN

		for entry := range res.Data {
			entriesRead++
			lastPtr = entry.ID.ID

			var rxReader = inference.NewInferenceRxReader()
			if err := rxReader.Unmarshal(entry.Data); err != nil {
				log.Warn().Err(err).Msgf("Failed to unmarshal InferenceRx, skipping")
				continue
			}

			// Search for queue matching by postfix
			for _, e := range rxReader.GetEntries() {
				queueName := e.GetQueue()
				if queueMatchesByPostfix(queueName, targetQueue) {
					ptr, err := uintn.FromLEBytes(e.GetPtr())
					if err != nil {
						return nil, fmt.Errorf("failed to parse ptr for queue %s: %w", queueName, err)
					}
					log.Info().Msgf("Found queue %s at inference-states pointer %v -> target queue pointer %v", targetQueue, lastPtr, ptr)
					return ptr, nil
				}
			}
		}

		if res.Err != nil {
			return nil, fmt.Errorf("failed to read inference-states: %w", res.Err)
		}

		if entriesRead == 0 {
			return nil, fmt.Errorf("no entries found at pointer %v", currentPtr)
		}

		// Move to the next batch
		var err error
		currentPtr, err = lastPtr.Add(uintn.FromU8(1))
		if err != nil {
			return nil, fmt.Errorf("failed to increment pointer: %w", err)
		}
	}
}

// searchBackwardForQueue searches backward from endPtr in batches of 10k entries
// until it finds the target queue. Returns the queue pointer.
// Note: Entries are read in chronological order (forward), so we read from (currentPtr - 10k)
// up to currentPtr and keep the LAST match found (which is closest to currentPtr).
func searchBackwardForQueue(client station.StationClient, startPtr, endPtr uintn.UintN, targetQueue string) (uintn.UintN, error) {
	// Start from endPtr and move backward in chunks
	currentPtr := endPtr

	for {
		// Check if we've gone past the start
		if currentPtr.Less(startPtr) || currentPtr.Equal(startPtr) {
			return nil, fmt.Errorf("target queue %s not found between %v and %v", targetQueue, startPtr, endPtr)
		}

		// Calculate batch start (currentPtr - searchStep, but not before startPtr)
		batchStart := startPtr
		step := uintn.FromU64(searchStep)
		stepPtr, err := uintn.Sub(currentPtr, step)
		if err == nil && stepPtr.Greater(startPtr) {
			batchStart = stepPtr
		}

		log.Info().Msgf("Searching backward for queue %s from %v (reading up to %v)", targetQueue, batchStart, currentPtr)

		// Read batch from batchStart forward
		// Entries will come in chronological order, so we process until we reach currentPtr
		res := client.ReadFromOffset(InferenceStatesQueue, batchStart, searchStep, 1, 30)

		var lastFoundPtr uintn.UintN
		var lastFoundQueuePtr uintn.UintN
		foundInBatch := false
		var lastEntryPtr uintn.UintN

		for entry := range res.Data {
			entryPtr := entry.ID.ID
			lastEntryPtr = entryPtr

			// Stop reading if we've passed currentPtr
			if entryPtr.Greater(currentPtr) {
				break
			}

			var rxReader = inference.NewInferenceRxReader()
			if err := rxReader.Unmarshal(entry.Data); err != nil {
				log.Warn().Err(err).Msgf("Failed to unmarshal InferenceRx, skipping")
				continue
			}

			// Search for queue matching by postfix
			// Keep updating lastFound so we get the match closest to currentPtr
			for _, e := range rxReader.GetEntries() {
				queueName := e.GetQueue()
				if queueMatchesByPostfix(queueName, targetQueue) {
					ptr, err := uintn.FromLEBytes(e.GetPtr())
					if err != nil {
						continue
					}
					// This is a match - keep it (we want the last one before currentPtr)
					lastFoundPtr = entryPtr
					lastFoundQueuePtr = ptr
					foundInBatch = true
				}
			}
		}

		if res.Err != nil {
			return nil, fmt.Errorf("failed to read inference-states: %w", res.Err)
		}

		if foundInBatch {
			log.Info().Msgf("Found queue %s at inference-states pointer %v -> target queue pointer %v", targetQueue, lastFoundPtr, lastFoundQueuePtr)
			return lastFoundQueuePtr, nil
		}

		// No match in this batch, move backward to previous batch
		// Use batchStart as new currentPtr (or lastEntryPtr if we read something)
		if !lastEntryPtr.IsZero() && lastEntryPtr.Less(currentPtr) {
			currentPtr = lastEntryPtr
		} else {
			currentPtr = batchStart
		}

		// Prevent infinite loop if batchStart == startPtr
		if currentPtr.Equal(startPtr) {
			return nil, fmt.Errorf("target queue %s not found between %v and %v", targetQueue, startPtr, endPtr)
		}
	}
}

// queueMatchesByPostfix checks if queueName ends with targetQueue
// For example: "/hash/inference/normvla" matches "inference/normvla"
func queueMatchesByPostfix(queueName, targetQueue string) bool {
	if queueName == targetQueue {
		return true
	}
	// Check if queueName ends with "/" + targetQueue
	suffix := "/" + targetQueue
	if len(queueName) > len(suffix) && queueName[len(queueName)-len(suffix):] == suffix {
		return true
	}
	return false
}

// FetchQueueBounds fetches the target queue pointers for both from and to
// inference-states entries in parallel. First tries single entry check,
// then falls back to batch search if needed.
func FetchQueueBounds(client station.StationClient, fromPtr, toPtr uintn.UintN, targetQueue string) (*QueueBounds, error) {
	var wg sync.WaitGroup
	var fromResult, toResult uintn.UintN
	var fromErr, toErr error

	wg.Add(2)

	// Search for 'from' bound (search forward)
	go func() {
		defer wg.Done()

		// First try single entry at exact pointer
		log.Info().Msgf("Checking for queue %s at from pointer %v", targetQueue, fromPtr)
		ptr, err := tryFindQueueAtPtr(client, fromPtr, targetQueue)
		if err != nil {
			fromErr = fmt.Errorf("failed to check 'from' bound: %w", err)
			return
		}
		if ptr != nil {
			log.Info().Msgf("Found queue %s immediately at from pointer %v", targetQueue, fromPtr)
			fromResult = ptr
			return
		}

		// Not found, search forward
		log.Info().Msgf("Queue %s not found at from pointer, searching forward", targetQueue)
		ptr, err = searchForwardForQueue(client, fromPtr, toPtr, targetQueue)
		if err != nil {
			fromErr = fmt.Errorf("failed to search forward from 'from' bound: %w", err)
			return
		}
		fromResult = ptr
	}()

	// Search for 'to' bound (search backward)
	go func() {
		defer wg.Done()

		// First try single entry at exact pointer
		log.Info().Msgf("Checking for queue %s at to pointer %v", targetQueue, toPtr)
		ptr, err := tryFindQueueAtPtr(client, toPtr, targetQueue)
		if err != nil {
			toErr = fmt.Errorf("failed to check 'to' bound: %w", err)
			return
		}
		if ptr != nil {
			log.Info().Msgf("Found queue %s immediately at to pointer %v", targetQueue, toPtr)
			toResult = ptr
			return
		}

		// Not found, search backward
		log.Info().Msgf("Queue %s not found at to pointer, searching backward", targetQueue)
		ptr, err = searchBackwardForQueue(client, fromPtr, toPtr, targetQueue)
		if err != nil {
			toErr = fmt.Errorf("failed to search backward from 'to' bound: %w", err)
			return
		}
		toResult = ptr
	}()

	wg.Wait()

	if fromErr != nil {
		return nil, fmt.Errorf("failed to fetch 'from' bound: %w", fromErr)
	}
	if toErr != nil {
		return nil, fmt.Errorf("failed to fetch 'to' bound: %w", toErr)
	}

	return &QueueBounds{
		From: fromResult,
		To:   toResult,
	}, nil
}
