package internal

import (
	"fmt"
	"norma_core/software/station/shared/station"
	"norma_core/target/generated-sources/protobuf/drivers/inferences/normvla"

	"github.com/norma-core/normfs/normfs_go/uintn"

	"github.com/rs/zerolog/log"
)

const readBatchSize = 10_000

// StreamFrames reads frames from the station queue within the given bounds
// and returns a channel of parsed FrameReaders. The error channel will
// receive any error that occurs during streaming.
func StreamFrames(client station.StationClient, queue string, bounds *QueueBounds) (<-chan *normvla.FrameReader, <-chan error) {
	frames := make(chan *normvla.FrameReader, 1000)
	errChan := make(chan error, 1)

	go func() {
		defer close(frames)
		defer close(errChan)

		cursor := bounds.From

		for {
			res := client.ReadFromOffset(queue, cursor, readBatchSize, 1, uint(readBatchSize))

			entriesReceived := 0
			for entry := range res.Data {
				entriesReceived++

				// Stop if we've passed the end pointer
				if entry.ID.ID.Greater(bounds.To) {
					return
				}

				frame := normvla.NewFrameReader()
				if err := frame.Unmarshal(entry.Data); err != nil {
					log.Warn().Err(err).Msg("Failed to unmarshal frame, skipping")
					continue
				}
				frames <- frame

				// Update cursor for next batch
				cursor = entry.ID.ID
			}

			if res.Err != nil {
				errChan <- fmt.Errorf("failed to read from queue %s: %w", queue, res.Err)
				return
			}

			// Stream ended before limit - no more data available
			if entriesReceived < readBatchSize {
				return
			}

			// Check if we've reached or passed the end
			if cursor.Greater(bounds.To) || cursor.Equal(bounds.To) {
				return
			}

			// Increment cursor to avoid re-reading the last entry
			var err error
			cursor, err = cursor.Add(uintn.FromU8(1))
			if err != nil {
				errChan <- fmt.Errorf("failed to increment cursor: %w", err)
				return
			}
		}
	}()

	return frames, errChan
}
