package main

import (
	"flag"
	"fmt"
	console_tool "norma_core/shared/console-tool"
	"norma_core/software/station/bin/dataset-generator/internal"
	"norma_core/software/station/bin/dataset-generator/internal/ui"
	"norma_core/software/station/shared/station"
	"time"

	"github.com/norma-core/normfs/normfs_go/uintn"

	tea "github.com/charmbracelet/bubbletea"
	"github.com/rs/zerolog/log"
)

//-build-me-for:linux
//-build-me-for:arm
//-build-me-for:freebsd
//-build-me-for:osx

var robot = flag.String("robot", "", "robot address/ip")
var queue = flag.String("queue", "inference/normvla", "streamfs queue to read from")
var from = flag.String("from", "", "inference-states queue start pointer")
var to = flag.String("to", "", "inference-states queue end pointer")
var output = flag.String("output", "", "output dataset path")
var task = flag.String("task", "", "task description")
var episodeDuration = flag.Uint64("episode.duration", 45, "episode duration in seconds")
var episodeMinCommands = flag.Uint64("episode.min-commands", 100, "minimum commands for valid episode")
var noTui = flag.Bool("no-tui", false, "disable TUI and use plain logging")

func formatBytes(bytes uint64) string {
	const (
		KB = 1024
		MB = KB * 1024
		GB = MB * 1024
	)
	switch {
	case bytes >= GB:
		return fmt.Sprintf("%.2f GB", float64(bytes)/GB)
	case bytes >= MB:
		return fmt.Sprintf("%.2f MB", float64(bytes)/MB)
	case bytes >= KB:
		return fmt.Sprintf("%.2f KB", float64(bytes)/KB)
	default:
		return fmt.Sprintf("%d B", bytes)
	}
}

func main() {
	console_tool.ConsoleInit("dataset-generator")

	if *from == "" || *to == "" {
		log.Panic().Msg("--from and --to are required")
	}
	if *output == "" {
		log.Panic().Msg("--output is required")
	}
	if *task == "" {
		log.Panic().Msg("--task is required")
	}

	fromPtr, err := uintn.ParseDecimal(*from)
	if err != nil {
		log.Panic().Err(err).Msg("Failed to parse --from")
	}

	toPtr, err := uintn.ParseDecimal(*to)
	if err != nil {
		log.Panic().Err(err).Msg("Failed to parse --to")
	}

	params := internal.Params{
		Robot:    *robot,
		Queue:    *queue,
		Task:     *task,
		Duration: time.Duration(*episodeDuration) * time.Second,
		Output:   *output,
		From:     fromPtr,
		To:       toPtr,
	}

	if *noTui {
		ui.RunWithoutTUI()
		if err := runProcessing(nil, params); err != nil {
			log.Fatal().Err(err).Msg("Processing failed")
		}
		return
	}

	runner := ui.NewRunner(params)
	program := runner.Program()

	runner.SetupLogging()

	go func() {
		defer func() {
			if p := recover(); p != nil {
				ui.RestoreTerminal(program)
				panic(p)
			}
		}()
		err := runProcessing(program, params)
		program.Send(internal.DoneMsg{Err: err})
	}()

	finalModel, err := runner.Run()

	// Restore console logging after TUI exits
	ui.RunWithoutTUI()

	if err != nil {
		log.Fatal().Err(err).Msg("TUI error")
	}

	// Print summary to terminal after alt screen clears
	finalModel.PrintSummary()

	if finalModel.Err() != nil {
		log.Fatal().Err(finalModel.Err()).Msg("Processing failed")
	}
}

func runProcessing(program *tea.Program, params internal.Params) error {
	client, err := station.NewStationClient(params.Robot)
	if err != nil {
		return fmt.Errorf("failed to create station client: %w", err)
	}

	log.Info().Msgf("Fetching queue bounds for %s from inference-states [%v -> %v]", params.Queue, params.From, params.To)

	bounds, err := internal.FetchQueueBounds(client, params.From, params.To, params.Queue)
	if err != nil {
		return fmt.Errorf("failed to fetch queue bounds: %w", err)
	}

	log.Info().Msgf("Target queue %s bounds: [%v -> %v]", params.Queue, bounds.From, bounds.To)

	detector := internal.NewEpisodeDetector(internal.EpisodeDetectorConfig{
		Path:        params.Output,
		Duration:    params.Duration,
		Task:        params.Task,
		MinCommands: *episodeMinCommands,
		Program:     program,
	})

	frames, errChan := internal.StreamFrames(client, params.Queue, bounds)

	stamp := time.Now()
	var totalBytes uint64
	var count uint64

	for frame := range frames {
		totalBytes += uint64(len(frame.SourceBytes()))
		count++

		detector.ProcessFrame(frame)

		if program != nil && count%500 == 0 {
			currentPos, err := uintn.FromLEBytes(frame.GetGlobalFrameId())
			if err == nil {
				program.Send(internal.ProgressMsg{
					Current:         currentPos,
					Total:           bounds.To,
					FramesProcessed: count,
				})
			}
		}

		if count%1000 == 0 {
			log.Info().Msgf("Processed %d frames, %s so far, speed = %v/s",
				count, formatBytes(totalBytes),
				formatBytes(uint64(float64(totalBytes)/time.Since(stamp).Seconds())))
		}
	}

	if err := <-errChan; err != nil {
		return fmt.Errorf("failed to read range: %w", err)
	}

	detector.Finalize()

	// Send final progress update to ensure 100% is shown
	if program != nil {
		program.Send(internal.ProgressMsg{
			Current:         params.To,
			Total:           params.To,
			FramesProcessed: count,
		})
	}

	elapsed := time.Since(stamp)
	log.Info().Msgf("Processed %d frames, %s in %v (%.2f MB/s)",
		count, formatBytes(totalBytes), elapsed, float64(totalBytes)/elapsed.Seconds()/1024.0/1024.0)

	return nil
}
