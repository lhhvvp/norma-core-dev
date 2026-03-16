package internal

import (
	"time"

	"github.com/norma-core/normfs/normfs_go/uintn"
)

// ProgressMsg updates the progress bar and frame count
type ProgressMsg struct {
	Current         uintn.UintN
	Total           uintn.UintN
	FramesProcessed uint64
}

// EpisodeStartedMsg signals an episode started
type EpisodeStartedMsg struct {
	Index uint64
}

// EpisodeSavedMsg signals an episode was saved
type EpisodeSavedMsg struct {
	Index  uint64
	Frames int
}

// EpisodeDiscardedMsg signals an episode was discarded
type EpisodeDiscardedMsg struct {
	Index   uint64
	Reason  string
	FrameId string
}

// EpisodeSkipMarkedMsg signals an episode is marked for skip
type EpisodeSkipMarkedMsg struct {
	Index  uint64
	Reason string
}

// FrameBufferedMsg signals a frame was buffered
type FrameBufferedMsg struct{}

// LogMsg adds a log line to the TUI
type LogMsg struct {
	Level   string
	Message string
	Time    time.Time
}

// DoneMsg signals processing is complete
type DoneMsg struct {
	Err error
}

// TickMsg for periodic updates
type TickMsg time.Time

// Params holds runtime parameters for display
type Params struct {
	Robot    string
	Queue    string
	Task     string
	Duration time.Duration
	Output   string
	From     uintn.UintN
	To       uintn.UintN
}
