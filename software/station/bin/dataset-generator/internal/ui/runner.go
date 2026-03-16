package ui

import (
	"io"
	"norma_core/software/station/bin/dataset-generator/internal"
	"os"
	"strings"
	"time"

	tea "github.com/charmbracelet/bubbletea"
	"github.com/rs/zerolog"
	"github.com/rs/zerolog/log"
)

type Runner struct {
	program *tea.Program
	model   Model
}

func NewRunner(params internal.Params) *Runner {
	model := NewModel(params)
	return &Runner{
		model: model,
	}
}

func (r *Runner) Program() *tea.Program {
	if r.program == nil {
		r.program = tea.NewProgram(r.model, tea.WithAltScreen())
	}
	return r.program
}

// SetupLogging redirects zerolog output to the TUI
func (r *Runner) SetupLogging() {
	writer := &tuiLogWriter{program: r.Program()}
	log.Logger = zerolog.New(writer).With().Timestamp().Logger()
}

// Run starts the TUI and blocks until it exits
func (r *Runner) Run() (Model, error) {
	m, err := r.Program().Run()
	if err != nil {
		return Model{}, err
	}
	if finalModel, ok := m.(Model); ok {
		return finalModel, nil
	}
	return Model{}, nil
}

// tuiLogWriter forwards log writes to TUI
type tuiLogWriter struct {
	program *tea.Program
}

func (w *tuiLogWriter) Write(p []byte) (n int, err error) {
	line := string(p)
	if len(line) > 0 && line[len(line)-1] == '\n' {
		line = line[:len(line)-1]
	}

	// Parse zerolog JSON output
	level := "INF"
	message := line

	// Simple parsing - zerolog outputs JSON like {"level":"info","message":"..."}
	if strings.Contains(line, `"level"`) {
		if strings.Contains(line, `"warn"`) || strings.Contains(line, `"warning"`) {
			level = "WRN"
		} else if strings.Contains(line, `"error"`) {
			level = "ERR"
		} else if strings.Contains(line, `"debug"`) {
			level = "DBG"
		}
	}

	// Extract message if JSON
	if idx := strings.Index(line, `"message":"`); idx != -1 {
		start := idx + 11
		end := strings.Index(line[start:], `"`)
		if end != -1 {
			message = line[start : start+end]
		}
	}

	w.program.Send(internal.LogMsg{
		Level:   level,
		Message: message,
		Time:    time.Now(),
	})
	return len(p), nil
}

// RunWithoutTUI runs processing without TUI (for debugging)
func RunWithoutTUI() {
	log.Logger = zerolog.New(zerolog.ConsoleWriter{
		Out:        os.Stderr,
		TimeFormat: "15:04:05",
	}).With().Timestamp().Logger()
}

// RestoreTerminal should be called if program panics
func RestoreTerminal(program *tea.Program) {
	if program != nil {
		program.Kill()
	}
}

var _ io.Writer = (*tuiLogWriter)(nil)
