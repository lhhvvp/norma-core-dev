package ui

import (
	"fmt"
	"norma_core/software/station/bin/dataset-generator/internal"
	"strings"
	"time"

	"github.com/norma-core/normfs/normfs_go/uintn"

	tea "github.com/charmbracelet/bubbletea"
	"github.com/charmbracelet/lipgloss"
)

const minLogLines = 3

var (
	titleStyle         = lipgloss.NewStyle().Bold(true).Foreground(lipgloss.Color("205"))
	progressBarStyle   = lipgloss.NewStyle().Foreground(lipgloss.Color("46"))
	progressEmptyStyle = lipgloss.NewStyle().Foreground(lipgloss.Color("240"))
	statsStyle         = lipgloss.NewStyle().Foreground(lipgloss.Color("252"))
	paramsStyle        = lipgloss.NewStyle().Foreground(lipgloss.Color("245"))
	logInfoStyle       = lipgloss.NewStyle().Foreground(lipgloss.Color("252"))
	logWarnStyle       = lipgloss.NewStyle().Foreground(lipgloss.Color("214"))
	logErrorStyle      = lipgloss.NewStyle().Foreground(lipgloss.Color("196"))
	separatorStyle     = lipgloss.NewStyle().Foreground(lipgloss.Color("240"))
)

type Model struct {
	params  internal.Params
	current uintn.UintN

	framesProcessed     uint64
	episodeCount        uint64
	episodesDiscarded   uint64
	discardedReasons    []string
	discardedFrameIds   []string
	currentEpisodeIndex uint64
	framesInEpisode     int
	totalFramesSaved    int
	episodeSkipReason   string

	startTime time.Time
	logs      []internal.LogMsg

	done bool
	err  error

	width  int
	height int
}

func NewModel(params internal.Params) Model {
	return Model{
		params:    params,
		current:   params.From,
		startTime: time.Now(),
		logs:      make([]internal.LogMsg, 0, 32),
		width:     80,
		height:    24,
	}
}

func (m Model) Err() error {
	return m.err
}

// PrintSummary outputs final statistics to stdout after TUI exits
func (m Model) PrintSummary() {
	if m.err != nil {
		fmt.Printf("\n❌ Processing failed: %v\n\n", m.err)
		return
	}

	fmt.Printf("\n✅ Processing complete!\n\n")
	fmt.Printf("Progress: 100%% - %s/%s\n", m.params.To.String(), m.params.To.String())

	elapsed := time.Since(m.startTime)
	var rate float64
	if elapsed > 0 && m.framesProcessed > 0 {
		rate = float64(m.framesProcessed) / elapsed.Seconds()
	}

	fmt.Printf("Episodes saved: %d\n", m.episodeCount)
	fmt.Printf("Episodes discarded: %d\n", m.episodesDiscarded)
	if len(m.discardedReasons) > 0 {
		fmt.Println("Discarded reasons:")
		for i, reason := range m.discardedReasons {
			fmt.Printf("- Frame %s: %s\n", m.discardedFrameIds[i], reason)
		}
	}
	fmt.Printf("Total frames saved: %d\n", m.totalFramesSaved)
	fmt.Printf("Frames processed: %d\n", m.framesProcessed)
	fmt.Printf("Rate: %.0f frames/sec\n", rate)
	fmt.Printf("Elapsed: %s\n", formatDuration(elapsed))
	fmt.Printf("Output: %s\n", m.params.Output)
	fmt.Printf("Task: %s\n\n", m.params.Task)
}

func (m Model) Init() tea.Cmd {
	return tickCmd()
}

func tickCmd() tea.Cmd {
	return tea.Tick(time.Second, func(t time.Time) tea.Msg {
		return internal.TickMsg(t)
	})
}

func (m Model) Update(msg tea.Msg) (tea.Model, tea.Cmd) {
	switch msg := msg.(type) {
	case tea.KeyMsg:
		if m.done {
			return m, tea.Quit
		}
		switch msg.String() {
		case "ctrl+c", "q":
			return m, tea.Quit
		}

	case tea.WindowSizeMsg:
		m.width = msg.Width
		m.height = msg.Height

	case internal.TickMsg:
		return m, tickCmd()

	case internal.ProgressMsg:
		m.current = msg.Current
		m.framesProcessed = msg.FramesProcessed

	case internal.EpisodeStartedMsg:
		m.currentEpisodeIndex = msg.Index
		m.framesInEpisode = 0
		m.episodeSkipReason = ""

	case internal.EpisodeSkipMarkedMsg:
		m.episodeSkipReason = msg.Reason

	case internal.EpisodeSavedMsg:
		m.episodeCount++
		m.totalFramesSaved += msg.Frames
		m.framesInEpisode = 0
		m.episodeSkipReason = ""

	case internal.EpisodeDiscardedMsg:
		m.episodesDiscarded++
		m.discardedReasons = append(m.discardedReasons, msg.Reason)
		m.discardedFrameIds = append(m.discardedFrameIds, msg.FrameId)
		m.framesInEpisode = 0
		m.episodeSkipReason = ""

	case internal.FrameBufferedMsg:
		m.framesInEpisode++

	case internal.LogMsg:
		m.logs = append(m.logs, msg)
		maxLogs := m.maxLogLines()
		if len(m.logs) > maxLogs {
			m.logs = m.logs[len(m.logs)-maxLogs:]
		}

	case internal.DoneMsg:
		m.done = true
		m.err = msg.Err
		return m, tea.Quit
	}

	return m, nil
}

func (m Model) View() string {
	var b strings.Builder

	b.WriteString(m.renderProgress())
	b.WriteString("\n")
	b.WriteString(m.renderSeparator())
	b.WriteString("\n")

	b.WriteString(m.renderStats())
	b.WriteString("\n")
	b.WriteString(m.renderSeparator())
	b.WriteString("\n")

	b.WriteString(m.renderParams())
	b.WriteString("\n")
	b.WriteString(m.renderSeparator())
	b.WriteString("\n")

	b.WriteString(m.renderLogs())

	if m.done {
		b.WriteString("\n")
		if m.err != nil {
			b.WriteString(logErrorStyle.Render(fmt.Sprintf("Error: %v", m.err)))
		} else {
			b.WriteString(titleStyle.Render("Processing complete!"))
		}
	}

	return b.String()
}

func (m Model) renderProgress() string {
	progress := uintn.Progress(m.current, m.params.From, m.params.To)
	percent := progress * 100

	currentStr := m.current.String()
	toStr := m.params.To.String()

	fixedWidth := 22 + len(currentStr) + len(toStr)
	barWidth := m.width - fixedWidth
	if barWidth < 10 {
		barWidth = 10
	}

	filled := int(float64(barWidth) * progress)
	empty := barWidth - filled

	bar := progressBarStyle.Render(strings.Repeat("█", filled)) +
		progressEmptyStyle.Render(strings.Repeat("░", empty))

	return fmt.Sprintf("Progress: [%s] %5.1f%%  %s/%s", bar, percent, currentStr, toStr)
}

func (m Model) renderStats() string {
	episodeStatus := "working"
	if m.episodeSkipReason != "" {
		episodeStatus = "skip: " + m.episodeSkipReason
	}
	episodeLine := fmt.Sprintf("Episode: #%d (%s)    Frames in episode: %d    Total saved: %d",
		m.currentEpisodeIndex, episodeStatus, m.framesInEpisode, m.totalFramesSaved)

	elapsed := time.Since(m.startTime)
	var rate float64
	if elapsed > 0 && m.framesProcessed > 0 {
		rate = float64(m.framesProcessed) / elapsed.Seconds()
	}

	rateLine := fmt.Sprintf("Rate: %.0f frames/sec   Elapsed: %s   Episodes saved: %d   Episodes discarded: %d   Frames processed: %d",
		rate, formatDuration(elapsed), m.episodeCount, m.episodesDiscarded, m.framesProcessed)

	return statsStyle.Render(episodeLine + "\n" + rateLine)
}

func (m Model) renderParams() string {
	line1 := fmt.Sprintf("Robot: %s    Queue: %s    Duration: %s",
		m.params.Robot, m.params.Queue, m.params.Duration)

	task := m.params.Task
	if len(task) > 50 {
		task = task[:47] + "..."
	}
	line2 := fmt.Sprintf("Output: %s    Task: %s", m.params.Output, task)

	return paramsStyle.Render(line1 + "\n" + line2)
}

func (m Model) renderLogs() string {
	if len(m.logs) == 0 {
		return paramsStyle.Render("(no logs yet)")
	}

	maxLogs := m.maxLogLines()
	logs := m.logs
	if len(logs) > maxLogs {
		logs = logs[len(logs)-maxLogs:]
	}

	var lines []string
	for _, log := range logs {
		var style lipgloss.Style
		switch log.Level {
		case "WRN", "WARN":
			style = logWarnStyle
		case "ERR", "ERROR":
			style = logErrorStyle
		default:
			style = logInfoStyle
		}

		msg := log.Message
		maxLen := m.width - 10
		if maxLen > 0 && len(msg) > maxLen {
			msg = msg[:maxLen-3] + "..."
		}

		lines = append(lines, style.Render(fmt.Sprintf("[%s] %s", log.Level, msg)))
	}

	return strings.Join(lines, "\n")
}

func (m Model) renderSeparator() string {
	return separatorStyle.Render(strings.Repeat("─", m.width))
}

func (m Model) maxLogLines() int {
	available := m.height - 12
	if available < minLogLines {
		return minLogLines
	}
	return available
}

func formatDuration(d time.Duration) string {
	h := int(d.Hours())
	m := int(d.Minutes()) % 60
	s := int(d.Seconds()) % 60
	return fmt.Sprintf("%02d:%02d:%02d", h, m, s)
}
