package main

import (
	"bytes"
	"context"
	"flag"
	"fmt"
	"image"
	"image/jpeg"
	"io"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"strings"
	"sync"
	"time"

	console_tool "norma_core/shared/console-tool"

	"github.com/apache/arrow/go/v18/arrow/array"
	"github.com/apache/arrow/go/v18/arrow/memory"
	"github.com/apache/arrow/go/v18/parquet/file"
	"github.com/apache/arrow/go/v18/parquet/pqarrow"
	"github.com/fogleman/gg"
	"github.com/norma-core/normfs/normfs_go/uintn"
	"github.com/rs/zerolog/log"
)

var input = flag.String("input", "", "input parquet file path")
var output = flag.String("output", "output.mp4", "output video file path")
var frameDuration = flag.Int("frame-duration", 100, "frame duration in milliseconds")
var workers = flag.Int("workers", 0, "number of worker goroutines (0 = num CPUs)")
var gridCols = flag.Int("grid-cols", 3, "number of columns in grid")
var gridRows = flag.Int("grid-rows", 3, "number of rows in grid")
var renderState = flag.Bool("render-state", true, "enable 3D state rendering (disable for robots without 3D view)")
var renderHost = flag.String("render-host", "localhost", "render service host")

const (
	renderPortStart = 8000
	renderPortEnd   = 8008
	workersPerPort  = 32
)

// HTTP client configured for high concurrency
var httpClient = &http.Client{
	Transport: &http.Transport{
		MaxIdleConns:        300,
		MaxIdleConnsPerHost: 50,
		MaxConnsPerHost:     50,
		IdleConnTimeout:     90 * time.Second,
	},
	Timeout: 30 * time.Second,
}

type renderJob struct {
	frames          []Frame // Grid of frames to render together
	episodeNums     []int   // Episode numbers for each frame in the grid
	frameIndex      int
	outputDir       string
	render3DService chan render3DRequest
}

type renderResult struct {
	frameIndex int
	err        error
}

type render3DRequest struct {
	key      string
	joints   []Joint
	response chan image.Image
}

type Episode struct {
	frames     []Frame
	startNS    uint64
	episodeNum int
}

type Frame struct {
	EpisodeStartNS               uint64  `parquet:"episode_start_ns"`
	GlobalFrameID                []byte  `parquet:"global_frame_id"`
	TimestampNsSinceEpisodeStart uint64  `parquet:"timestamp_ns_since_episode_start"`
	Joints                       []Joint `parquet:"joints"`
	Images                       []Image `parquet:"images"`
	Task                         string  `parquet:"task"`
}

type Joint struct {
	RangeMin     uint32  `parquet:"range_min"`
	RangeMax     uint32  `parquet:"range_max"`
	Position     uint32  `parquet:"position"`
	PositionNorm float32 `parquet:"position_norm"`
	Goal         uint32  `parquet:"goal"`
	GoalNorm     float32 `parquet:"goal_norm"`
	CurrentMA    uint32  `parquet:"current_ma"`
	Velocity     uint32  `parquet:"velocity"`
}

type Image struct {
	JPEG []byte `parquet:"jpeg"`
}

func main() {
	console_tool.ConsoleInit("dataset-mp4")
	flag.Parse()

	if *input == "" {
		log.Fatal().Msg("input parquet file is required")
	}

	log.Info().Msgf("Reading parquet file: %s", *input)

	// Open parquet file with Arrow
	rdr, err := file.OpenParquetFile(*input, false)
	if err != nil {
		log.Fatal().Err(err).Msg("failed to open parquet file")
	}
	defer rdr.Close()

	arrowRdr, err := pqarrow.NewFileReader(rdr, pqarrow.ArrowReadProperties{BatchSize: 1024}, memory.DefaultAllocator)
	if err != nil {
		log.Fatal().Err(err).Msg("failed to create arrow reader")
	}

	log.Info().Msgf("Parquet file has %d rows", rdr.NumRows())

	// Create temporary directory for frames
	tmpDir, err := os.MkdirTemp("", "dataset-mp4-*")
	if err != nil {
		log.Fatal().Err(err).Msg("failed to create temp directory")
	}
	defer os.RemoveAll(tmpDir)

	log.Info().Msgf("Temporary directory: %s", tmpDir)

	// First pass: collect all episodes
	log.Info().Msg("Collecting episodes...")
	episodes := collectEpisodes(arrowRdr)
	log.Info().Msgf("Collected %d episodes", len(episodes))

	// Start 3D render service with multiple ports (if enabled)
	var render3DService chan render3DRequest
	var render3DWg sync.WaitGroup

	if *renderState {
		numPorts := renderPortEnd - renderPortStart + 1
		totalWorkers := numPorts * workersPerPort
		log.Info().Msgf("Starting %d 3D render workers across %d ports (%d workers per port)",
			totalWorkers, numPorts, workersPerPort)

		render3DService = make(chan render3DRequest, totalWorkers*2)

		// Create workers for each port
		for port := renderPortStart; port <= renderPortEnd; port++ {
			for w := 0; w < workersPerPort; w++ {
				render3DWg.Add(1)
				go func(p int) {
					defer render3DWg.Done()
					for req := range render3DService {
						img, err := fetch3DRender(req.joints, p)
						if err == nil {
							req.response <- img
						}
						close(req.response)
					}
				}(port)
			}
		}
	} else {
		log.Info().Msg("3D rendering disabled")
	}

	// Group episodes into grids
	gridSize := (*gridCols) * (*gridRows)
	numGrids := (len(episodes) + gridSize - 1) / gridSize
	log.Info().Msgf("Rendering %d grids of %dx%d (%d episodes each)", numGrids, *gridCols, *gridRows, gridSize)

	// Determine number of workers
	numWorkers := *workers
	if numWorkers == 0 {
		numWorkers = runtime.NumCPU()
	}
	log.Info().Msgf("Using %d worker threads", numWorkers)

	// Create channels for worker pool
	jobs := make(chan renderJob, numWorkers*2)
	results := make(chan renderResult, numWorkers*2)

	// Start worker pool
	var wg sync.WaitGroup
	for w := 0; w < numWorkers; w++ {
		wg.Add(1)
		go func(workerID int) {
			defer wg.Done()
			for job := range jobs {
				err := renderGridFrame(job.frames, job.episodeNums, job.frameIndex, job.outputDir, *gridCols, *gridRows, job.render3DService, *renderState)
				results <- renderResult{
					frameIndex: job.frameIndex,
					err:        err,
				}
			}
		}(w)
	}

	// Start result collector
	var resultWg sync.WaitGroup
	resultWg.Add(1)
	var renderErrors int
	go func() {
		defer resultWg.Done()
		renderedCount := 0
		for result := range results {
			if result.err != nil {
				log.Error().Err(result.err).Msgf("failed to render frame %d", result.frameIndex)
				renderErrors++
			} else {
				renderedCount++
				if renderedCount%100 == 0 {
					log.Info().Msgf("Rendered %d grid frames", renderedCount)
				}
			}
		}
	}()

	// Process each grid
	frameIndex := 0
	for gridIdx := 0; gridIdx < numGrids; gridIdx++ {
		startEp := gridIdx * gridSize
		endEp := min(startEp+gridSize, len(episodes))
		gridEpisodes := episodes[startEp:endEp]

		log.Info().Msgf("Rendering grid %d/%d with episodes %d-%d", gridIdx+1, numGrids, startEp+1, endEp)

		// Find max frame count in this grid
		maxFrames := 0
		for _, ep := range gridEpisodes {
			if len(ep.frames) > maxFrames {
				maxFrames = len(ep.frames)
			}
		}

		// Render each frame index across all episodes in the grid
		for frameIdx := 0; frameIdx < maxFrames; frameIdx++ {
			gridFrames := make([]Frame, gridSize)
			episodeNums := make([]int, gridSize)

			// Collect frames for this grid position
			for i := 0; i < gridSize; i++ {
				if i < len(gridEpisodes) && frameIdx < len(gridEpisodes[i].frames) {
					gridFrames[i] = gridEpisodes[i].frames[frameIdx]
					episodeNums[i] = gridEpisodes[i].episodeNum
				} else {
					// Empty frame (black screen) if episode doesn't have this frame
					gridFrames[i] = Frame{}
					episodeNums[i] = 0
				}
			}

			// Send to worker pool
			jobs <- renderJob{
				frames:          gridFrames,
				episodeNums:     episodeNums,
				frameIndex:      frameIndex,
				outputDir:       tmpDir,
				render3DService: render3DService,
			}
			frameIndex++
		}
	}

	log.Info().Msgf("Total grid frames to render: %d", frameIndex)

	// Close jobs channel and wait for workers to finish
	close(jobs)
	wg.Wait()

	// Close results channel and wait for collector
	close(results)
	resultWg.Wait()

	// Close 3D render service and wait for workers (if enabled)
	if *renderState {
		close(render3DService)
		render3DWg.Wait()
	}

	if renderErrors > 0 {
		log.Warn().Msgf("Encountered %d render errors", renderErrors)
	}

	log.Info().Msgf("Total frames rendered: %d", frameIndex)

	// Generate video using ffmpeg
	log.Info().Msg("Generating video with ffmpeg...")
	fps := 1000.0 / float64(*frameDuration)
	if err := generateVideo(tmpDir, *output, fps); err != nil {
		log.Fatal().Err(err).Msg("failed to generate video")
	}

	log.Info().Msgf("Video generated: %s", *output)
}

func collectEpisodes(arrowRdr *pqarrow.FileReader) []Episode {
	batchReader, err := arrowRdr.GetRecordReader(context.Background(), nil, nil)
	if err != nil {
		log.Fatal().Err(err).Msg("failed to create record batch reader")
	}
	defer batchReader.Release()

	var episodes []Episode
	var currentEpisode *Episode
	globalRowIdx := 0

	for batchReader.Next() {
		batch := batchReader.Record()

		episodeStartArr := batch.Column(0).(*array.Uint64)
		idArr := batch.Column(1).(*array.Binary)
		timestampArr := batch.Column(2).(*array.Uint64)
		jointsArr := batch.Column(3).(*array.List)
		imagesArr := batch.Column(4).(*array.List)
		taskArr := batch.Column(5).(*array.String)

		for i := 0; i < int(batch.NumRows()); i++ {
			episodeStart := episodeStartArr.Value(i)

			// Detect episode change
			if currentEpisode == nil || episodeStart != currentEpisode.startNS {
				// Save previous episode if valid
				if currentEpisode != nil && len(currentEpisode.frames) > 0 {
					episodes = append(episodes, *currentEpisode)
					log.Info().Msgf("Collected episode %d with %d frames", currentEpisode.episodeNum, len(currentEpisode.frames))
				}

				// Start new episode
				currentEpisode = &Episode{
					frames:     []Frame{},
					startNS:    episodeStart,
					episodeNum: len(episodes) + 1,
				}
			}

			// Extract frame data
			frame := extractFrameFromBatch(i, episodeStartArr, idArr, timestampArr, jointsArr, imagesArr, taskArr)

			// Check if frame has exactly 2 images
			if len(frame.Images) != 2 {
				// Skip this entire episode
				currentEpisode.frames = nil // Mark as invalid
				globalRowIdx++
				continue
			}

			// Skip if episode already marked as invalid
			if currentEpisode.frames == nil {
				globalRowIdx++
				continue
			}

			currentEpisode.frames = append(currentEpisode.frames, frame)
			globalRowIdx++
		}
	}

	// Save last episode if valid
	if currentEpisode != nil && len(currentEpisode.frames) > 0 {
		episodes = append(episodes, *currentEpisode)
		log.Info().Msgf("Collected episode %d with %d frames", currentEpisode.episodeNum, len(currentEpisode.frames))
	}

	if err := batchReader.Err(); err != nil && err != io.EOF {
		log.Fatal().Err(err).Msg("error reading batches")
	}

	return episodes
}

func fetch3DRender(joints []Joint, port int) (image.Image, error) {
	if len(joints) < 8 {
		return nil, fmt.Errorf("need at least 8 joints, got %d", len(joints))
	}

	// Convert normalized positions (0.0-1.0) to 0-100 range
	angles := make([]string, 8)
	for i := 0; i < 8; i++ {
		angles[i] = fmt.Sprintf("%.0f", joints[i].PositionNorm*100)
	}

	url := fmt.Sprintf("http://%s:%d/render?angles=%s&view=perspective", *renderHost, port, strings.Join(angles, ","))

	resp, err := httpClient.Get(url)
	if err != nil {
		return nil, err
	}
	defer resp.Body.Close()

	if resp.StatusCode != http.StatusOK {
		return nil, fmt.Errorf("render service returned status %d", resp.StatusCode)
	}

	img, err := jpeg.Decode(resp.Body)
	if err != nil {
		return nil, err
	}

	return img, nil
}

func renderGridFrame(frames []Frame, episodeNums []int, frameIndex int, outputDir string, cols, rows int, render3DService chan render3DRequest, enableRendering bool) error {
	singleWidth := 448  // 224*2 for images side by side
	singleHeight := 448 // 224*2 (images top + 3D render/text bottom)
	borderWidth := 2
	canvasWidth := singleWidth*cols + borderWidth*(cols+1)
	canvasHeight := singleHeight*rows + borderWidth*(rows+1)

	// Request all 3D renders for this grid in parallel (if enabled)
	gridSize := cols * rows
	var renders []image.Image
	var responseChans []chan image.Image

	if enableRendering {
		renders = make([]image.Image, gridSize)
		responseChans = make([]chan image.Image, gridSize)

		for i := 0; i < gridSize && i < len(frames); i++ {
			if len(frames[i].Joints) >= 8 && len(frames[i].Images) == 2 {
				responseChans[i] = make(chan image.Image, 1)
				render3DService <- render3DRequest{
					joints:   frames[i].Joints,
					response: responseChans[i],
				}
			}
		}

		// Collect responses
		for i := 0; i < gridSize && i < len(frames); i++ {
			if responseChans[i] != nil {
				img, ok := <-responseChans[i]
				if ok {
					renders[i] = img
				}
			}
		}
	}

	// Create canvas
	dc := gg.NewContext(canvasWidth, canvasHeight)
	dc.SetRGB(0, 0, 0) // Black background
	dc.Clear()

	// Draw grid lines
	dc.SetRGB(0.3, 0.3, 0.3) // Dark gray for grid lines
	dc.SetLineWidth(float64(borderWidth))

	// Vertical lines
	for col := 0; col <= cols; col++ {
		x := float64(col*(singleWidth+borderWidth) + borderWidth/2)
		dc.DrawLine(x, 0, x, float64(canvasHeight))
		dc.Stroke()
	}

	// Horizontal lines
	for row := 0; row <= rows; row++ {
		y := float64(row*(singleHeight+borderWidth) + borderWidth/2)
		dc.DrawLine(0, y, float64(canvasWidth), y)
		dc.Stroke()
	}

	// Render each episode in the grid
	for i, frame := range frames {
		if i >= cols*rows {
			break
		}

		row := i / cols
		col := i % cols
		offsetX := col*(singleWidth+borderWidth) + borderWidth
		offsetY := row*(singleHeight+borderWidth) + borderWidth

		// Render the single frame at this grid position
		if len(frame.Images) == 2 {
			// Draw images (top row)
			for imgIdx := 0; imgIdx < 2; imgIdx++ {
				if len(frame.Images[imgIdx].JPEG) > 0 {
					imgData, err := jpeg.Decode(bytes.NewReader(frame.Images[imgIdx].JPEG))
					if err == nil {
						dc.DrawImage(imgData, offsetX+imgIdx*224, offsetY)
					}
				}
			}

			// Episode number (top-right corner with background)
			if i < len(episodeNums) && episodeNums[i] > 0 {
				dc.SetRGB(1, 1, 1)
				fontSize := 11.0
				if err := dc.LoadFontFace("/System/Library/Fonts/Courier.ttc", fontSize); err == nil {
					episodeText := fmt.Sprintf("Ep%d", episodeNums[i])
					textWidth, textHeight := dc.MeasureString(episodeText)

					// Draw background rectangle
					padding := 3.0
					rectX := float64(offsetX + 448 - int(textWidth) - 8)
					rectY := float64(offsetY + 5)
					dc.SetRGB(0, 0, 0.8) // Dark blue background
					dc.DrawRectangle(rectX-padding, rectY-padding, textWidth+2*padding, textHeight+2*padding)
					dc.Fill()

					// Draw text
					dc.SetRGB(1, 1, 1) // White text
					dc.DrawString(episodeText, rectX, float64(offsetY+int(textHeight)))
				}
			}

			// Draw 3D render (bottom-left) or joint positions if rendering disabled
			if enableRendering {
				if renders[i] != nil {
					dc.DrawImage(renders[i], offsetX, offsetY+224)
				} else if len(frame.Joints) >= 8 {
					// Draw error placeholder if render failed
					dc.SetRGB(0.2, 0, 0)
					dc.DrawRectangle(float64(offsetX), float64(offsetY+224), 224, 224)
					dc.Fill()
				}
			} else if len(frame.Joints) > 0 {
				// Draw joint information in bottom-left when rendering is disabled
				dc.SetRGB(1, 1, 1) // White text
				fontSize := 11.0
				if err := dc.LoadFontFace("/System/Library/Fonts/Courier.ttc", fontSize); err == nil {
					textX := float64(offsetX + 5)
					y := float64(offsetY + 224 + 15)

					// Show first half of joints
					numJointsLeft := (len(frame.Joints) + 1) / 2
					for j := 0; j < numJointsLeft && j < len(frame.Joints); j++ {
						joint := frame.Joints[j]
						text := fmt.Sprintf("M%d P:%d G:%d", j, joint.Position, joint.Goal)
						dc.DrawString(text, textX, y)
						y += fontSize + 2
						text = fmt.Sprintf("   C:%dmA V:%d", joint.CurrentMA, joint.Velocity)
						dc.DrawString(text, textX, y)
						y += fontSize + 3
					}
				}
			}

			// Draw text info (bottom-right)
			dc.SetRGB(1, 1, 1) // White text
			fontSize := 11.0
			if err := dc.LoadFontFace("/System/Library/Fonts/Courier.ttc", fontSize); err == nil {
				textX := float64(offsetX + 224 + 5)
				y := float64(offsetY + 224 + 15)

				if enableRendering {
					// Original compact display when 3D rendering is enabled
					// Frame info
					globalFrameID, _ := uintn.FromBEBytes(frame.GlobalFrameID)
					dc.DrawString(fmt.Sprintf("ID:%s", globalFrameID.String()), textX, y)
					y += fontSize + 2

					// Timestamp
					timestampMs := frame.TimestampNsSinceEpisodeStart / 1_000_000
					dc.DrawString(fmt.Sprintf("T:%dms", timestampMs), textX, y)
					y += fontSize + 4

					// Task (if any)
					if frame.Task != "" {
						dc.DrawString(fmt.Sprintf("Task:%s", frame.Task), textX, y)
						y += fontSize + 4
					}

					// Motors (compact)
					fontSize = 10.0
					dc.LoadFontFace("/System/Library/Fonts/Courier.ttc", fontSize)

					for i, joint := range frame.Joints {
						if i >= 8 { // Limit to 8 motors
							break
						}

						yPos := y + float64(i)*(fontSize+2)
						text := fmt.Sprintf("M%d:P%d C%d", i, joint.Position, joint.CurrentMA)
						dc.DrawString(text, textX, yPos)
					}
				} else {
					// Show second half of joints when rendering is disabled
					numJointsLeft := (len(frame.Joints) + 1) / 2
					for j := numJointsLeft; j < len(frame.Joints); j++ {
						joint := frame.Joints[j]
						text := fmt.Sprintf("M%d P:%d G:%d", j, joint.Position, joint.Goal)
						dc.DrawString(text, textX, y)
						y += fontSize + 2
						text = fmt.Sprintf("   C:%dmA V:%d", joint.CurrentMA, joint.Velocity)
						dc.DrawString(text, textX, y)
						y += fontSize + 3
					}
				}
			}
		}
	}

	// Save frame
	outputPath := filepath.Join(outputDir, fmt.Sprintf("frame_%06d.png", frameIndex))
	return dc.SavePNG(outputPath)
}

func min(vals ...int) int {
	m := vals[0]
	for _, v := range vals[1:] {
		if v < m {
			m = v
		}
	}
	return m
}

func extractFrameFromBatch(idx int, episodeStartArr *array.Uint64, idArr *array.Binary, timestampArr *array.Uint64, jointsArr *array.List, imagesArr *array.List, taskArr *array.String) Frame {
	frame := Frame{
		EpisodeStartNS:               episodeStartArr.Value(idx),
		GlobalFrameID:                idArr.Value(idx),
		TimestampNsSinceEpisodeStart: timestampArr.Value(idx),
		Task:                         taskArr.Value(idx),
	}

	// Extract joints
	if !jointsArr.IsNull(idx) {
		start, end := jointsArr.ValueOffsets(idx)
		structArr := jointsArr.ListValues().(*array.Struct)

		rangeMin := structArr.Field(0).(*array.Uint32)
		rangeMax := structArr.Field(1).(*array.Uint32)
		position := structArr.Field(2).(*array.Uint32)
		positionNorm := structArr.Field(3).(*array.Float32)
		goal := structArr.Field(4).(*array.Uint32)
		goalNorm := structArr.Field(5).(*array.Float32)
		currentMA := structArr.Field(6).(*array.Uint32)
		velocity := structArr.Field(7).(*array.Uint32)

		for j := int(start); j < int(end); j++ {
			joint := Joint{
				RangeMin:     rangeMin.Value(j),
				RangeMax:     rangeMax.Value(j),
				Position:     position.Value(j),
				PositionNorm: positionNorm.Value(j),
				Goal:         goal.Value(j),
				GoalNorm:     goalNorm.Value(j),
				CurrentMA:    currentMA.Value(j),
				Velocity:     velocity.Value(j),
			}
			frame.Joints = append(frame.Joints, joint)
		}
	}

	// Extract images
	if !imagesArr.IsNull(idx) {
		start, end := imagesArr.ValueOffsets(idx)
		structArr := imagesArr.ListValues().(*array.Struct)
		jpegArr := structArr.Field(0).(*array.Binary)

		for j := int(start); j < int(end); j++ {
			img := Image{
				JPEG: jpegArr.Value(j),
			}
			frame.Images = append(frame.Images, img)
		}
	}

	return frame
}

func renderFrame(frame Frame, frameIndex int, outputDir string) error {
	// Fixed layout for 2 images of 224x224
	imgWidth := 224
	imgHeight := 224
	canvasWidth := imgWidth * 2 // Two images side by side
	textHeight := 280           // Space for text below images
	canvasHeight := imgHeight + textHeight

	// Create canvas
	dc := gg.NewContext(canvasWidth, canvasHeight)
	dc.SetRGB(0, 0, 0) // Black background
	dc.Clear()

	// Draw both images side by side (we always expect 2 images)
	for i := 0; i < 2 && i < len(frame.Images); i++ {
		if len(frame.Images[i].JPEG) == 0 {
			continue
		}

		// Decode JPEG
		imgData, err := jpeg.Decode(bytes.NewReader(frame.Images[i].JPEG))
		if err != nil {
			log.Warn().Err(err).Msgf("failed to decode image %d", i)
			continue
		}

		// Draw image at position i * 224
		dc.DrawImage(imgData, i*imgWidth, 0)
	}

	// Draw text overlay
	dc.SetRGB(1, 1, 1) // White text
	fontSize := 14.0
	if err := dc.LoadFontFace("/System/Library/Fonts/Courier.ttc", fontSize); err != nil {
		// Fallback if font loading fails
		log.Warn().Err(err).Msg("failed to load font, using default")
	}

	// Frame info
	y := float64(imgHeight + 20)
	dc.DrawString(fmt.Sprintf("Frame: %d", frameIndex), 10, y)
	y += fontSize + 5

	// Global Frame ID
	globalFrameID, _ := uintn.FromBEBytes(frame.GlobalFrameID)
	dc.DrawString(fmt.Sprintf("Global Frame ID: %s", globalFrameID.String()), 10, y)
	y += fontSize + 5

	// Timestamp
	timestampMs := frame.TimestampNsSinceEpisodeStart / 1_000_000
	dc.DrawString(fmt.Sprintf("Time: %d ms", timestampMs), 10, y)
	y += fontSize + 10

	// Motor positions and current
	dc.DrawString("Motors:", 10, y)
	y += fontSize + 5

	for i, joint := range frame.Joints {
		text := fmt.Sprintf("  Motor %d: Pos=%d Goal=%d Curr=%dmA Vel=%d",
			i, joint.Position, joint.Goal, joint.CurrentMA, joint.Velocity)
		dc.DrawString(text, 10, y)
		y += fontSize + 3
	}

	// Save frame
	outputPath := filepath.Join(outputDir, fmt.Sprintf("frame_%06d.png", frameIndex))
	return dc.SavePNG(outputPath)
}

func renderEpisodeSeparator(frameIndex int, outputDir string, episodeNumber int) error {
	// Create a black frame with episode info
	canvasWidth := 448
	canvasHeight := 524

	dc := gg.NewContext(canvasWidth, canvasHeight)
	dc.SetRGB(0, 0, 0) // Black background
	dc.Clear()

	// Draw episode separator text
	dc.SetRGB(1, 1, 1) // White text
	fontSize := 32.0
	if err := dc.LoadFontFace("/System/Library/Fonts/Courier.ttc", fontSize); err != nil {
		log.Warn().Err(err).Msg("failed to load font")
	}

	text := fmt.Sprintf("Episode %d", episodeNumber)
	w, h := dc.MeasureString(text)
	dc.DrawString(text, (float64(canvasWidth)-w)/2, (float64(canvasHeight)-h)/2)

	outputPath := filepath.Join(outputDir, fmt.Sprintf("frame_%06d.png", frameIndex))
	return dc.SavePNG(outputPath)
}

func generateVideo(inputDir, outputPath string, fps float64) error {
	cmd := exec.Command("ffmpeg",
		"-framerate", fmt.Sprintf("%.2f", fps),
		"-i", filepath.Join(inputDir, "frame_%06d.png"),
		"-c:v", "libx264",
		"-pix_fmt", "yuv420p",
		"-y",
		outputPath,
	)
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr
	return cmd.Run()
}
