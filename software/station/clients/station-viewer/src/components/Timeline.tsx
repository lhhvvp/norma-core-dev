import React, {
  useEffect,
  useMemo,
  useRef,
  useState,
  useCallback,
  memo,
} from 'react';
import HistoryTimelineTrack, { Tick } from './HistoryTimelineTrack';
import TickLabel from './TickLabel';
import { TimelineState, TimelineActions } from '../hooks/useTimelineState';

interface TimelineProps {
  state: TimelineState;
  actions: TimelineActions;
}

const useFrameToPercent = (minFrame: number, maxFrame: number) => {
  return useCallback(
    (frame: number) => {
      const totalFrames = maxFrame - minFrame + 1;
      if (totalFrames <= 1) return 0;
      return ((frame - minFrame) / (totalFrames - 1)) * 100;
    },
    [minFrame, maxFrame],
  );
};

const TimelineTrackWithOverlay = memo(function TimelineTrackWithOverlay({
  minFrame,
  maxFrame,
  ticks,
  selectionRange,
  currentFrame,
  onMouseDown,
  tracksRef,
}: {
  minFrame: number;
  maxFrame: number;
  ticks: Tick[];
  selectionRange: { start: number; end: number } | null;
  currentFrame: number;
  onMouseDown: (e: React.MouseEvent<HTMLDivElement>) => void;
  tracksRef: React.RefObject<HTMLDivElement | null>;
}) {
  const frameToPercent = useFrameToPercent(minFrame, maxFrame);

  return (
    <div
      className="relative"
      ref={tracksRef}
      onMouseDown={onMouseDown}
    >
      <HistoryTimelineTrack
        minFrame={minFrame}
        maxFrame={maxFrame}
        ticks={ticks}
      />

      {selectionRange && (
        <div
          className="absolute top-0 bottom-0 bg-green-500/20 border-x-2 border-green-500 pointer-events-none"
          style={{
            left: `${frameToPercent(
              Math.min(selectionRange.start, selectionRange.end),
            )}%`,
            width: `${Math.abs(
              frameToPercent(selectionRange.end) -
                frameToPercent(selectionRange.start),
            )}%`,
          }}
        />
      )}

      {currentFrame >= minFrame && currentFrame <= maxFrame && (
        <div
          className="absolute top-0 bottom-0 w-0.5 bg-green-500 pointer-events-none"
          style={{ left: `${frameToPercent(currentFrame)}%` }}
        >
          <div className="absolute -top-2 left-1/2 -translate-x-1/2 w-4 h-4 rounded-full bg-green-500 border-2 border-white" />
        </div>
      )}
    </div>
  );
});

const TickLabelsContainer = memo(function TickLabelsContainer({
  ticks,
  minFrame,
  maxFrame,
}: {
  ticks: Tick[];
  minFrame: number;
  maxFrame: number;
}) {
  const frameToPercent = useFrameToPercent(minFrame, maxFrame);
  const majorTicks = useMemo(() => ticks.filter((t) => t.isMajor), [ticks]);

  return (
    <div className="relative h-20 z-10 pointer-events-none mt-2">
      {majorTicks.map((tick) => (
        <TickLabel
          key={tick.frame}
          frame={tick.frame}
          framePercent={frameToPercent(tick.frame)}
        />
      ))}
    </div>
  );
});

const Timeline: React.FC<TimelineProps> = ({ state, actions }) => {
  const { currentFrame, range, selection, isZoomed } = state;
  const { selectFrame, zoomToRange, resetZoom } = actions;

  const [isDragging, setIsDragging] = useState(false);
  const [localSelection, setLocalSelection] = useState<{ start: number; end: number } | null>(null);
  const [trackWidth, setTrackWidth] = useState(0);

  const tracksRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (!tracksRef.current) return;

    const resizeObserver = new ResizeObserver((entries) => {
      for (const entry of entries) {
        setTrackWidth(entry.contentRect.width);
      }
    });

    resizeObserver.observe(tracksRef.current);
    setTrackWidth(tracksRef.current.offsetWidth);

    return () => resizeObserver.disconnect();
  }, []);

  const pixelToFrame = useCallback(
    (pixel: number) => {
      if (!tracksRef.current) return range.min;
      const rect = tracksRef.current.getBoundingClientRect();
      const percent = pixel / rect.width;
      const frame = Math.round(range.min + percent * (range.max - range.min));
      return Math.max(range.min, Math.min(range.max, frame));
    },
    [range.min, range.max],
  );

  const handleMouseDown = useCallback(
    (e: React.MouseEvent<HTMLDivElement>) => {
      if (e.button !== 0) return;
      const frame = pixelToFrame(e.nativeEvent.offsetX);
      setIsDragging(true);
      setLocalSelection({ start: frame, end: frame });
    },
    [pixelToFrame],
  );

  useEffect(() => {
    const handleWindowMouseMove = (e: MouseEvent) => {
      if (!isDragging || !tracksRef.current) return;
      const rect = tracksRef.current.getBoundingClientRect();
      const pixel = e.clientX - rect.left;
      const frame = pixelToFrame(pixel);
      setLocalSelection((prev) => (prev ? { ...prev, end: frame } : null));
    };

    const handleWindowMouseUp = () => {
      if (!isDragging) return;

      setIsDragging(false);

      if (localSelection) {
        const { start, end } = localSelection;
        if (Math.abs(start - end) < 2) {
          setLocalSelection(null);
          selectFrame(start);
        } else {
          zoomToRange(start, end);
          selectFrame(Math.max(start, end));
          setLocalSelection(null);
        }
      }
    };

    if (isDragging) {
      window.addEventListener('mousemove', handleWindowMouseMove);
      window.addEventListener('mouseup', handleWindowMouseUp);
    }

    return () => {
      window.removeEventListener('mousemove', handleWindowMouseMove);
      window.removeEventListener('mouseup', handleWindowMouseUp);
    };
  }, [isDragging, localSelection, pixelToFrame, selectFrame, zoomToRange]);

  const ticks = useMemo(() => {
    const newTicks: Tick[] = [];
    const totalFrames = range.max - range.min + 1;

    if (totalFrames <= 1 || trackWidth === 0) return newTicks;

    const numMajorTicks = Math.max(
      2,
      Math.min(10, Math.floor(trackWidth / 100)),
    );
    const rangeSize = range.max - range.min;

    if (rangeSize <= 0) return newTicks;

    const powerOf10 = Math.pow(
      10,
      Math.floor(Math.log10(rangeSize / numMajorTicks)),
    );
    const majorTickStep = Math.max(
      1,
      Math.round(rangeSize / numMajorTicks / powerOf10) * powerOf10,
    );

    if (majorTickStep === 0) return [];

    const firstMajor = Math.ceil(range.min / majorTickStep) * majorTickStep;
    for (let major = firstMajor; major <= range.max; major += majorTickStep) {
      if (major >= range.min) {
        newTicks.push({ frame: major, isMajor: true });
      }
    }

    const minorTickStep = majorTickStep / 10;
    if (minorTickStep > 0) {
      const firstMinor = Math.ceil(range.min / minorTickStep) * minorTickStep;
      for (let minor = firstMinor; minor <= range.max; minor += minorTickStep) {
        if (minor >= range.min && minor % majorTickStep !== 0) {
          newTicks.push({ frame: minor, isMajor: false });
        }
      }
    }

    return newTicks;
  }, [range.min, range.max, trackWidth]);

  const displaySelection = localSelection || selection;

  return (
    <div className="w-full">
      <div className="flex items-center mb-2">
        {isZoomed && (
          <button
            onClick={resetZoom}
            className="text-white bg-blue-500 hover:bg-blue-700 rounded px-2 py-1 text-xs"
          >
            Reset Zoom
          </button>
        )}
      </div>

      <div className="w-full relative">
        <TimelineTrackWithOverlay
          minFrame={range.min}
          maxFrame={range.max}
          ticks={ticks}
          selectionRange={displaySelection}
          currentFrame={currentFrame}
          onMouseDown={handleMouseDown}
          tracksRef={tracksRef}
        />

        <TickLabelsContainer
          ticks={ticks}
          minFrame={range.min}
          maxFrame={range.max}
        />
      </div>
    </div>
  );
};

export default Timeline;
