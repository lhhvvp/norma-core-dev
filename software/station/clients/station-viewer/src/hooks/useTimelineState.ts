import { useState, useCallback, useEffect, useRef, useMemo } from 'react';
import Long from 'long';
import webSocketManager from '../api/websocket';

const INFERENCE_STATES_QUEUE = 'inference-states';
const MAX_INITIAL_ENTRIES = 500000;

export interface TimelineRange {
  min: number;
  max: number;
}

export interface TimelineState {
  currentFrame: number;
  range: TimelineRange;
  originalRange: TimelineRange;
  selection: { start: number; end: number } | null;
  isLoading: boolean;
  error: string | null;
  isZoomed: boolean;
  isNavigationImmediate: boolean;
}

export interface TimelineActions {
  selectFrame: (frame: number, immediate?: boolean) => void;
  nextFrame: (step?: number, immediate?: boolean) => void;
  prevFrame: (step?: number, immediate?: boolean) => void;
  jumpToStart: (immediate?: boolean) => void;
  jumpToEnd: (immediate?: boolean) => void;
  zoomToRange: (start: number, end: number) => void;
  resetZoom: () => void;
  setSelection: (selection: { start: number; end: number } | null) => void;
  reload: () => Promise<void>;
}

export interface UseTimelineStateReturn {
  state: TimelineState;
  actions: TimelineActions;
}

const DEFAULT_RANGE: TimelineRange = { min: 0, max: 0 };

function clampFrameToRange(frame: number, range: TimelineRange): number {
  return Math.max(range.min, Math.min(range.max, frame));
}

export function useTimelineState(): UseTimelineStateReturn {
  const [currentFrame, setCurrentFrame] = useState(0);
  const [range, setRange] = useState<TimelineRange>(DEFAULT_RANGE);
  const [originalRange, setOriginalRange] = useState<TimelineRange>(DEFAULT_RANGE);
  const [selection, setSelection] = useState<{ start: number; end: number } | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [isNavigationImmediate, setIsNavigationImmediate] = useState(false);

  const isInitialized = useRef(false);
  const retryTimeoutRef = useRef<number | null>(null);

  const fetchRange = useCallback(async () => {
    if (!webSocketManager.isConnected()) {
      return false;
    }

    try {
      const lastEntry = await webSocketManager.normFs.readLastEntry(INFERENCE_STATES_QUEUE);

      const min = 0;
      const max = Long.fromBytesLE(Array.from(lastEntry.id)).toNumber();
      const totalEntries = max - min + 1;

      const newOriginalRange = { min, max };
      setOriginalRange(newOriginalRange);

      if (totalEntries > MAX_INITIAL_ENTRIES) {
        const newMin = max - MAX_INITIAL_ENTRIES + 1;
        setRange({ min: newMin, max });
      } else {
        setRange(newOriginalRange);
      }

      setCurrentFrame(max);
      setError(null);
      return true;
    } catch (err) {
      console.error('Failed to get frame range:', err);
      setError(
        err instanceof Error
          ? `Failed to load data from server: ${err.message}`
          : 'Failed to load data from server'
      );
      return false;
    }
  }, []);

  const reload = useCallback(async () => {
    setIsLoading(true);
    await fetchRange();
    setIsLoading(false);
  }, [fetchRange]);

  useEffect(() => {
    if (isInitialized.current) return;
    let isMounted = true;

    const initializeTimeline = async () => {
      setIsLoading(true);

      const tryFetch = async () => {
        if (!isMounted) {
          return;
        }
        if (!webSocketManager.isConnected()) {
          retryTimeoutRef.current = window.setTimeout(tryFetch, 100);
          return;
        }
        await fetchRange();
        if (isMounted) {
          setIsLoading(false);
          isInitialized.current = true;
        }
      };

      tryFetch();
    };

    initializeTimeline();

    return () => {
      isMounted = false;
      if (retryTimeoutRef.current !== null) {
        window.clearTimeout(retryTimeoutRef.current);
        retryTimeoutRef.current = null;
      }
    };
  }, [fetchRange]);

  const selectFrame = useCallback((frame: number, immediate?: boolean) => {
    setIsNavigationImmediate(immediate ?? false);

    const clampedFrame = clampFrameToRange(frame, originalRange);

    setRange((prevRange) => {
      if (clampedFrame < prevRange.min || clampedFrame > prevRange.max) {
        return originalRange;
      }
      return prevRange;
    });

    setCurrentFrame((prev) => (clampedFrame === prev ? prev : clampedFrame));
  }, [originalRange]);

  const nextFrame = useCallback((step = 1, immediate?: boolean) => {
    setIsNavigationImmediate(immediate ?? false);
    setCurrentFrame((prev) => Math.min(range.max, prev + step));
  }, [range.max]);

  const prevFrame = useCallback((step = 1, immediate?: boolean) => {
    setIsNavigationImmediate(immediate ?? false);
    setCurrentFrame((prev) => Math.max(range.min, prev - step));
  }, [range.min]);

  const jumpToStart = useCallback((immediate?: boolean) => {
    setIsNavigationImmediate(immediate ?? false);
    setCurrentFrame(range.min);
  }, [range.min]);

  const jumpToEnd = useCallback((immediate?: boolean) => {
    setIsNavigationImmediate(immediate ?? false);
    setCurrentFrame(range.max);
  }, [range.max]);

  const zoomToRange = useCallback((start: number, end: number) => {
    const orderedStart = Math.min(start, end);
    const orderedEnd = Math.max(start, end);

    const clampedStart = Math.max(originalRange.min, orderedStart);
    const clampedEnd = Math.min(originalRange.max, orderedEnd);

    if (clampedEnd - clampedStart < 1) return;

    setRange({ min: clampedStart, max: clampedEnd });
    setSelection(null);
  }, [originalRange]);

  const resetZoom = useCallback(() => {
    setRange(originalRange);
    setSelection(null);
  }, [originalRange]);

  const isZoomed = range.min !== originalRange.min || range.max !== originalRange.max;

  const state = useMemo(() => ({
    currentFrame,
    range,
    originalRange,
    selection,
    isLoading,
    error,
    isZoomed,
    isNavigationImmediate,
  }), [currentFrame, range, originalRange, selection, isLoading, error, isZoomed, isNavigationImmediate]);

  const actions = useMemo(() => ({
    selectFrame,
    nextFrame,
    prevFrame,
    jumpToStart,
    jumpToEnd,
    zoomToRange,
    resetZoom,
    setSelection,
    reload,
  }), [selectFrame, nextFrame, prevFrame, jumpToStart, jumpToEnd, zoomToRange, resetZoom, reload]);

  return { state, actions };
}
