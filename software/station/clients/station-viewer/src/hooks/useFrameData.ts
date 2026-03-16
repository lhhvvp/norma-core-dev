import { useState, useRef, useCallback, useEffect } from 'react';
import Long from 'long';
import webSocketManager from '../api/websocket';
import { Frame } from '../api/frame-parser';

const DEBOUNCE_DELAY_MS = 200;

export interface FrameDataState {
  currentFrame: number;
  parsedFrame: Frame | null;
  isLoading: boolean;
  error: string | null;
}

export interface UseFrameDataReturn extends FrameDataState {
  selectFrame: (frame: number, immediate?: boolean) => void;
}

export function useFrameData(): UseFrameDataReturn {
  const [currentFrame, setCurrentFrame] = useState<number>(0);
  const [parsedFrame, setParsedFrame] = useState<Frame | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const debounceTimeout = useRef<number | null>(null);
  const parsedFrameRef = useRef<Frame | null>(null);

  // Keep ref in sync with state
  useEffect(() => {
    parsedFrameRef.current = parsedFrame;
  }, [parsedFrame]);

  const readEntryData = useCallback(async (frameNumber: number, previousFrame: Frame | null) => {
    setIsLoading(true);
    setError(null);

    try {
      const entryIdBytes = new Uint8Array(Long.fromNumber(frameNumber).toBytesLE());

      // Parse full frame using getFrame, pass previous frame for optimization
      try {
        const frame = await webSocketManager.getFrame(entryIdBytes, previousFrame || undefined);
        setParsedFrame(frame);
      } catch (frameError) {
        console.error('Failed to parse frame:', frameError);
        setError(`Failed to parse frame: ${frameError instanceof Error ? frameError.message : 'Unknown error'}`);
      }
    } catch (err) {
      console.error('Error reading entry:', err);
      setError(err instanceof Error ? err.message : 'Unknown error reading entry');
    } finally {
      setIsLoading(false);
    }
  }, []);

  const selectFrame = useCallback((frameNumber: number, immediate?: boolean) => {
    setCurrentFrame((prev) => {
      if (frameNumber === prev) return prev;

      if (debounceTimeout.current) {
        clearTimeout(debounceTimeout.current);
      }

      if (immediate) {
        readEntryData(frameNumber, parsedFrameRef.current);
      } else {
        debounceTimeout.current = window.setTimeout(() => {
          readEntryData(frameNumber, parsedFrameRef.current);
        }, DEBOUNCE_DELAY_MS);
      }

      return frameNumber;
    });
  }, [readEntryData]);

  useEffect(() => {
    return () => {
      if (debounceTimeout.current) {
        clearTimeout(debounceTimeout.current);
      }
    };
  }, []);

  return {
    currentFrame,
    parsedFrame,
    isLoading,
    error,
    selectFrame,
  };
}
