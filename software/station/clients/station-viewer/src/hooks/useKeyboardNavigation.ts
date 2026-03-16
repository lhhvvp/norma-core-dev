import { useEffect } from 'react';
import { TimelineActions, TimelineState } from './useTimelineState';

export interface TimelineControlsRef {
  focusGotoInput: () => void;
}

export interface KeyboardNavigationOptions {
  enabled?: boolean;
  frameStep?: number;
  largeFrameStep?: number;
  gotoInputRef?: React.RefObject<TimelineControlsRef | null>;
}

const DEFAULT_OPTIONS: Required<Omit<KeyboardNavigationOptions, 'gotoInputRef'>> & {
  gotoInputRef: undefined;
} = {
  enabled: true,
  frameStep: 1,
  largeFrameStep: 10,
  gotoInputRef: undefined,
};

export function useKeyboardNavigation(
  actions: TimelineActions,
  state: TimelineState,
  options: KeyboardNavigationOptions = {}
): void {
  const { enabled, frameStep, largeFrameStep, gotoInputRef } = { ...DEFAULT_OPTIONS, ...options };

  useEffect(() => {
    if (!enabled) return;

    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement) {
        return;
      }

      switch (e.key) {
        case 'ArrowLeft':
          e.preventDefault();
          actions.prevFrame(e.shiftKey ? largeFrameStep : frameStep, true);
          break;
        case 'ArrowRight':
          e.preventDefault();
          actions.nextFrame(e.shiftKey ? largeFrameStep : frameStep, true);
          break;
        case 'Home':
          e.preventDefault();
          actions.jumpToStart(true);
          break;
        case 'End':
          e.preventDefault();
          actions.jumpToEnd(true);
          break;
        case 'g':
        case 'G':
          e.preventDefault();
          gotoInputRef?.current?.focusGotoInput();
          break;
        case 'Escape':
          e.preventDefault();
          if (state.isZoomed) {
            actions.resetZoom();
          }
          break;
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [actions, state.isZoomed, enabled, frameStep, largeFrameStep, gotoInputRef]);
}
