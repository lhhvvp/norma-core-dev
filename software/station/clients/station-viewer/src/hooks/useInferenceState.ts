import { useState, useEffect } from "react";
import webSocketManager from "../api/websocket";
import { Frame } from "../api/frame-parser";
import { WS_EVENTS } from "../api/websocket-events";

export function useInferenceState(): Frame | null {
  const [state, setState] = useState<Frame | null>(
    () => webSocketManager.getCurrentFrame()
  );

  useEffect(() => {
    const handler = () => setState(webSocketManager.getCurrentFrame());
    webSocketManager.addEventListener(WS_EVENTS.INFERENCE_STATE, handler);
    return () => webSocketManager.removeEventListener(WS_EVENTS.INFERENCE_STATE, handler);
  }, []);

  return state;
}
