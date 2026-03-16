import { useState, useEffect } from "react";
import webSocketManager from "../api/websocket";
import { WS_EVENTS } from "../api/websocket-events";

export function useLatestEntryId(): number | null {
  const [entryId, setEntryId] = useState<number | null>(
    () => webSocketManager.getLatestEntryId()
  );

  useEffect(() => {
    const handler = () => setEntryId(webSocketManager.getLatestEntryId());
    webSocketManager.addEventListener(WS_EVENTS.INFERENCE_STATE, handler);
    return () => webSocketManager.removeEventListener(WS_EVENTS.INFERENCE_STATE, handler);
  }, []);

  return entryId;
}
