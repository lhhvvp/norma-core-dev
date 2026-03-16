import { useState, useEffect, useCallback, useRef } from "react";
import webSocketManager, { ConnectionStats } from "../api/websocket";
import { WS_EVENTS } from "../api/websocket-events";

export function useConnectionStats(): ConnectionStats | null {
  const [stats, setStats] = useState<ConnectionStats | null>(
    () => webSocketManager.getConnectionStats()
  );

  useEffect(() => {
    const handler = () => setStats(webSocketManager.getConnectionStats());
    webSocketManager.addEventListener(WS_EVENTS.STATS, handler);
    return () => webSocketManager.removeEventListener(WS_EVENTS.STATS, handler);
  }, []);

  return stats;
}

export function useConnectionStatsWithUptime(): ConnectionStats | null {
  const [stats, setStats] = useState<ConnectionStats | null>(
    () => webSocketManager.getConnectionStats()
  );
  const connectedAtRef = useRef<number | null>(null);

  const updateStats = useCallback(() => {
    setStats(webSocketManager.getConnectionStats());
  }, []);

  useEffect(() => {
    webSocketManager.addEventListener(WS_EVENTS.STATS, updateStats);
    return () => webSocketManager.removeEventListener(WS_EVENTS.STATS, updateStats);
  }, [updateStats]);

  useEffect(() => {
    connectedAtRef.current = stats?.connectedAt ?? null;
  }, [stats?.connectedAt]);

  useEffect(() => {
    const interval = setInterval(() => {
      if (connectedAtRef.current) {
        setStats(webSocketManager.getConnectionStats());
      }
    }, 1000);

    return () => clearInterval(interval);
  }, []);

  return stats;
}
