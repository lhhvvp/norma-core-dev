import { normfs } from "./proto.js";
import Long from "long";

export interface TimeSyncState {
  /** Time adjustment in nanoseconds (server time - client time) */
  timeAdjustmentNs: number;
  serverStartId: Long | null;
  lastSyncRequestTime: number;
  pingTimeMs: number;
  isActive: boolean;
  syncCount: number;
}

export interface AdjustedTimestamp {
  originalMonotonicNs: Long;
  adjustedMonotonicNs: Long;
  adjustmentAppliedAt: number;
}

class TimeSyncManager {
  private state: TimeSyncState = {
    timeAdjustmentNs: 0,
    serverStartId: null,
    lastSyncRequestTime: 0,
    pingTimeMs: 0,
    isActive: false,
    syncCount: 0,
  };

  private syncInterval: number | null = null;
  private readonly SYNC_INTERVAL_MS = 1000; // 1 second
  private sendPingRequest: ((request: normfs.IPingRequest) => void) | null = null;
  private nextSequence: number = 0;

  /**
   * Initialize time sync with a callback to send ping requests
   */
  public initialize(sendRequest: (request: normfs.IPingRequest) => void): void {
    this.sendPingRequest = sendRequest;
    this.startPeriodicSync();
  }

  /**
   * Stop time sync
   */
  public stop(): void {
    this.stopPeriodicSync();
    this.state.isActive = false;
  }

  /**
   * Start periodic time sync requests
   */
  private startPeriodicSync(): void {
    if (this.syncInterval !== null) {
      return; // Already running
    }

    this.state.isActive = true;
    
    // Send initial sync request immediately
    this.sendSyncRequest();

    // Set up periodic sync
    this.syncInterval = window.setInterval(() => {
      this.sendSyncRequest();
    }, this.SYNC_INTERVAL_MS);
  }

  /**
   * Stop periodic time sync requests
   */
  private stopPeriodicSync(): void {
    if (this.syncInterval !== null) {
      window.clearInterval(this.syncInterval);
      this.syncInterval = null;
    }
  }

  /**
   * Send a ping request with current client timestamp
   */
  private sendSyncRequest(): void {
    if (!this.sendPingRequest) {
      console.warn("TimeSyncManager: No send request callback configured");
      return;
    }

    const clientTimestamp = Date.now();
    this.state.lastSyncRequestTime = clientTimestamp;

    // Convert to nanoseconds and create Long
    const clientTimestampNs = Long.fromNumber(clientTimestamp * 1_000_000);

    const request: normfs.IPingRequest = {
      sequence: Long.fromNumber(this.nextSequence++),
      clientTimestampNs: clientTimestampNs,
    };

    this.sendPingRequest(request);
  }

  /**
   * Convert a value that can be either number or Long to a number
   */
  private toLongValue(value: number | Long | null | undefined): Long | null {
    if (value === null || value === undefined) {
      return null;
    }
    if (typeof value === 'number') {
      return Long.fromNumber(value);
    }
    if (Long.isLong(value)) {
      return value;
    }
    // Try to convert if it's a Long-like object
    return Long.fromValue(value);
  }

  /**
   * Process ping response from server for time sync
   */
  public processPingResponse(response: normfs.IPingResponse): void {
    const now = Date.now();

    if (!response.request || !response.request.clientTimestampNs || !response.localStampNs || !response.monotonicStampNs) {
      console.warn("TimeSyncManager: Invalid ping response", response);
      return;
    }

    // Convert response fields to Long objects
    const clientTimestampLong = this.toLongValue(response.request.clientTimestampNs);
    const localStampLong = this.toLongValue(response.localStampNs);
    const monotonicStampLong = this.toLongValue(response.monotonicStampNs);

    if (!clientTimestampLong || !localStampLong || !monotonicStampLong) {
      console.warn("TimeSyncManager: Failed to convert ping response fields to Long", response);
      return;
    }

    // Calculate ping time (round-trip time)
    const requestTimestampMs = clientTimestampLong.toNumber() / 1_000_000;
    this.state.pingTimeMs = now - requestTimestampMs;


    // Calculate time adjustment
    // Server monotonic time - estimated client time when server processed request
    const serverMonotonicNs = monotonicStampLong.toNumber();
    const estimatedClientTimeWhenServerProcessed = requestTimestampMs + (this.state.pingTimeMs / 2);
    const estimatedClientTimeNs = estimatedClientTimeWhenServerProcessed * 1_000_000;

    this.state.timeAdjustmentNs = serverMonotonicNs - estimatedClientTimeNs;
    this.state.syncCount++;
  }

  /**
   * Adjust a server monotonic timestamp using current time sync
   */
  public adjustTimestamp(serverMonotonicNs: number | Long): AdjustedTimestamp {
    const serverMonotonicLong = this.toLongValue(serverMonotonicNs);
    if (!serverMonotonicLong) {
      throw new Error("Invalid server monotonic timestamp");
    }

    const originalNs = serverMonotonicLong.toNumber();
    const adjustedNs = originalNs + this.state.timeAdjustmentNs;
    
    return {
      originalMonotonicNs: serverMonotonicLong,
      adjustedMonotonicNs: Long.fromNumber(adjustedNs),
      adjustmentAppliedAt: Date.now(),
    };
  }

  /**
   * Get current time sync state (read-only)
   */
  public getState(): Readonly<TimeSyncState> {
    return { ...this.state };
  }

  /**
   * Get current time adjustment in nanoseconds
   */
  public getTimeAdjustmentNs(): number {
    return this.state.timeAdjustmentNs;
  }

  /**
   * Get current ping time in milliseconds
   */
  public getPingTimeMs(): number {
    return this.state.pingTimeMs;
  }

  /**
   * Check if time sync is active and has received at least one response
   */
  public isTimeSyncActive(): boolean {
    return this.state.isActive && this.state.syncCount > 0;
  }
}

// Global time sync manager instance
export const timeSyncManager = new TimeSyncManager();

// Export for external access to time adjustment
export function getGlobalTimeAdjustmentNs(): number {
  return timeSyncManager.getTimeAdjustmentNs();
}

export function adjustServerTimestamp(serverMonotonicNs: number | Long): AdjustedTimestamp {
  return timeSyncManager.adjustTimestamp(serverMonotonicNs);
}

export function isTimeSyncActive(): boolean {
  return timeSyncManager.isTimeSyncActive();
}