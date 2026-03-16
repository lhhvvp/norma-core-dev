import Long from "long";
import { getGlobalTimeAdjustmentNs, isTimeSyncActive } from "./time-sync.js";

/**
 * Convert server monotonic timestamp to local client wall clock time
 * @param serverMonotonicNs Server monotonic timestamp in nanoseconds (number or Long)
 * @returns Local client time equivalent in nanoseconds as Long
 */
export function serverToLocal(serverMonotonicNs: number | Long): Long {
  // Convert input to Long if it's a number
  const serverMonotonicLong = typeof serverMonotonicNs === 'number'
    ? Long.fromNumber(serverMonotonicNs)
    : Long.isLong(serverMonotonicNs)
      ? serverMonotonicNs
      : Long.fromValue(serverMonotonicNs);

  if (!isTimeSyncActive()) {
    return serverMonotonicLong;
  }
  
  const adjustmentNs = getGlobalTimeAdjustmentNs();
  const serverNs = serverMonotonicLong.toNumber();
  
  // The adjustment is (server monotonic time - client time when server processed request)
  // So to get client time equivalent: server monotonic time - adjustment
  const clientEquivalentNs = serverNs - adjustmentNs;
  
  return Long.fromNumber(clientEquivalentNs);
}