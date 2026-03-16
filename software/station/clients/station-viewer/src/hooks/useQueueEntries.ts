import { useState, useEffect, useCallback } from 'react';
import webSocketManager from '../api/websocket';
import { inference } from '../api/proto.js';
import { HistoryElementData } from '../components/history/HistoryElement';
import { mapQueueId, getQueueTypeWithId } from '../api/queue-utils';

export interface UseQueueEntriesReturn {
  entries: HistoryElementData[];
  isLoading: boolean;
  error: string | null;
}

export function useQueueEntries(inferenceRx: inference.InferenceRx | null): UseQueueEntriesReturn {
  const [entries, setEntries] = useState<HistoryElementData[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const loadQueueEntries = useCallback(async (rx: inference.InferenceRx) => {
    if (!rx.entries || rx.entries.length === 0) {
      setEntries([]);
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const queueEntriesToRead: Array<{ queueId: string; entryId: Uint8Array; entryType?: number }> = [];

      for (const entry of rx.entries) {
        if (entry.queue && entry.ptr && entry.ptr.length > 0) {
          queueEntriesToRead.push({
            queueId: mapQueueId(entry.queue),
            entryId: entry.ptr,
            entryType: entry.type ?? undefined,
          });
        }
      }

      if (queueEntriesToRead.length === 0) {
        setEntries([]);
        setIsLoading(false);
        return;
      }

      const promises = queueEntriesToRead.map(async (entryToRead) => {
        try {
          const result = await webSocketManager.normFs.readSingleEntry(
            entryToRead.queueId,
            entryToRead.entryId
          );
          return {
            queueId: entryToRead.queueId,
            entryId: entryToRead.entryId,
            data: result.data,
            error: undefined,
            entryType: entryToRead.entryType ?? 0,
          };
        } catch (err) {
          console.error(`Failed to read entry from ${entryToRead.queueId}:`, err);
          return {
            queueId: entryToRead.queueId,
            entryId: entryToRead.entryId,
            data: null,
            error: err instanceof Error ? err.message : 'Failed to read entry',
            entryType: entryToRead.entryType ?? 0,
          };
        }
      });

      const results = await Promise.all(promises);

      const historyElements: HistoryElementData[] = results.map((result) => ({
        queueId: result.queueId,
        entryId: result.entryId,
        data: result.data,
        error: result.error,
        queueType: result.entryType,
        type: getQueueTypeWithId(result.entryType, result.queueId),
      }));

      setEntries(historyElements);
    } catch (err) {
      console.error('Error loading queue entries:', err);
      setError(err instanceof Error ? err.message : 'Unknown error loading queue entries');
    } finally {
      setIsLoading(false);
    }
  }, []);

  useEffect(() => {
    if (inferenceRx && inferenceRx.entries && inferenceRx.entries.length > 0) {
      loadQueueEntries(inferenceRx);
    } else {
      setEntries([]);
      setError(null);
    }
  }, [inferenceRx, loadQueueEntries]);

  return {
    entries,
    isLoading,
    error,
  };
}
