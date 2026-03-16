import { useState, useEffect, useRef, useCallback } from 'react';
import NoSleep from 'nosleep.js';

export interface UseWakeLockReturn {
  isActive: boolean;
  error: string | null;
}

export function useWakeLock(): UseWakeLockReturn {
  const [isActive, setIsActive] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const wakeLockRef = useRef<WakeLockSentinel | null>(null);
  const noSleepRef = useRef<NoSleep | null>(null);
  const isRequestingRef = useRef(false);

  const handleWakeLockRelease = useCallback(() => {
    wakeLockRef.current = null;
    setIsActive(false);
  }, []);

  const requestWakeLock = useCallback(async () => {
    if (isRequestingRef.current) return;
    isRequestingRef.current = true;

    try {
      if ('wakeLock' in navigator) {
        try {
          const sentinel = await navigator.wakeLock.request('screen');
          if (wakeLockRef.current) {
            wakeLockRef.current.removeEventListener('release', handleWakeLockRelease);
          }
          wakeLockRef.current = sentinel;
          setIsActive(true);
          setError(null);
          wakeLockRef.current.addEventListener('release', handleWakeLockRelease);
        } catch (err) {
          const errorMessage = err instanceof Error ? err.message : 'Unknown error';
          if (errorMessage.includes('denied') || errorMessage.includes('permission')) {
            setError('Permission denied. Screen may sleep during operation.');
          }
          throw err;
        }
      } else {
        throw new Error('Wake Lock API not supported');
      }
    } catch {
      if (!noSleepRef.current) {
        noSleepRef.current = new NoSleep();
      }
      try {
        await noSleepRef.current.enable();
        setIsActive(true);
        setError(null);
      } catch (err) {
        const errorMessage = err instanceof Error ? err.message : 'Unknown error';
        setError(`Failed to prevent screen sleep: ${errorMessage}`);
        setIsActive(false);
      }
    } finally {
      isRequestingRef.current = false;
    }
  }, [handleWakeLockRelease]);

  const releaseWakeLock = useCallback(() => {
    const wakeLock = wakeLockRef.current;
    if (wakeLock) {
      wakeLock.removeEventListener('release', handleWakeLockRelease);
      wakeLock.release().catch(console.error);
      wakeLockRef.current = null;
    }
    if (noSleepRef.current) {
      noSleepRef.current.disable();
      noSleepRef.current = null;
    }
    setIsActive(false);
  }, [handleWakeLockRelease]);

  useEffect(() => {
    const handleVisibilityChange = () => {
      if (document.visibilityState === 'visible') {
        if (!wakeLockRef.current && !isRequestingRef.current) {
          requestWakeLock();
        }
      }
    };

    requestWakeLock();

    document.addEventListener('visibilitychange', handleVisibilityChange);

    return () => {
      document.removeEventListener('visibilitychange', handleVisibilityChange);
      releaseWakeLock();
    };
  }, [requestWakeLock, releaseWakeLock]);

  return { isActive, error };
}
