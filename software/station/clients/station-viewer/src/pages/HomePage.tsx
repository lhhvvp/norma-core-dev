import { useState, useEffect } from 'react';
import { useInferenceState, useConnectionStatsWithUptime, useLatestEntryId, useWakeLock } from "@/hooks";
import BusViewer from "@/st3215/BusViewer";
import AsciiRobot from "@/components/AsciiRobot";
import { copyToClipboard } from "@/api/clipboard-utils";

function formatBytes(bytes: number): string {
  if (bytes === 0) return '0 B';
  const k = 1024;
  const sizes = ['B', 'KB', 'MB', 'GB'];
  const i = Math.floor(Math.log(bytes) / Math.log(k));
  return `${(bytes / Math.pow(k, i)).toFixed(2)} ${sizes[i]}`;
}

function formatUptime(connectedAt: number | null): string {
  if (!connectedAt) return 'N/A';
  const seconds = Math.floor((Date.now() - connectedAt) / 1000);
  const hours = Math.floor(seconds / 3600);
  const minutes = Math.floor((seconds % 3600) / 60);
  const secs = seconds % 60;
  return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
}

function HomePage() {
  useWakeLock();
  const inferenceState = useInferenceState();
  const latestEntryId = useLatestEntryId();
  const connectionStats = useConnectionStatsWithUptime();
  const [copied, setCopied] = useState(false);
  const hasRobotData = Boolean(inferenceState?.st3215?.data?.buses?.length);

  useEffect(() => {
    if (copied) {
      const timer = setTimeout(() => setCopied(false), 1000);
      return () => clearTimeout(timer);
    }
  }, [copied]);

  const handleCopyEntryId = () => {
    if (latestEntryId !== null) {
      copyToClipboard(latestEntryId.toString())
        .then(() => setCopied(true))
        .catch(err => console.error('Failed to copy entry ID:', err));
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'connected': return 'text-green-400';
      case 'connecting': return 'text-yellow-400';
      case 'disconnected': return 'text-red-400';
      default: return 'text-gray-400';
    }
  };

  const getFPSColor = (fps: number) => {
    if (fps >= 15) return 'text-green-400';
    if (fps >= 10) return 'text-yellow-400';
    if (fps >= 5) return 'text-orange-400';
    return 'text-red-400';
  };

  return (
    <div className="flex-1 flex flex-col">
      <div className="relative z-20 bg-gray-900 border-b-2 border-gray-700">
        <div className="px-4 py-2 flex flex-wrap gap-x-4 gap-y-2 items-center">
          {connectionStats && (
            <>
              <div className="flex items-center gap-2">
                <div className="flex items-center gap-2 px-2 py-1 bg-gray-800 rounded border border-gray-700">
                  <span className="text-gray-400 text-xs uppercase tracking-wide">Status</span>
                  <span className={`font-semibold uppercase text-xs ${getStatusColor(connectionStats.status)}`}>
                    {connectionStats.status}
                  </span>
                </div>
                {connectionStats.status === 'connected' && inferenceState?.st3215?.data?.buses && inferenceState.st3215.data.buses.length > 0 && (
                  <div className="flex items-center gap-2 px-2 py-1 bg-gray-800 rounded border border-gray-700">
                    <span className="text-gray-400 text-xs uppercase tracking-wide">FPS</span>
                    <span className={`font-bold text-xs font-mono ${connectionStats.isFpsReady ? getFPSColor(connectionStats.fps) : 'text-gray-400'}`}>
                      {connectionStats.isFpsReady ? `${connectionStats.fps.toFixed(1)} Hz` : '--'}
                    </span>
                  </div>
                )}
                <div className="group relative flex items-center gap-2 px-2 py-1 bg-gray-800 rounded border border-gray-700 cursor-pointer" onClick={handleCopyEntryId}>
                  <span className="text-gray-400 text-xs uppercase tracking-wide">Entry ID</span>
                  <span className={`font-bold text-xs font-mono ${copied ? 'text-green-400' : 'text-yellow-400'}`}>
                    {latestEntryId?.toLocaleString() ?? 'N/A'}
                  </span>
                  <div className="absolute bottom-full mb-1 left-1/2 -translate-x-1/2 px-2 py-1 bg-black text-white text-xs rounded whitespace-nowrap z-50 invisible opacity-0 group-hover:visible group-hover:opacity-100 transition-opacity duration-200">
                    Click to copy
                  </div>
                </div>
              </div>
              <div className="flex flex-wrap items-center gap-x-4 gap-y-1 text-xs font-mono">
                <div className="flex items-center gap-1.5">
                  <span className="text-gray-500">Endpoint:</span>
                  <span className="text-cyan-400">{connectionStats.endpoint}</span>
                </div>
                <div className="flex items-center gap-1.5">
                  <span className="text-gray-500">Packets:</span>
                  <span className="text-blue-400 font-semibold">{connectionStats.packetsReceived.toLocaleString()}</span>
                </div>
                <div className="flex items-center gap-1.5">
                  <span className="text-gray-500">Data:</span>
                  <span className="text-purple-400 font-semibold">{formatBytes(connectionStats.bytesReceived)}</span>
                </div>
                <div className="flex items-center gap-1.5">
                  <span className="text-gray-500">Uptime:</span>
                  <span className="text-green-400 font-semibold">{formatUptime(connectionStats.connectedAt)}</span>
                </div>
              </div>
            </>
          )}
        </div>
      </div>
      <div className="flex-1 overflow-auto p-4">
        {hasRobotData ? (
          <BusViewer
            inferenceState={inferenceState!.st3215!.data}
            videoSources={inferenceState?.videoQueues}
            mirroringState={inferenceState?.mirroring?.data.state || undefined}
          />
        ) : (
          <div className="flex h-full min-h-[240px] items-center justify-center rounded-lg border border-dashed border-gray-700 bg-gray-900/40 px-6">
            <AsciiRobot />
          </div>
        )}
      </div>
    </div>
  );
}

export default HomePage;
