import { sysinfo } from '@/api/proto.js';

interface SysinfoGridProps {
  data: sysinfo.IEnvelope;
}

export default function SysinfoGrid({ data }: SysinfoGridProps) {
  return (
    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-2 min-w-[200px]">
      <div className="bg-gray-900 rounded p-2 max-h-48 overflow-y-auto">
        <div className="text-xs text-gray-400 border-b border-gray-700 pb-1 mb-1">System</div>
        {data.data?.hostname && <div className="text-xs text-cyan-400">{data.data.hostname}</div>}
        {data.data?.os && <div className="text-xs text-green-400">{data.data.os.name}</div>}
        {data.data?.cpuArch && <div className="text-xs text-gray-300">{data.data.cpuArch}</div>}
        {data.data?.name && <div className="text-[10px] text-gray-500">{data.data.name}</div>}
      </div>

      <div className="bg-gray-900 rounded p-2 max-h-48 overflow-y-auto">
        <div className="text-xs text-gray-400 border-b border-gray-700 pb-1 mb-1">CPU ({data.data?.cpu?.length || 0})</div>
        {data.data?.cpu?.map((cpu, idx) => (
          <div key={idx} className="flex justify-between text-xs">
            <span className="text-orange-400">C{idx}</span>
            <span className="text-cyan-400">{cpu.usage?.toFixed(1)}%</span>
            <span className="text-gray-400">{cpu.frequency && Number(cpu.frequency) > 0 ? `${(Number(cpu.frequency) / 1000).toFixed(2)}GHz` : ''}</span>
          </div>
        ))}
      </div>

      <div className="bg-gray-900 rounded p-2 max-h-48 overflow-y-auto">
        <div className="text-xs text-gray-400 border-b border-gray-700 pb-1 mb-1">Memory</div>
        {data.data?.memory && (
          <>
            <div className="flex justify-between text-xs">
              <span className="text-green-400">RAM</span>
              <span className="text-cyan-400">{(Number(data.data.memory.usedBytes || 0) / (1024 * 1024 * 1024)).toFixed(2)}/{(Number(data.data.memory.totalBytes || 0) / (1024 * 1024 * 1024)).toFixed(2)}GB</span>
            </div>
            <div className="text-[10px] text-gray-400 text-right">{((Number(data.data.memory.usedBytes || 0) / Number(data.data.memory.totalBytes || 1)) * 100).toFixed(1)}%</div>
            {Number(data.data.memory.totalSwapBytes || 0) > 0 && (
              <div className="flex justify-between text-xs mt-1">
                <span className="text-blue-400">Swap</span>
                <span className="text-cyan-400">{(Number(data.data.memory.usedSwapBytes || 0) / (1024 * 1024 * 1024)).toFixed(2)}/{(Number(data.data.memory.totalSwapBytes || 0) / (1024 * 1024 * 1024)).toFixed(2)}GB</span>
              </div>
            )}
          </>
        )}
      </div>

      <div className="bg-gray-900 rounded p-2 max-h-48 overflow-y-auto">
        <div className="text-xs text-gray-400 border-b border-gray-700 pb-1 mb-1">Disks ({data.data?.disks?.length || 0})</div>
        {data.data?.disks?.map((disk, idx) => (
          <div key={idx} className="text-xs mb-1">
            <div className="flex justify-between">
              <span className="text-purple-400">{disk.mountPoint}</span>
              <span className="text-gray-500 text-[10px]">{disk.fs}</span>
            </div>
            <div className="text-cyan-400">{((Number(disk.totalSpaceBytes || 0) - Number(disk.availableSpaceBytes || 0)) / (1024 * 1024 * 1024)).toFixed(2)}/{(Number(disk.totalSpaceBytes || 0) / (1024 * 1024 * 1024)).toFixed(2)}GB</div>
          </div>
        ))}
      </div>

      <div className="bg-gray-900 rounded p-2 max-h-48 overflow-y-auto">
        <div className="text-xs text-gray-400 border-b border-gray-700 pb-1 mb-1">Network ({data.data?.networks?.length || 0})</div>
        {data.data?.networks?.map((net, idx) => (
          <div key={idx} className="text-xs mb-1">
            <div className="flex justify-between">
              <span className="text-blue-400">{net.iface}</span>
              <span className="text-cyan-400 text-[10px]">↓{(Number(net.bytesReceived || 0) / (1024 * 1024)).toFixed(1)} ↑{(Number(net.bytesTransmitted || 0) / (1024 * 1024)).toFixed(1)}MB</span>
            </div>
            {net.ips?.[0] && <div className="text-green-400 text-[10px]">{net.ips[0].addr}</div>}
          </div>
        ))}
      </div>

      <div className="bg-gray-900 rounded p-2 max-h-48 overflow-y-auto">
        <div className="text-xs text-gray-400 border-b border-gray-700 pb-1 mb-1">Temp ({data.data?.temperatures?.length || 0})</div>
        {data.data?.temperatures?.map((temp, idx) => (
          <div key={idx} className="flex justify-between text-xs">
            <span className="text-gray-300">{temp.name || temp.id}</span>
            <span className={temp.value && temp.critical && temp.value > temp.critical ? "text-red-400" : "text-orange-400"}>{temp.value?.toFixed(1)}°C</span>
          </div>
        ))}
      </div>
    </div>
  );
}
