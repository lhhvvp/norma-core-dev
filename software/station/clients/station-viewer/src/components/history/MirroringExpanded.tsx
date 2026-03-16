import { motors_mirroring } from '@/api/proto.js';

interface MirroringExpandedProps {
  data: motors_mirroring.RxEnvelope;
}

export default function MirroringExpanded({ data }: MirroringExpandedProps) {
  return (
    <div>
      <div className="text-xs text-gray-400 mb-1">Motors Mirroring RxEnvelope:</div>
      <div className="bg-gray-900 p-2 rounded text-xs space-y-1">
        <div className="text-purple-400">Type: Motors Mirroring</div>
        {data.state?.mirroring && data.state.mirroring.length > 0 && (
          <div className="text-cyan-400">
            Mirroring Configurations: {data.state.mirroring.length}
          </div>
        )}
      </div>
    </div>
  );
}
