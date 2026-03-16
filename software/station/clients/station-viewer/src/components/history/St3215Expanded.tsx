import { memo, useMemo } from 'react';
import { st3215 } from '@/api/proto.js';
import { getMotorCurrent, getMotorPosition, getMotorTemperature, getMotorVelocity, isTorqueEnabled } from '@/st3215/motor-parser';
import BusWebGLRenderer from '@/st3215/BusWebGLRenderer';

interface St3215ExpandedProps {
  data: st3215.InferenceState;
}

interface MotorSummaryTableProps {
  motors?: st3215.InferenceState.IMotorState[] | null;
}

const MotorSummaryTable = memo(function MotorSummaryTable({ motors }: MotorSummaryTableProps) {
  const sortedMotors = useMemo(() => {
    if (!motors || motors.length === 0) return [];
    return [...motors].sort((a: st3215.InferenceState.IMotorState, b: st3215.InferenceState.IMotorState) => {
      const aId = a.id ?? Number.POSITIVE_INFINITY;
      const bId = b.id ?? Number.POSITIVE_INFINITY;
      if (aId === bId) {
        return 0;
      }
      return aId < bId ? -1 : 1;
    });
  }, [motors]);

  if (sortedMotors.length === 0) {
    return <div className="text-xs text-gray-500">No motors reported.</div>;
  }



  return (
    <div className="overflow-x-auto">
      <table className="min-w-full text-xs text-gray-300">
        <thead>
          <tr className="text-gray-400 border-b-2 border-gray-700">
            <th className="text-center font-semibold py-1 pr-3 whitespace-nowrap w-auto">Motor</th>
            <th className="text-center font-semibold py-1 pr-3">Position</th>
            <th className="text-center font-semibold py-1 pr-3">Min</th>
            <th className="text-center font-semibold py-1 pr-3">Max</th>
            <th className="text-center font-semibold py-1 pr-3">Current</th>
            <th className="text-center font-semibold py-1 pr-3">Speed</th>
            <th className="text-center font-semibold py-1 pr-3">Temp</th>
            <th className="text-center font-semibold py-1">Torque</th>
          </tr>
        </thead>
        <tbody>
          {sortedMotors.map((motor, idx) => {
            const state = motor.state ?? null;
            const position = state ? getMotorPosition(state) : null;
            const current = state ? getMotorCurrent(state) : null;
            const velocity = state ? getMotorVelocity(state) : null;
            const temperature = state ? getMotorTemperature(state) : null;
            const driveEnabled = state ? isTorqueEnabled(state) : null;
            const rangeMin = motor.rangeMin;
            const rangeMax = motor.rangeMax;
            const key = motor.id ?? idx;

            return (
              <tr key={key} className={`border-t border-gray-800 ${idx % 2 === 1 ? 'bg-gray-900/30' : ''}`}>
                <td className="py-1 pr-3 text-center text-cyan-400 font-mono whitespace-nowrap">{motor.id ?? '--'}</td>
                <td className="py-1 pr-3 text-center text-purple-400">
                  {position === null ? '--' : position}
                </td>
                <td className="py-1 pr-3 text-center text-pink-400">
                  {rangeMin === null || rangeMin === undefined ? '--' : rangeMin}
                </td>
                <td className="py-1 pr-3 text-center text-pink-400">
                  {rangeMax === null || rangeMax === undefined ? '--' : rangeMax}
                </td>
                <td className="py-1 pr-3 text-center text-green-400">
                  {current === null ? '--' : `${current} mA`}
                </td>
                <td className="py-1 pr-3 text-center text-blue-400">
                  {velocity === null ? '--' : velocity}
                </td>
                <td className="py-1 pr-3 text-center text-orange-400">
                  {temperature === null ? '--' : `${temperature}C`}
                </td>
                <td className="py-1 text-center">
                  {driveEnabled === null ? (
                    <span className="text-gray-500">--</span>
                  ) : driveEnabled ? (
                    <span className="text-green-400">On</span>
                  ) : (
                    <span className="text-gray-500">Off</span>
                  )}
                </td>
              </tr>
            );
          })}

        </tbody>
      </table>
    </div>
  );
});

const St3215Expanded = memo(function St3215Expanded({ data }: St3215ExpandedProps) {
  const busCount = data.buses?.length ?? 0;
  const totalMotors = data.buses?.reduce((total, bus) => total + (bus.motors?.length || 0), 0) ?? 0;
  const canRenderWebGL = BusWebGLRenderer.canRender();

  return (
    <div className="space-y-3">
      <div>
        <div className="text-xs text-gray-400 mb-1">ST3215 Inference State:</div>
        <div className="bg-gray-900 p-2 rounded text-xs space-y-1">
          <div className="text-orange-400">Type: ST3215 Inference State</div>
          <div className="text-cyan-400">Buses: {busCount}</div>
          <div className="text-green-400">Total Motors: {totalMotors}</div>
        </div>
      </div>

      {busCount === 0 && (
        <div className="bg-gray-900 p-2 rounded text-xs text-gray-400">
          No bus data available.
        </div>
      )}

      <div className="grid grid-cols-1 2xl:grid-cols-2 gap-3">
        {data.buses?.map((bus, busIndex) => {
          const busLabel = bus.bus?.serialNumber ? `#${bus.bus.serialNumber}` : `Bus ${busIndex + 1}`;
          const motorCount = bus.motors?.length ?? 0;

          return (
            <div key={bus.bus?.serialNumber ?? busIndex} className="bg-gray-900/60 border border-gray-800 rounded p-2 space-y-2" data-bus-container={bus.bus?.serialNumber ?? busIndex}>
              <div className="flex items-center gap-3 text-xs">
                <span className="text-cyan-400 font-mono">{busLabel}</span>
                <span className="text-gray-400">Motors: {motorCount}</span>
              </div>
              <div className="flex flex-col lg:flex-row gap-3 max-w-4xl">
                <div className="bg-gray-950 rounded w-56 h-56 flex-shrink-0 overflow-hidden">
                  {canRenderWebGL ? (
                    <BusWebGLRenderer
                      busSerialNumber={bus.bus?.serialNumber}
                      bus={bus}
                      busIndex={busIndex}
                    />
                  ) : (
                    <div className="text-xs text-gray-500 flex items-center justify-center h-full">
                      WebGL preview unavailable
                    </div>
                  )}
                </div>
                <div className="flex-1 min-w-0">
                  <MotorSummaryTable motors={bus.motors} />
                </div>
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
});

export default St3215Expanded;
