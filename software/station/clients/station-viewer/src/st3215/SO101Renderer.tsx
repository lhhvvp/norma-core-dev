import { forwardRef } from 'react';
import { st3215 } from '../api/proto';
import BaseRobotRenderer, { BaseRobotRendererRef } from './BaseRobotRenderer';

interface SO101RendererProps {
  busSerialNumber: string | null | undefined;
  bus: st3215.InferenceState.IBusState;
  isLeader?: boolean;
}

const SO101Renderer = forwardRef<BaseRobotRendererRef, SO101RendererProps>((props, ref) => {
  const { busSerialNumber, bus, isLeader } = props;
  const urdfPath = isLeader ? 'so101/so101_robot_leader.urdf' : 'so101/so101_robot_follower.urdf';
  const basePos: [number, number, number] = [0.125, -0.03, -0.17];
  const baseRpy: [number, number, number] = [-Math.PI/2, 0, 0];

  return (
    <BaseRobotRenderer
      ref={ref}
      busSerialNumber={busSerialNumber}
      bus={bus}
      isLeader={isLeader}
      urdfPath={urdfPath}
      basePos={basePos}
      baseRpy={baseRpy}
      robotType="so101"
    />
  );
});

export default SO101Renderer;
