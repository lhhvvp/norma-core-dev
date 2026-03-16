import { forwardRef } from 'react';
import { st3215 } from '../api/proto';
import BaseRobotRenderer, { BaseRobotRendererRef } from './BaseRobotRenderer';

interface ElRobotRendererProps {
  busSerialNumber: string | null | undefined;
  bus: st3215.InferenceState.IBusState;
  isLeader?: boolean;
  motorCount?: number;
}

const jointNames: (string | string[])[] = [
  'rev_motor_01',
  'rev_motor_02',
  'rev_motor_03',
  'rev_motor_04',
  'rev_motor_05',
  'rev_motor_06',
  'rev_motor_07',
  ['rev_motor_08', 'rev_motor_08_1', 'rev_motor_08_2'],
];

const ElRobotRenderer = forwardRef<BaseRobotRendererRef, ElRobotRendererProps>((props, ref) => {
  const { busSerialNumber, bus, isLeader, motorCount } = props;
  const urdfPath = 'elrobot/elrobot_follower.urdf'; // TODO: Add leader URDF
  const basePos: [number, number, number] = [0, 0, 0];
  const baseRpy: [number, number, number] = [-Math.PI/2, 0, -Math.PI/2];

  return (
    <BaseRobotRenderer
      ref={ref}
      busSerialNumber={busSerialNumber}
      bus={bus}
      isLeader={isLeader}
      urdfPath={urdfPath}
      jointNames={jointNames.slice(0, motorCount ?? jointNames.length)}
      basePos={basePos}
      baseRpy={baseRpy}
      robotType="elrobot"
    />
  );
});

export default ElRobotRenderer;
