export const WS_EVENTS = {
  INFERENCE_STATE: 'inferenceState',
  STATS: 'stats',
} as const;

export type WSEventName = typeof WS_EVENTS[keyof typeof WS_EVENTS];
