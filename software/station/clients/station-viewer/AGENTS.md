# AGENTS.md

Guidelines for AI coding agents operating in this React/TypeScript project.

## Commands

```bash
yarn dev              # Start Vite dev server (http://localhost:5173)
yarn build            # Full build: hashes, proto, type-check, and Vite build
yarn build:proto      # Regenerate protobuf bindings from ../../../../../protobufs
yarn lint             # Run ESLint with flat config
yarn type-check       # Run TypeScript compiler without emitting
yarn preview          # Preview production build locally
```

**Testing:** Not configured. No test runner exists in this project.

## Tech Stack

- **React 19** with function components only
- **TypeScript 5.9** with strict mode enabled
- **Vite 7** for bundling (supports top-level await, URDF/STL assets)
- **Tailwind CSS v4** with @tailwindcss/vite plugin
- **Three.js** for 3D rendering (URDF robot visualization)
- **Protobuf.js** for binary protocol communication
- **React Router v7** for routing with lazy-loaded pages

## Project Structure

```
src/
  api/          # WebSocket, protobuf, time sync, queue utilities
  components/   # Shared UI components
  hooks/        # Custom React hooks (re-exported from index.ts)
  pages/        # Route components (suffixed with Page)
  st3215/       # Motor driver components and utilities
  usbvideo/     # Camera/video stream components
public/
  so101/        # Robot URDF models and STL assets
```

## Code Style

### Imports
Use `@/*` path aliases. Order: external deps → `@/api/*` → `@/components/*` → `@/hooks` → types

```typescript
import { forwardRef, memo, useImperativeHandle, useRef } from 'react';
import Long from 'long';
import webSocketManager from '@/api/websocket';
import { serverToLocal } from '@/api/timestamp-utils';
import Timeline from '@/components/Timeline';
import { useFrameData, useTimelineState } from '@/hooks';
```

### Formatting & Linting
- 2-space indentation, semicolons required
- `src/api/proto.*` files are auto-generated and excluded from linting
- ESLint flat config enforces rules

### Naming Conventions

| Entity | Convention | Example |
|--------|------------|---------|
| Components | PascalCase | `TimelineControls`, `BusViewer` |
| Page components | PascalCase + Page suffix | `HomePage`, `HistoryPage` |
| Hooks | camelCase with `use` prefix | `useTimelineState`, `useFrameData` |
| Utilities | kebab-case filenames | `queue-utils.ts`, `time-sync.ts` |
| Variables/functions | camelCase | `currentFrame`, `selectFrame` |
| Constants | UPPER_SNAKE_CASE | `WS_EVENTS`, `DEFAULT_TIMEOUT` |
| Interfaces | PascalCase | `TimelineState`, `ConnectionStats` |
| Props interfaces | PascalCase + Props suffix | `TimelineProps`, `BusViewerProps` |
| Error singletons | Err prefix | `ErrNotConnected`, `ErrBufferFull` |
| Protobuf interfaces | I prefix (from codegen) | `web.IClientPacket`, `st3215.IInferenceState` |

## Component Patterns

### Function Components
```typescript
interface TimelineControlsProps {
  state: TimelineState;
  actions: TimelineActions;
  frameStep?: number;
}

const TimelineControlsComponent = forwardRef<TimelineControlsRef, TimelineControlsProps>(
  function TimelineControls({ state, actions, frameStep = 1 }: TimelineControlsProps, ref) {
    // ...
  }
);

const TimelineControls = memo(TimelineControlsComponent);
TimelineControls.displayName = 'TimelineControls';
export default TimelineControls;
```

- Use function components only
- All components use default exports
- Define props interfaces directly above the component
- Wrap components that receive props with `memo()`
- Use `forwardRef` when exposing imperative handles
- Route components are lazy-loaded: `const HomePage = lazy(() => import('./pages/HomePage'));`

## Hook Patterns

### State/Actions Pattern
Complex hooks return separate state and actions objects:

```typescript
export interface UseTimelineStateReturn {
  state: TimelineState;
  actions: TimelineActions;
}

export function useTimelineState(): UseTimelineStateReturn {
  const [currentFrame, setCurrentFrame] = useState(0);
  // ...
  
  const state = useMemo(() => ({
    currentFrame,
    range,
    isLoading,
    error,
  }), [currentFrame, range, isLoading, error]);

  const actions = useMemo(() => ({
    selectFrame,
    nextFrame,
    prevFrame,
  }), [selectFrame, nextFrame, prevFrame]);

  return { state, actions };
}
```

### useEffect Cleanup
Always clean up event listeners, timers, and subscriptions:

```typescript
useEffect(() => {
  const handler = () => setStats(webSocketManager.getConnectionStats());
  webSocketManager.addEventListener(WS_EVENTS.STATS, handler);
  return () => webSocketManager.removeEventListener(WS_EVENTS.STATS, handler);
}, []);
```

### Hook Exports
All hooks are re-exported from `src/hooks/index.ts`:
```typescript
export { useInferenceState } from "./useInferenceState";
export { useConnectionStats } from "./useConnectionStats";
```

## Error Handling

### Module-Level Error Singletons
```typescript
export const ErrNotConnected = new Error("client not connected or setup not complete");
export const ErrBufferFull = new Error("client request buffer is full");
export const ErrRequestTimeout = new Error("request timed out waiting for server response");
```

### Async Error Handling
```typescript
try {
  const result = await fetchData();
  setError(null);
  return result;
} catch (err) {
  console.error('Failed to fetch data:', err);
  setError(err instanceof Error ? err.message : 'Unknown error');
  return null;
}
```

## Protobuf Patterns

- Use `IInterface` (with I prefix) for plain objects passed as parameters
- Use `Class` for static methods (create, encode, decode)

```typescript
public send(packet: web.IClientPacket) {
  const clientPacket = web.ClientPacket.create(packet);
  const buffer = web.ClientPacket.encode(clientPacket).finish();
  this.ws.send(buffer);
}
```

Run `yarn build:proto` after modifying .proto files.

## State Management

State is managed through custom hooks, not global state libraries. WebSocket events drive state updates via EventTarget. Global managers are exported as default singletons:

```typescript
const webSocketManager = new WebSocketManager(`ws://${host}/api`);
export default webSocketManager;
```

## WebSocket Configuration

The dev server proxies `/api` to the robot backend. Update `vite.config.ts` to change the target:

```typescript
proxy: {
  '/api': {
    target: 'ws://localhost:8889',
    ws: true,
  }
}
```
