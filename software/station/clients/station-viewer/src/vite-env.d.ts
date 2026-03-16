/// <reference types="vite/client" />

declare const __STATION_VERSION__: string;

declare module '*assets-manifest.json?raw' {
  const content: string;
  export default content;
}
