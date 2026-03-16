import manifest from '../assets-manifest.json?raw';
const hashes = JSON.parse(manifest);

export function appendHash(url: string): string {
  const urlPath = url.startsWith('/') ? url.substring(1) : url;
  const hash = hashes[urlPath];
  
  if (!hash) {
    console.warn(`No hash found for asset: ${url}`);
    return url;
  }
  
  const separator = url.includes('?') ? '&' : '?';
  return `${url}${separator}hash=${hash}`;
}
