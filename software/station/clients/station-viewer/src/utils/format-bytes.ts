export function formatPtrBytes(ptr: Uint8Array): string {
  if (!ptr || ptr.length === 0) return 'N/A';

  // Parse as BigInt using little-endian
  let bigintValue = 0n;
  for (let i = ptr.length - 1; i >= 0; i--) {
    bigintValue = (bigintValue << 8n) | BigInt(ptr[i]);
  }

  // Format BigInt as string, truncate if too long (> 20 chars)
  let numStr = bigintValue.toString();
  if (numStr.length > 20) {
    numStr = numStr.slice(0, 17) + '...';
  }

  // Format hex, show first 8 bytes max
  const hexBytes = Array.from(ptr.slice(0, 8))
    .map(b => b.toString(16).padStart(2, '0'))
    .join(' ');
  const hexStr = ptr.length > 8 ? `${hexBytes}...` : hexBytes;

  return `${numStr} (${hexStr})`;
}
