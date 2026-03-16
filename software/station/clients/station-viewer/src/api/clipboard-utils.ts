export async function copyToClipboard(text: string): Promise<void> {
  if (navigator.clipboard && window.isSecureContext) {
    await navigator.clipboard.writeText(text);
    return;
  }

  const textarea = document.createElement('textarea');
  textarea.value = text;

  textarea.style.position = 'fixed';
  textarea.style.top = '0';
  textarea.style.left = '0';
  textarea.style.opacity = '0';
  textarea.style.pointerEvents = 'none';
  textarea.style.fontSize = '16px';

  textarea.readOnly = true;

  document.body.appendChild(textarea);

  textarea.select();
  textarea.setSelectionRange(0, text.length);

  try {
    const success = document.execCommand('copy');
    if (!success) {
      throw new Error('Copy command failed');
    }
  } finally {
    document.body.removeChild(textarea);
  }
}
