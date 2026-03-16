import { createHash } from 'crypto';
import { readFile, readdir, writeFile } from 'fs/promises';
import { join } from 'path';

async function computeHash(filePath) {
  const content = await readFile(filePath);
  return createHash('md5').update(content).digest('hex').substring(0, 8);
}

async function findFiles(dir, extensions) {
  const files = [];
  const entries = await readdir(dir, { withFileTypes: true });
  
  for (const entry of entries) {
    const fullPath = join(dir, entry.name);
    if (entry.isDirectory()) {
      files.push(...await findFiles(fullPath, extensions));
    } else if (extensions.some(ext => entry.name.endsWith(ext))) {
      files.push(fullPath);
    }
  }
  
  return files;
}

async function main() {
  const publicDir = join(process.cwd(), 'public');
  const extensions = ['.urdf', '.stl'];
  
  const files = await findFiles(publicDir, extensions);
  const manifest = {};
  
  for (const file of files) {
    const relativePath = file.replace(publicDir + '/', '');
    const hash = await computeHash(file);
    manifest[relativePath] = hash;
    console.log(`Hashed: ${relativePath} -> ${hash}`);
  }
  
  const manifestPath = join(process.cwd(), 'src', 'assets-manifest.json');
  await writeFile(manifestPath, JSON.stringify(manifest, null, 2));
  console.log(`\nWrote manifest to ${manifestPath}`);
}

main().catch(console.error);
