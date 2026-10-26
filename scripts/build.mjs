import { cp, mkdir, readFile, rm, stat, writeFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { resolve } from 'node:path';
import { renderPage } from './render.mjs';

const root = fileURLToPath(new URL('../', import.meta.url));
const output = resolve(root, 'dist');
const files = [
  'index.html', 'style.css', 'script.js', 'assets',
  'robots.txt', 'sitemap.xml', 'LICENSE',
];

// Validate inputs before replacing the generated output directory.
for (const file of files) await stat(resolve(root, file));
const resume = await readFile(resolve(root, 'assets/resume/Monojit-Layek-Resume.pdf'));
if (resume.subarray(0, 5).toString() !== '%PDF-') {
  throw new Error('The downloadable resume is not a valid PDF.');
}
const html = await renderPage();
await rm(output, { recursive: true, force: true });
await mkdir(output, { recursive: true });
for (const file of files) {
  await cp(resolve(root, file), resolve(output, file), { recursive: true });
}
await writeFile(resolve(output, 'index.html'), html);
console.log('Production site built in dist/; project content, resume, and static assets included.');
