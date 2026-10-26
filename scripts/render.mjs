import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { resolve } from 'node:path';
import { renderProjects } from '../content/render-projects.mjs';

const root = fileURLToPath(new URL('../', import.meta.url));

export async function renderPage() {
  const html = await readFile(resolve(root, 'index.html'), 'utf8');
  const projects = JSON.parse(await readFile(resolve(root, 'content/projects.json'), 'utf8'));
  const pattern = /<!-- PROJECTS:start -->[\s\S]*?<!-- PROJECTS:end -->/;
  if (!pattern.test(html)) throw new Error('Project rendering markers are missing from index.html.');
  return html.replace(pattern, () => `<!-- PROJECTS:start -->\n${renderProjects(projects)}\n<!-- PROJECTS:end -->`);
}
