import { readdir, readFile, stat } from 'node:fs/promises';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { dirname, resolve, relative, sep } from 'node:path';
import { renderPage } from './render.mjs';

const root = resolve(fileURLToPath(new URL('../', import.meta.url)));
const problems = [];
const html = await renderPage();
const ids = [...html.matchAll(/\bid="([^"]+)"/g)].map(match => match[1]);
const idSet = new Set(ids);
for (const id of idSet) if (ids.filter(value => value === id).length > 1) problems.push(`Duplicate HTML id: ${id}`);
if (!/<html\b[^>]*\blang="en"/.test(html)) problems.push('Missing English document language.');
if (!/<title>[^<]+<\/title>/.test(html)) problems.push('Missing page title.');
if (!/<meta\s+name="description"\s+content="[^"]+"/.test(html)) problems.push('Missing SEO description.');
if ((html.match(/<h1\b/g) ?? []).length !== 1) problems.push('The page must have exactly one h1.');
for (const image of html.matchAll(/<img\b[^>]*>/g)) {
  if (!/\balt="[^"]*"/.test(image[0])) problems.push(`Image missing alt text: ${image[0]}`);
}
for (const anchor of html.matchAll(/<a\b[^>]*>/g)) {
  if (/target="_blank"/.test(anchor[0]) && !/rel="[^"]*noopener/.test(anchor[0])) {
    problems.push('An external new-tab link is missing rel="noopener".');
  }
}
for (const match of html.matchAll(/\b(?:href|src)="([^"]+)"/g)) {
  const value = match[1];
  if (value === '#' || /^javascript:/i.test(value)) {
    problems.push(`Placeholder or invalid link: ${value}`);
  } else if (value.startsWith('#')) {
    if (!idSet.has(value.slice(1))) problems.push(`Missing internal anchor: ${value}`);
  } else if (/^(?:https:|mailto:|tel:|data:)/.test(value)) {
    if (value.startsWith('https:')) {
      try { new URL(value); } catch { problems.push(`Invalid external URL: ${value}`); }
    }
  } else {
    const asset = resolve(root, value.split(/[?#]/)[0].replace(/^\//, ''));
    try { await stat(asset); } catch { problems.push(`Missing local asset: ${value}`); }
  }
}

async function collect(directory) {
  const files = [];
  for (const entry of await readdir(directory, { withFileTypes: true })) {
    if (['.git', 'node_modules', 'dist', 'img', 'My_resume'].includes(entry.name)) continue;
    const file = resolve(directory, entry.name);
    if (entry.isDirectory()) files.push(...await collect(file));
    else if (/\.(?:js|mjs)$/.test(entry.name)) files.push(file);
  }
  return files;
}
const scripts = await collect(root);
for (const script of scripts) {
  try { execFileSync(process.execPath, ['--check', script], { stdio: 'pipe' }); }
  catch (error) { problems.push(`${relative(root, script)}: ${error.stderr?.toString() ?? error.message}`); }
  const source = await readFile(script, 'utf8');
  for (const match of source.matchAll(/(?:import|export)\s+(?:[^'";]*?\s+from\s+)?['"](\.[^'"]+)['"]/g)) {
    const dependency = resolve(dirname(script), match[1]);
    if (!dependency.startsWith(`${root}${sep}`)) problems.push(`Import outside project: ${match[1]}`);
    try { await stat(dependency); } catch { problems.push(`Missing imported module: ${match[1]}`); }
  }
}
if (problems.length) {
  console.error(problems.map(problem => `• ${problem}`).join('\n'));
  process.exitCode = 1;
} else {
  console.log(`Checks passed: ${scripts.length} JavaScript files, document structure, anchors, local assets, and external URL syntax.`);
}
