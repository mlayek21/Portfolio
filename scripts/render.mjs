import { readFile } from "node:fs/promises";
import { fileURLToPath } from "node:url";
import { resolve } from "node:path";
import { renderProjects } from "../content/render-projects.mjs";
import { loadProjects } from "../content/project-assets.mjs";

const root = fileURLToPath(new URL("../", import.meta.url));

export async function renderPage() {
  const html = await readFile(resolve(root, "index.html"), "utf8");
  const projects = await loadProjects();
  const pattern = /<!-- PROJECTS:start -->[\s\S]*?<!-- PROJECTS:end -->/;
  if (!pattern.test(html))
    throw new Error("Project rendering markers are missing from index.html.");
  return html.replace(
    pattern,
    () =>
      `<!-- PROJECTS:start -->\n${renderProjects(projects)}\n<!-- PROJECTS:end -->`,
  );
}
