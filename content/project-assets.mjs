import { readFile, realpath, stat } from "node:fs/promises";
import { fileURLToPath } from "node:url";
import { resolve, sep } from "node:path";

const root = fileURLToPath(new URL("../", import.meta.url));

export function projectAssetPaths(projects) {
  return [
    ...new Set(
      projects
        .flatMap(({ resources: r }) => [
          ...r.downloads,
          ...r.images,
          ...r.videos.filter((video) => video.kind === "local"),
          ...r.fileGroups.flatMap((group) => group.files),
        ])
        .map((file) => file.path),
    ),
  ];
}

export async function loadProjects() {
  const projects = JSON.parse(
    await readFile(resolve(root, "content/projects.json"), "utf8"),
  );
  for (const project of projects) {
    const r = project.resources;
    const links = [
      r.github,
      r.liveDemo,
      ...r.drive.map((item) => item.url),
      ...r.videos
        .filter((video) => video.kind !== "local")
        .map((video) => video.url),
      ...(r.hostedFiles || []).flatMap((file) => [file.url, file.downloadUrl]),
    ].filter(Boolean);
    for (const link of links) {
      if (new URL(link).protocol !== "https:")
        throw new Error(`Invalid resource URL in ${project.id}`);
    }
    if (r.github && new URL(r.github).hostname !== "github.com")
      throw new Error("Expected a GitHub repository URL");
    for (const item of r.drive) {
      if (new URL(item.url).hostname !== "drive.google.com")
        throw new Error("Expected a Google Drive URL");
    }
  }
  for (const path of projectAssetPaths(projects)) {
    if (
      !path.startsWith("/assets/") ||
      path
        .split("/")
        .some(
          (part) =>
            part.startsWith(".") ||
            ["build", "install", "log", "__pycache__"].includes(part),
        )
    ) {
      throw new Error(`Invalid public project asset: ${path}`);
    }
    const file = await realpath(resolve(root, `.${path}`));
    if (!file.startsWith(`${root}assets${sep}`) || !(await stat(file)).isFile())
      throw new Error(`Missing project resource: ${path}`);
  }
  return projects;
}
