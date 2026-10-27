import { createServer } from "node:http";
import { createReadStream } from "node:fs";
import { realpath, stat } from "node:fs/promises";
import { fileURLToPath } from "node:url";
import { extname, resolve, sep } from "node:path";
import { renderPage } from "./render.mjs";

const projectRoot = fileURLToPath(new URL("../", import.meta.url));
const production = process.argv.includes("--dist");
const root = await realpath(resolve(projectRoot, production ? "dist" : "."));
const port = Number(process.env.PORT ?? 5173);
if (!Number.isInteger(port) || port < 1 || port > 65535)
  throw new Error("PORT must be between 1 and 65535.");
const mime = {
  ".html": "text/html; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".svg": "image/svg+xml",
  ".pdf": "application/pdf",
  ".png": "image/png",
  ".jpg": "image/jpeg",
  ".jpeg": "image/jpeg",
  ".webp": "image/webp",
  ".txt": "text/plain; charset=utf-8",
  ".xml": "application/xml; charset=utf-8",
  ".woff2": "font/woff2",
  ".mp4": "video/mp4",
  ".mov": "video/quicktime",
  ".zip": "application/zip",
};

const server = createServer(async (request, response) => {
  response.setHeader("X-Content-Type-Options", "nosniff");
  response.setHeader("Cache-Control", "no-store");
  if (!["GET", "HEAD"].includes(request.method)) {
    response.writeHead(405, { Allow: "GET, HEAD" }).end("Method not allowed");
    return;
  }
  try {
    const pathname = decodeURIComponent(
      new URL(request.url, "http://localhost").pathname,
    );
    const segments = pathname.split("/").filter(Boolean);
    if (
      segments.some((segment) => segment.startsWith(".")) ||
      segments.includes("node_modules")
    ) {
      response.writeHead(404).end("Not found");
      return;
    }
    let file = resolve(root, `.${pathname}`);
    if (pathname.endsWith("/")) file = resolve(file, "index.html");
    file = await realpath(file);
    if (!file.startsWith(`${root}${sep}`) || !(await stat(file)).isFile()) {
      response.writeHead(404).end("Not found");
      return;
    }
    const contentType =
      mime[extname(file).toLowerCase()] ?? "application/octet-stream";
    if (!production && file === resolve(root, "index.html")) {
      const data = Buffer.from(await renderPage());
      response.writeHead(200, {
        "Content-Type": contentType,
        "Content-Length": data.length,
      });
      response.end(request.method === "HEAD" ? undefined : data);
      return;
    }
    const { size } = await stat(file);
    const range = request.headers.range;
    let start = 0,
      end = size - 1;
    if (range) {
      const match = /^bytes=(\d*)-(\d*)$/.exec(range);
      if (match) {
        start = match[1]
          ? Number(match[1])
          : Math.max(0, size - Number(match[2]));
        end =
          match[1] && match[2]
            ? Math.min(Number(match[2]), size - 1)
            : size - 1;
      }
      if (!match || (!match[1] && !match[2]) || start > end || start >= size) {
        response.writeHead(416, { "Content-Range": `bytes */${size}` }).end();
        return;
      }
    }
    response.writeHead(range ? 206 : 200, {
      "Content-Type": contentType,
      "Content-Length": end - start + 1,
      "Accept-Ranges": "bytes",
      ...(range ? { "Content-Range": `bytes ${start}-${end}/${size}` } : {}),
    });
    if (request.method === "HEAD") response.end();
    else
      createReadStream(file, { start, end })
        .on("error", () => response.destroy())
        .pipe(response);
  } catch (error) {
    response.writeHead(error instanceof URIError ? 400 : 404).end("Not found");
  }
});

server.listen(port, "127.0.0.1", () =>
  console.log(`Local: http://127.0.0.1:${port}`),
);
server.on("error", (error) => {
  console.error(error.message);
  process.exitCode = 1;
});
