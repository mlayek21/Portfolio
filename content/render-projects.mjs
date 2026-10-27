// One data source, rendered at build time: searchable and usable without JavaScript.
const escapeHTML = (value) =>
  String(value).replace(
    /[&<>"']/g,
    (character) =>
      ({
        "&": "&amp;",
        "<": "&lt;",
        ">": "&gt;",
        '"': "&quot;",
        "'": "&#39;",
      })[character],
  );

const localURL = (path) => path.split("/").map(encodeURIComponent).join("/");
const fileType = (path) =>
  path.split("/").pop().includes(".")
    ? path.split(".").pop().toUpperCase()
    : "TEXT";
const externalLink = (url, label) =>
  `<a class="resource-link" href="${escapeHTML(url)}" target="_blank" rel="noopener noreferrer">${escapeHTML(label)} <span aria-hidden="true">↗</span></a>`;
function renderFiles(files) {
  return `<ul class="resource-files">${files.map((file) => `<li><span class="file-icon" aria-hidden="true">↓</span><div><strong>${escapeHTML(file.label)}</strong><span class="file-meta">${escapeHTML(file.path.split("/").pop())} · ${escapeHTML(fileType(file.path))}</span></div><a class="resource-download" href="${escapeHTML(localURL(file.path))}" download aria-label="Download ${escapeHTML(file.path.split("/").pop())}">Download</a></li>`).join("")}</ul>`;
}
function renderResources(project) {
  const r = project.resources;
  const e = escapeHTML;
  const links = [
    r.github ? externalLink(r.github, "View on GitHub") : "",
    ...r.drive.map((item) =>
      externalLink(item.url, item.label || "View on Google Drive"),
    ),
    r.liveDemo ? externalLink(r.liveDemo, "Open live demo") : "",
  ].join("");
  const videos = r.videos
    .map((video, index) => {
      if (video.kind === "drive") {
        const preview = new URL(video.url);
        preview.pathname = preview.pathname.replace(/\/view$/, "/preview");
        return `<figure class="resource-video"><figcaption>${e(video.label)}</figcaption><iframe loading="lazy" src="${e(preview.href)}" title="${e(video.label)}" allow="fullscreen; encrypted-media" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>${externalLink(video.url, "Watch Video on Google Drive")}<p>${e(video.note || "")}</p></figure>`;
      }
      if (video.kind === "youtube") {
        const id = new URL(video.url).searchParams.get("v");
        if (!/^[a-zA-Z0-9_-]{11}$/.test(id))
          throw new Error("Invalid YouTube video ID");
        return `<figure class="resource-video"><figcaption>${e(video.label)}</figcaption><iframe loading="lazy" src="https://www.youtube-nocookie.com/embed/${e(id)}" title="${e(video.label)}" allow="encrypted-media; picture-in-picture; fullscreen" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>${externalLink(video.url, "Watch Video on YouTube")}</figure>`;
      }
      const path = e(localURL(video.path));
      return `<figure class="resource-video"><figcaption id="video-${e(project.id)}-${index}">${e(video.label)}</figcaption><video controls playsinline preload="none" aria-labelledby="video-${e(project.id)}-${index}" data-project-video><source src="${path}" ${video.type === "video/quicktime" ? "" : `type="${e(video.type)}"`}>Your browser does not support this video. Use the video links below.</video><div class="video-links"><a class="resource-link" href="${path}" target="_blank" rel="noopener noreferrer">Watch Video <span class="sr-only">${e(video.label)}</span> ↗</a><a class="resource-download" href="${path}" download>Download Video <span class="sr-only">${e(video.label)}</span></a></div><p class="video-error" hidden>This browser could not play this file. Use Watch Video or Download Video to open the original recording.</p></figure>`;
    })
    .join("");
  return `<section class="project-resources" aria-labelledby="resources-${e(project.id)}"><h4 id="resources-${e(project.id)}">Project resources<span class="sr-only">: ${e(project.title)}</span></h4>
    ${links ? `<div class="resource-links">${links}</div>` : ""}
    ${r.downloads.length ? `<h5>Documents &amp; downloads</h5>${renderFiles(r.downloads)}` : ""}
    ${r.hostedFiles?.length ? `<h5>Files on Google Drive</h5><ul class="resource-files">${r.hostedFiles.map((file) => `<li><span class="file-icon" aria-hidden="true">↓</span><div><strong><a href="${e(file.url)}" target="_blank" rel="noopener noreferrer">${e(file.label)}</a></strong><span class="file-meta">${e(file.filename)} · ${e(fileType(file.filename))} · Google Drive</span></div><a class="resource-download" href="${e(file.downloadUrl)}" target="_blank" rel="noopener noreferrer" aria-label="Download ${e(file.filename)}">Download</a></li>`).join("")}</ul>` : ""}
    ${videos ? `<h5>Demonstration videos</h5><div class="resource-videos">${videos}</div>` : ""}
    ${r.images.length ? `<h5>Project images &amp; plots</h5><div class="resource-gallery">${r.images.map((image) => `<figure><a href="${e(localURL(image.path))}" target="_blank" rel="noopener noreferrer"><img src="${e(localURL(image.path))}" alt="${e(image.alt)}" loading="lazy" decoding="async"></a><figcaption>${e(image.label)} <a href="${e(localURL(image.path))}" download aria-label="Download ${e(image.label)}">Download</a></figcaption></figure>`).join("")}</div>` : ""}
    ${r.fileGroups.map((group) => `<details class="resource-file-group"><summary>${e(group.label)} <span>(${group.files.length} files)</span></summary>${renderFiles(group.files)}</details>`).join("")}
    ${r.driveUrlNeeded && !r.drive.length ? '<p class="resource-missing">Google Drive: URL needed for the MRE project.</p>' : ""}
  </section>`;
}

export function renderProjects(projects) {
  return projects
    .map((project, index) => {
      const e = escapeHTML;
      if (!/^[a-z][a-z0-9-]*$/.test(project.id))
        throw new Error("Invalid project id");
      if (!/^[a-z-]+\.svg$/.test(project.image))
        throw new Error("Invalid project image");
      return `        <article class="project-card" id="project-${e(project.id)}" data-category="${e(project.category)}" aria-labelledby="title-${e(project.id)}">
          <figure class="project-figure"><img src="/assets/diagrams/${e(project.image)}" width="640" height="360" loading="lazy" decoding="async" alt="${e(project.imageAlt)}"><span class="project-number" aria-hidden="true">PROJECT / ${String(index + 1).padStart(2, "0")}</span><figcaption>CONCEPTUAL SCHEMATIC</figcaption></figure>
          <div class="project-body">
            <div class="project-meta"><span>${e(project.categoryLabel)}</span><time datetime="${e(project.datetime)}">${e(project.date)}</time></div>
            <h3 id="title-${e(project.id)}">${e(project.title)}</h3>
            <p class="project-subtitle">${e(project.subtitle)}</p>
            <p class="project-description">${e(project.summary)}</p>
            <ul class="tags" aria-label="Technologies">${project.tags.map((tag) => `<li>${e(tag)}</li>`).join("")}</ul>
            <p class="project-outcome">${e(project.outcome)}</p>
            <details class="project-details"><summary aria-controls="case-${e(project.id)}"><span class="expand-label">Expand Project</span><span class="collapse-label">Collapse Project</span><span class="sr-only">: ${e(project.title)}</span></summary><div class="case-study" id="case-${e(project.id)}">
              ${renderResources(project)}
              <p class="case-organization">${e(project.organization)}</p>
              <h4>Engineering problem</h4><p>${e(project.problem)}</p>
              <h4>Architecture &amp; approach</h4><ol class="pipeline" aria-label="Engineering pipeline">${project.pipeline.map((step) => `<li>${e(step)}</li>`).join("")}</ol><p>${e(project.method)}</p>
              <h4>Technical contributions</h4><ul class="contribution-list">${project.contributions.map((item) => `<li>${e(item)}</li>`).join("")}</ul>
              <h4>Evaluation &amp; scope</h4><p>${e(project.validation)}</p>

            </div></details>
          </div>
        </article>`;
    })
    .join("\n").replace(/[ \t]+$/gm, "");
}
