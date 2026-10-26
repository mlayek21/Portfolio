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

export function renderProjects(projects) {
  return projects
    .map((project, index) => {
      const e = escapeHTML;
      if (!/^[a-z][a-z0-9-]*$/.test(project.id))
        throw new Error("Invalid project id");
      if (!/^[a-z-]+\.svg$/.test(project.image))
        throw new Error("Invalid project image");
      if (project.source && new URL(project.source).protocol !== "https:")
        throw new Error("Project source must use HTTPS");
      return `        <article class="project-card" id="project-${e(project.id)}" data-category="${e(project.category)}" aria-labelledby="title-${e(project.id)}">
          <figure class="project-figure"><img src="/assets/diagrams/${e(project.image)}" width="640" height="360" loading="lazy" decoding="async" alt="${e(project.imageAlt)}"><span class="project-number" aria-hidden="true">PROJECT / ${String(index + 1).padStart(2, "0")}</span><figcaption>CONCEPTUAL SCHEMATIC</figcaption></figure>
          <div class="project-body">
            <div class="project-meta"><span>${e(project.categoryLabel)}</span><time datetime="${e(project.datetime)}">${e(project.date)}</time></div>
            <h3 id="title-${e(project.id)}">${e(project.title)}</h3>
            <p class="project-subtitle">${e(project.subtitle)}</p>
            <p class="project-description">${e(project.summary)}</p>
            <ul class="tags" aria-label="Technologies">${project.tags.map((tag) => `<li>${e(tag)}</li>`).join("")}</ul>
            <p class="project-outcome">${e(project.outcome)}</p>
            <details class="project-details"><summary>Explore case study<span class="sr-only">: ${e(project.title)}</span></summary><div class="case-study">
              <p class="case-organization">${e(project.organization)}</p>
              <h4>Engineering problem</h4><p>${e(project.problem)}</p>
              <h4>Architecture &amp; approach</h4><ol class="pipeline" aria-label="Engineering pipeline">${project.pipeline.map((step) => `<li>${e(step)}</li>`).join("")}</ol><p>${e(project.method)}</p>
              <h4>Technical contributions</h4><ul class="contribution-list">${project.contributions.map((item) => `<li>${e(item)}</li>`).join("")}</ul>
              <h4>Evaluation &amp; scope</h4><p>${e(project.validation)}</p>
              ${project.source ? `<a class="text-link" href="${e(project.source)}" target="_blank" rel="noopener">View project on GitHub <span aria-hidden="true">↗</span></a>` : ""}
              <div class="media-placeholder"><strong>ORIGINAL PROJECT MEDIA / TO BE ADDED</strong>${e(project.media)}</div>
            </div></details>
          </div>
        </article>`;
    })
    .join("\n");
}
