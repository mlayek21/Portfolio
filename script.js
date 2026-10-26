// Progressive enhancement: the portfolio remains readable without JavaScript.
const menuButton = document.querySelector(".menu-toggle");
const navigation = document.querySelector("#nav-links");
document.documentElement.classList.add("js");

function closeMenu(returnFocus = false) {
  menuButton.setAttribute("aria-expanded", "false");
  menuButton.setAttribute("aria-label", "Open navigation");
  navigation.classList.remove("is-open");
  if (returnFocus) menuButton.focus();
}
menuButton.addEventListener("click", () => {
  const open = menuButton.getAttribute("aria-expanded") !== "true";
  menuButton.setAttribute("aria-expanded", String(open));
  menuButton.setAttribute(
    "aria-label",
    open ? "Close navigation" : "Open navigation",
  );
  navigation.classList.toggle("is-open", open);
});
navigation.addEventListener("click", (event) => {
  if (event.target.closest("a")) closeMenu();
});
document.addEventListener("keydown", (event) => {
  if (
    event.key === "Escape" &&
    menuButton.getAttribute("aria-expanded") === "true"
  )
    closeMenu(true);
});
document.addEventListener("click", (event) => {
  if (!event.target.closest(".navigation")) closeMenu();
});
navigation.addEventListener("focusout", () => {
  requestAnimationFrame(() => {
    if (!document.querySelector(".navigation").contains(document.activeElement))
      closeMenu();
  });
});
window.matchMedia("(min-width: 1081px)").addEventListener("change", (event) => {
  if (event.matches) closeMenu();
});

const filters = document.querySelector(".project-filters");
const projects = [...document.querySelectorAll(".project-card")];
const count = document.querySelector(".project-count");
filters.hidden = false;
function filterProjects(category) {
  let visible = 0;
  for (const project of projects) {
    const show =
      category === "all" ||
      project.dataset.category.split(" ").includes(category);
    project.hidden = !show;
    if (show) visible += 1;
  }
  for (const filter of filters.querySelectorAll("button")) {
    const active = filter.dataset.filter === category;
    filter.classList.toggle("active", active);
    filter.setAttribute("aria-pressed", String(active));
  }
  count.textContent = `${String(visible).padStart(2, "0")} PROJECT${visible === 1 ? "" : "S"}`;
}
filters.addEventListener("click", (event) => {
  const button = event.target.closest("button[data-filter]");
  if (button) filterProjects(button.dataset.filter);
});

// A deep link restores a filtered-out project and opens its case study.
function revealLinkedProject() {
  let id;
  try {
    id = decodeURIComponent(window.location.hash.slice(1));
  } catch {
    return;
  }
  const project = projects.find((item) => item.id === id);
  if (!project) return;
  filterProjects("all");
  project.querySelector("details").open = true;
  requestAnimationFrame(() =>
    project.scrollIntoView({ block: "start", behavior: "instant" }),
  );
}
window.addEventListener("hashchange", revealLinkedProject);
document.addEventListener("click", (event) => {
  const link = event.target.closest('a[href^="#project-"]');
  if (link && link.hash === window.location.hash) revealLinkedProject();
});
revealLinkedProject();
if ("IntersectionObserver" in window) {
  const links = [...navigation.querySelectorAll("a")];
  const observer = new IntersectionObserver(
    (entries) => {
      for (const entry of entries) {
        if (!entry.isIntersecting) continue;
        for (const link of links) {
          if (link.hash === `#${entry.target.id}`)
            link.setAttribute("aria-current", "location");
          else link.removeAttribute("aria-current");
        }
      }
    },
    { rootMargin: "-12% 0px -65% 0px", threshold: 0 },
  );
  document
    .querySelectorAll("main > section[id]")
    .forEach((section) => observer.observe(section));
}
