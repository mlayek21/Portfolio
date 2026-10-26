# Monojit Layek — Robotics Engineering Portfolio

A responsive portfolio focused on robotic motion planning, controls, mechatronic hardware, and numerical optimization. The existing repository's HTML/CSS/JavaScript stack is retained. No framework, runtime package, or third-party CDN is required.

## Local preview and checks

Use Node.js 22 or newer. There are no dependencies to install.

```sh
npm run dev
```

Open <http://127.0.0.1:5173>. To check and preview the production files:

```sh
npm run lint
npm run build
npm run preview
```

`npm run lint` checks JavaScript syntax and imports, HTML structure, image alt attributes, navigation targets, local assets, and external URL syntax. It does not verify remote services or substitute for browser accessibility checks. There was no pre-existing automated test suite; no test framework was added.

`npm run build` renders project cards from `content/projects.json`, publishes an allowlist of site files to `dist/`, checks the résumé is a PDF, and excludes old stock images, source tooling, Git metadata, and documentation. `npm run preview` serves that output. Both servers use port 5173; stop one before starting the other, or set a different `PORT`.

## Redesign verification

The production build was checked in headless Google Chrome using temporary Playwright and axe tooling outside this repository. No browser-testing packages were added to the project.

- `npm run lint` and `npm run build` pass.
- No horizontal overflow at 320, 390, 768, 1024, and 1440 pixel widths.
- Project filters, live counts, case-study keyboard controls, and deep links to filtered projects work.
- Mobile menu opening, Escape with focus return, and closing after navigation work.
- Desktop and mobile axe scans found zero WCAG A/AA violations in the scanned states. Automated scans do not establish complete accessibility conformance.
- Project content, navigation, and expandable case studies work with JavaScript disabled.
- The production PDF returns HTTP 200 with the correct content type and matches the supplied résumé byte-for-byte.
- All project illustrations load; no JavaScript exceptions or local asset HTTP failures were observed.

The browser pass used Chrome; Safari and Firefox have not been tested. Netlify production deployment and its response headers should be checked after publication.

## Source layout

| Path | Purpose |
| --- | --- |
| `index.html` | Semantic page content, navigation, contact links, and metadata |
| `style.css` | Responsive dark theme and reduced-motion support |
| `script.js` | Navigation and project interactions |
| `content/projects.json` | Reusable technical project data |
| `content/render-projects.mjs` | Server/build-time project renderer; no browser dependency |
| `assets/diagrams/` | Clearly labeled conceptual illustrations |
| `assets/resume/Monojit-Layek-Resume.pdf` | Latest supplied résumé, available in production |
| `assets/favicon.svg` | Site icon |
| `robots.txt`, `sitemap.xml` | Search discovery for the existing domain |
| `scripts/` | Dependency-free preview, validation, and production build |
| `netlify.toml` | Netlify build and security headers |

Edit project descriptions in `content/projects.json`, then run `npm run build`. The development server renders project data on each page request; production and direct-file hosting retain readable project content without JavaScript. Changes to renderer code require restarting the development server.

The old `img/` directory and MIT license are retained in the repository. The original generic gallery, inactive blog/service links, CDN animation/carousel scripts, and SMTP form were replaced by relevant project content and working email links. No backend or API credentials are required.

The original page contained a client-side SMTP credential. It has been removed from the site; revoke that legacy credential if it is still active, since previous Git history can retain it.

## Netlify

Keep the existing Netlify site connected to this repository. The committed configuration uses `npm run build`, the `dist` publish directory, and Node.js 22. No environment variables are needed. After merging/pushing the changes, Netlify can deploy through its existing Git integration. This local implementation does not itself push or deploy.

Retain the existing `mlayek.netlify.app` domain. If it changes, update the canonical URL, social metadata, `robots.txt`, and `sitemap.xml` together. The site uses ordinary anchor navigation; no single-page-app fallback rewrite is necessary.

## Content and media provenance

Professional facts and dates come from the supplied `MonojitLayekResume.pdf`, which is copied verbatim for downloading. AeThrone's role title is **System Design Engineer**, per the portfolio owner's correction. The supplied résumé PDF retains its original wording. Research and personal projects are separate from full-time employment.

The **2–3.1× runtime improvement** is a résumé-reported result for the research benchmarks, not a general performance guarantee. Schematics are illustrative and are not simulation screenshots, fabricated experimental results, or photographs of completed hardware.

The Stewart platform's [public source repository](https://github.com/mlayek21/Stewart-Platform) was verified. No verified source repository for the six-axis Dobot project or MRE prototype, and no verified publication URL for the Kanpur–Zoutendijk work, was supplied. Their links are intentionally omitted. The distinct three-DOF planning repository must not be presented as the six-axis Dobot source.

Useful follow-up assets:

- Actual RViz/Gazebo/PyBullet captures and trajectory plots for the six-axis manipulator.
- Stewart platform CAD screenshots and simulation media.
- MRE prototype photographs, CAD assembly captures, and measured results.
- The optimization paper's DOI or publisher URL and benchmark methodology/data.
- Verified project repository and credential links, where available.

Dates, education, certificates, contacts, and research results should be reviewed by the portfolio owner when updating the résumé.

The résumé's LinkedIn URL (`https://www.linkedin.com/in/mlayek`) returned HTTP 404 during automated verification. It is retained exactly as supplied; confirm the current profile URL. The GitHub profile, Stewart repository, and existing Netlify domain returned HTTP 200.
