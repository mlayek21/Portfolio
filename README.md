# Monojit Layek — Robotics Engineering Portfolio

A responsive portfolio focused on robotic motion planning, controls, mechatronic hardware, and numerical optimization. The existing repository's HTML/CSS/JavaScript stack is retained. No framework, runtime package, or third-party CDN is required.

## Local preview and checks

Use Node.js 22 (pinned in `.nvmrc` and `netlify.toml`). There are no runtime dependencies. `npm ci` verifies the committed lockfile for a clean setup.

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

`npm run build` first runs the validation checks, then renders project cards from `content/projects.json`, publishes an allowlist of site files to `dist/`, checks the résumé is a PDF, and excludes old stock images, source tooling, Git metadata, and documentation. `npm run preview` serves that output. Both servers use port 5173; stop one before starting the other, or set a different `PORT`.

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

Keep the existing `mlayek` Netlify project connected to `https://github.com/mlayek21/Portfolio`, with production branch **main** and automatic publishing enabled. The root-level `netlify.toml` specifies:

The production project uses GitHub as its source provider, a read-only deployment key, and a push webhook on `mlayek21/Portfolio`. Keep these account-level settings pointed at this repository when changing Netlify configuration. Deployment credentials and local `.netlify/` state are not committed.

No Git submodules are required. The résumé is a regular tracked PDF in `assets/resume/`, so a fresh Netlify checkout does not depend on an external résumé repository.

| Setting | Value |
| --- | --- |
| Base directory | Repository root (`.`) |
| Package directory | Leave unset |
| Build command | `npm run build` |
| Publish directory | `dist` |
| Node.js | `22` |

The build validates the source before generating the deployable files. The lockfile supports clean installs; no application secrets, server functions, or framework adapters are required. Do not set `dist` as the base directory: Netlify generates it during the build.

Pushing to the connected production branch triggers Netlify's Git deployment. If the latest commit is not deployed, check **Deploys** for a failed build, paused builds, or locked publishing. For a one-time recovery, retry the latest commit with a cleared build cache. Paste the first build error and surrounding log lines when diagnosing a failure; a successful local build alone does not prove that the hosted deployment succeeded.

After deployment, verify `/`, `/style.css`, `/script.js`, and `/assets/resume/Monojit-Layek-Resume.pdf`; the homepage should contain **System Design Engineer**. Production HTML and assets are generated locally with `npm run build`, and can also be uploaded from `dist/` through the existing project's manual-deploy interface.

### Accessibility plugin failures

No Netlify Build Plugin is required for this portfolio. Leave the legacy UI-installed `netlify-plugin-a11y@0.0.12` disabled: it previously timed out after 30 seconds in `onPostBuild`, after the production build had already succeeded. Source validation still runs before every production build, and the browser accessibility checks described above remain part of the verification record.

The build configuration keeps the publish directory at `dist`, so audits only inspect the generated site. Hero social-link arrows were lightened after the legacy pa11y/HTML_CodeSniffer runner reported three contrast findings. A local audit with the plugin's pa11y 5.3.1 runner then checks the built homepage with no findings; this does not verify Netlify's browser installation or account-level plugin configuration.

If that plugin is re-enabled, its published `0.0.12` package uses `resultMode`, not the newer scoped plugin's `failWithIssues` input. Neither reporting-only settings nor suppressing accessibility findings fixes a runner timeout. Disable the plugin in Netlify's project settings if this legacy timeout returns.

These settings follow Netlify's [file-based configuration](https://docs.netlify.com/build/configure-builds/file-based-configuration/) and [build dependency documentation](https://docs.netlify.com/build/configure-builds/manage-dependencies/).

Retain the existing `mlayek.netlify.app` domain. If it changes, update the canonical URL, social metadata, `robots.txt`, and `sitemap.xml` together. The site uses ordinary anchor navigation; no single-page-app fallback rewrite is necessary.

## Content and media provenance

Professional facts and dates come from the latest supplied `MonojitLayekResume.pdf`, which is copied verbatim for downloading. It lists AeThrone's role as **System Design Engineer**, March–August 2023, and the B.Tech graduation date as June 2018. The website reflects those dates. Research and personal projects are separate from full-time employment.

The **2–3.1× runtime improvement** is a résumé-reported result for the research benchmarks, not a general performance guarantee. Schematics are illustrative and are not simulation screenshots, fabricated experimental results, or photographs of completed hardware.

## Project resources

Each card has a native keyboard-accessible **Expand Project / Collapse Project** control. The same `content/projects.json` records supply the static HTML, build validation, downloads, project links, and media. Resources appear before the engineering case study, and additional source/model files are grouped in their own disclosures.

| Project | Connected resources |
| --- | --- |
| Dobot | Two original H.264 MOV recordings, setup guide, and the supplied robot source/configuration/mesh files. |
| Stewart platform | Public GitHub repository, PDF documentation, ZIP model, Fusion CAD, notebook, four local MP4 simulations, the existing YouTube demo, nine images/plots, and supporting source/URDF/mesh files. |
| Magnetic Reciprocating Actuator (MRE) | Owner-supplied Google Drive folder, its CAD and Inventor subfolders, DOCX report, PPTX presentation, two photographs, Arduino sketch, and Drive-hosted AVI demonstration. |
| Kanpur–Zoutendijk | Existing `assets/kzountendijk/Kanpur_zoutendijk.pdf` research document. |

MRE uses its own exact [Google Drive folder](https://drive.google.com/drive/folders/0B8mi-UIPyFnQMjFPVi15czdRMWM?resourcekey=0-Eh_bgKei0EmU-U8omZkVig&usp=drive_link). File IDs and per-file resource keys were read from that public folder. Five smaller files returned successful download responses with `Content-Disposition: attachment`; their Download controls use those verified URLs. The 1.17 GB AVI requires Google's download-confirmation flow, so it has a Drive preview and Watch Video link, without being presented as a direct download. Folder sharing links remain folder links.

Local recordings use the original files, with no duplicate conversions. MOV sources omit the HTML source `type` hint to let browsers detect their H.264 contents; the server still sends the correct QuickTime MIME type. Original local videos have HTML5 controls, Watch Video and Download Video links, no autoplay, and `preload="none"`. Embedded YouTube/Drive players load on expansion and unload on collapse or filtering. Third-party playback still depends on the hosting service and browser. Local video serving supports byte ranges for seeking. Production CSP permits only the two required video iframe hosts.

The build publishes only files explicitly referenced by project data, plus the existing website assets. Nested `.git` repositories, ROS build/install/log directories, environments, caches, and unrelated vendored files are excluded. The supplied nested Dobot checkout remains intact locally; its selected assets are ordinary files in the portfolio Git tree, not a submodule. Asset paths with spaces and Unicode are URL-encoded and checked before publication.

Still unconfigured because no public resource was supplied:

- **Dobot public GitHub button:** the discovered `mlayek21/ros2-dobot-manipulator` repository is private and returns HTTP 404 to visitors. Its public button is intentionally omitted. Repository visibility has not been changed.
- **MRE and optimization GitHub repositories:** no project-specific public URLs supplied.
- **Optimization publisher/DOI, source code, benchmark data, and video:** no verified URLs or additional files supplied. The existing PDF is connected.
- **Google Drive resources for Dobot, Stewart, or optimization:** none supplied or found; no Drive buttons are invented.

Dates, education, certificates, contacts, and research results should be reviewed by the portfolio owner when updating the résumé.

The résumé's LinkedIn URL (`https://www.linkedin.com/in/mlayek`) returned HTTP 404 during automated verification. It is retained exactly as supplied; confirm the current profile URL. The GitHub profile, Stewart repository, and existing Netlify domain returned HTTP 200.
