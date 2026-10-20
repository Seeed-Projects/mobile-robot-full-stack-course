# Contributing

## Scope

This repository contains course documentation, per-module code, hardware notes, project integration assets, and media resources. Keep those concerns separated.

## Authoring Rules

1. Keep `README.md` and `README_CN.md` semantically aligned when both are updated.
2. Put course documentation under `docs/`.
3. Put per-module code and module-specific assets under `modules/`.
4. Keep module-level learning outcomes, prerequisites, and deliverables explicit.
5. Prefer Markdown unless a binary artifact is necessary.
6. Avoid committing large raw datasets or generated model artifacts directly into Git.

## Directory Rules

- `hardware/`: platform notes, BOMs, wiring, calibration, CAD references
- `docs/`: syllabus, module map, repository notes, roadmap, localization notes
- `modules/`: module code, shared ROS 2 workspace, Docker, scripts, configs
- `projects/`: integrated projects, demos, evaluation, reusable templates
- `assets/`: images, videos, and shared visual resources

## Pull Request Expectations

- Explain which module, project, or document changed.
- Note any prerequisite changes.
- Document new dependencies.
- Update the relevant `README.md` files if navigation or scope changed.
- Keep filenames stable and descriptive.

## Translation Workflow

1. Update the Chinese and English README files together when they describe the same repository state.
2. Keep section order and meaning aligned across languages.
3. Keep module names and dates consistent across both versions.

## Documentation Checks

Every push to `main` and every pull request runs `.github/workflows/docs-check.yml`:

| Check | Scope | Blocks merge |
| --- | --- | --- |
| Markdown format (`markdownlint-cli2`) | `docs/`, `hardware/`, root and `.github` markdown files; `**/code/**` excluded | Yes |
| Local file links and in-page anchors (`lychee --offline`) | same | Yes |
| External links (`lychee`) | same | No — report only |

Run the same commands locally before pushing:

```bash
npx --yes markdownlint-cli2@0.23.3
lychee --offline --no-progress --include-fragments --exclude-path code/ \
  'docs/**/*.md' 'hardware/**/*.md' '*.md'
```

Rules and ignore patterns live in `.markdownlint-cli2.jsonc`. That file also lists the two rules that are currently off (`MD040` fenced-code-language, `MD001` heading-increment) together with the files that still violate them; flip each rule back to `true` once those are cleaned up.
