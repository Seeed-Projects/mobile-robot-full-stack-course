# Repository Architecture

## Design Goal

This repository is structured for long-term open-course maintenance, not just for storing files.

It needs to support:

- course documentation
- per-module implementation
- hardware bring-up
- integrated project delivery
- public collaboration

## Top-Level Structure

### `hardware/`

Stores physical platform information such as BOMs, interface maps, wiring diagrams, calibration assets, and platform notes.

### `docs/`

Stores course-level documentation such as the syllabus, module map, roadmap, repository notes, localization notes, and deployment documentation.

### `modules/`

Stores per-module code and shared technical assets. This includes module folders as well as common resources such as ROS 2 workspaces, Docker assets, scripts, and configs.

### `projects/`

Stores integrated projects, demos, evaluation assets, and reusable templates.

### `assets/`

Stores shared images, diagrams, videos, and other media used by the repository.

## Module Pattern

Each module directory should eventually contain:

- `README.md`
- `lectures/`
- `labs/`
- `assignments/`
- `code/`
- `assets/`
- `references/`

## Localization Pattern

Localized repository notes should live under `docs/` rather than using a separate top-level content tree.

Recommended pattern:

- `docs/zh-CN/...`
- `README_CN.md`

## Project Pattern

Each integrated project should have:

- an objective
- prerequisites
- hardware stack
- software stack
- milestone checklist
- evaluation criteria
- demo deliverables
