# Data Model & Content Structure

## Content Architecture

The "database" for this application is the file system (Markdown/MDX files).

### Directory Structure

```text
docs/
├── intro.md                    # Course Introduction
├── hardware-requirements.md    # FR-004
├── weekly-breakdown.md         # FR-003
├── module-01-nervous-system/   # FR-002
│   ├── 01-overview.md
│   ├── 02-ros2-nodes.md
│   └── ...
├── module-02-digital-twin/
│   └── ...
├── module-03-ai-brain/
│   └── ...
└── module-04-vla/
    └── ...
```

## Frontmatter Schema

Every MDX file MUST include:

```yaml
---
id: [unique-slug]
title: [Display Title]
sidebar_label: [Menu Label]
description: [SEO/Meta description]
sidebar_position: [Integer for ordering]
---
```

## Entities

*   **Module**: Defined by a directory containing an `_category_.json` or `index.md` (or simply grouping via sidebars).
*   **Lesson/Chapter**: Individual `.md` or `.mdx` file.
*   **Asset**: Images in `static/img/` referenced via relative paths.
