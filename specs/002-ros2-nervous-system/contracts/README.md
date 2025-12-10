# Content Contracts

Since this is a static site, the "API" is the Frontmatter schema and file naming convention.

## Schema Validation

All content files in `docs/module-01-nervous-system/` MUST pass the following checks:

1.  **ID Uniqueness**: `id` must be unique across the documentation set.
2.  **Title Presence**: `title` must be non-empty.
3.  **Position**: `sidebar_position` must be an integer between 1 and 20.
4.  **Tags**: `tags` must be a list of strings.

## Interactive Component Contracts

### Mermaid Diagrams
-   Must utilize `@docusaurus/theme-mermaid`.
-   Must be wrapped in ` ```mermaid ` blocks.

### Code Snippets
-   Must specify language: `python`, `xml`, `bash`.
-   Must use `title="..."` for file context.
