!# Implementation Tasks: Physical AI Textbook

**Feature**: Physical AI Textbook (Docusaurus)
**Branch**: `1-physical-ai-textbook`
**Spec**: [spec.md](spec.md)
**Plan**: [plan.md](plan.md)

## Implementation Strategy
- **Approach**: Static Site Generation (SSG) using Docusaurus v3.
- **Phasing**:
    -   **Phase 1 (Setup)**: Initialize framework and plugins (Math, Mermaid).
    -   **Phase 2 (Foundation)**: Create file hierarchy and navigation structure.
    -   **Phase 3 (US1)**: Implement core navigation and module content.
    -   **Phase 4 (US2)**: Implement hardware documentation.
    -   **Phase 5 (US3)**: Finalize landing page and curriculum verification.

## Dependencies
- US1 (Navigation) depends on Phase 2 (Foundation).
- US2 (Hardware) depends on Phase 2 (Foundation).
- US3 (Landing) is effectively independent but polishes the entry point.

## Phase 1: Setup & Infrastructure
**Goal**: Initialize Docusaurus project with required plugins and configuration.

- [x] T001 Initialize Docusaurus project (classic template) in project root
- [x] T002 [P] Install Docusaurus dependencies (`npm install`)
- [x] T003 [P] Install diagramming plugin `@docusaurus/theme-mermaid`
- [x] T004 [P] Install math plugins `remark-math` and `rehype-katex`
- [x] T005 Configure `docusaurus.config.js` to enable Mermaid theme and markdown support
- [x] T006 Configure `docusaurus.config.js` to add remark-math and rehype-katex to presets
- [x] T007 Add KaTeX CSS import to `src/css/custom.css`
- [x] T008 [P] Update `docusaurus.config.js` with site metadata (title: "Physical AI & Humanoid Robotics", taglines)
- [x] T009 Clean up default Docusaurus scaffold content (remove `docs/tutorial-basics`, `blog/` if unused)

## Phase 2: Foundation
**Goal**: Establish the directory structure defined in data-model.md.

- [x] T010 Create `docs/module-01-nervous-system` directory
- [x] T011 [P] Create `docs/module-02-digital-twin` directory
- [x] T012 [P] Create `docs/module-03-ai-brain` directory
- [x] T013 [P] Create `docs/module-04-vla` directory
- [x] T014 Create placeholder `docs/intro.md` for Course Introduction
- [x] T015 Create placeholder `docs/weekly-breakdown.md`
- [x] T016 Create placeholder `docs/hardware-requirements.md`

## Phase 3: Student Course Navigation (US1)
**Goal**: Implement the 4 modules and weekly breakdown navigation.
**Independent Test**: Verify sidebars show all 4 modules and Weekly Breakdown is accessible.

- [x] T017 [US1] Create `docs/module-01-nervous-system/_category_.json` for sidebar grouping
- [x] T018 [US1] Create `docs/module-01-nervous-system/01-overview.md` with Module 1 focus/topics
- [x] T019 [US1] [P] Create `docs/module-02-digital-twin/_category_.json` for sidebar grouping
- [x] T020 [US1] [P] Create `docs/module-02-digital-twin/01-overview.md` with Module 2 focus/topics
- [x] T021 [US1] [P] Create `docs/module-03-ai-brain/_category_.json` for sidebar grouping
- [x] T022 [US1] [P] Create `docs/module-03-ai-brain/01-overview.md` with Module 3 focus/topics
- [x] T023 [US1] [P] Create `docs/module-04-vla/_category_.json` for sidebar grouping
- [x] T024 [US1] [P] Create `docs/module-04-vla/01-overview.md` with Module 4 focus/topics
- [x] T025 [US1] Implement full content for `docs/weekly-breakdown.md` covering Weeks 1-13
- [x] T026 [US1] Configure `sidebars.js` to autogenerate or strictly define hierarchy

## Phase 4: Hardware Setup Reference (US2)
**Goal**: Detailed guide on required hardware.
**Independent Test**: Verify `Hardware Requirements` page exists and lists RTX 4070 Ti, Jetson Orin, etc.

- [x] T027 [US2] Implement `docs/hardware-requirements.md` with "Digital Twin Workstation" specs (RTX 4070 Ti, etc.)
- [x] T028 [US2] Add "Physical AI Edge Kit" section to `docs/hardware-requirements.md` (Jetson Orin, RealSense)
- [x] T029 [US2] Add "Robot Lab" options (Proxy, Miniature, Premium) to `docs/hardware-requirements.md`

## Phase 5: Instructor Curriculum Verification (US3)
**Goal**: Landing page with clear goals and learning outcomes.
**Independent Test**: Landing page shows "Goal" and "Why Physical AI Matters".

- [x] T030 [US3] Update `src/pages/index.js` Header with Course Title and Tagline
- [x] T031 [US3] Implement "Focus and Theme" section in `src/pages/index.js` (or a dedicated component)
- [x] T032 [US3] [P] Implement "Goal" and "Quarter Overview" sections in `src/pages/index.js`
- [x] T033 [US3] [P] Implement "Why Physical AI Matters" section in `src/pages/index.js`
- [x] T034 [US3] [P] Implement "Learning Outcomes" section in `src/pages/index.js`
- [x] T035 [US3] Update `docusaurus.config.js` navbar to link to Intro, Weekly Breakdown, and Hardware

## Phase 6: Polish
**Goal**: Final build verification and cleanup.

- [x] T036 Run `npm run build` to verify static generation success
- [x] T037 Check for broken links using `npm run build` output or checking tool
- [x] T038 Verify Mermaid diagrams render correctly in Module overviews (add a test diagram if needed)
- [x] T039 Verify Math equations render correctly (add a test equation to Intro)

## Parallel Execution Examples
- **Setup**: T002, T003, T004 can run in parallel.
- **Content**: T018, T020, T022, T024 (Module content) can be written in parallel by different authors.
- **Landing Page**: Components (T031-T034) can be built as separate React components in parallel.
