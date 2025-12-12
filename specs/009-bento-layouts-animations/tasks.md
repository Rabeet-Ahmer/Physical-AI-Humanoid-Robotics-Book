---
description: "Task list for Bento Layouts & Animations for Other Sections"
---

# Tasks: Bento Layouts & Animations for Other Sections

**Input**: Design documents from `/specs/009-bento-layouts-animations/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md (decisions), data-model.md (component props), quickstart.md (verification)

**Tests**: Primarily visual and structural verification. `npm run start`, `npm run build`, and Lighthouse audits.

**Organization**: Tasks are grouped by component and user story to ensure modular development and clear dependencies.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Static Assets**: `static/img/`
- **React Components**: `src/components/`
- **CSS**: `src/css/custom.css`
- **Landing Page**: `src/pages/index.tsx`
- **New global styles**: `src/styles/animations.module.css`

---

## Phase 1: Setup (Dependencies & Base Components)

> This phase installs required libraries and sets up foundational components for bento layout and animations.

- [ ] T001 Install `react-intersection-observer`: `npm install react-intersection-observer` (Research: On-scroll reveals)
- [ ] T002 Create component directory: `src/components/BentoGrid`
- [ ] T003 Create component directory: `src/components/AnimatedContent`
- [ ] T004 Create new stylesheet: `src/styles/animations.module.css` for global animation utilities

**Checkpoint**: All assets are ready, dependencies installed, and new component directories/files created.

---

## Phase 2: Foundational (Bento Layout Components)

> This phase develops the reusable components for the bento grid structure.

- [ ] T005 Implement `src/components/BentoGrid/index.tsx` for `BentoGridContainer` and `BentoGridItem` (FR-001)
- [ ] T006 [P] Style `src/components/BentoGrid/styles.module.css` for responsive CSS Grid layout, default item styling, and basic bento sizing (FR-001, FR-006, SC-001)

**Checkpoint**: Reusable bento grid components are functional.

---

## Phase 3: User Story 1 - Engaging Content Presentation (Priority: P1)

**Goal**: Redesign `HomepageFeatures` and `HomepageCurriculum` with bento layouts.

**Independent Test**: Visually inspect the homepage on desktop for bento grid structures in both sections.

### Implementation for User Story 1

- [ ] T007 [US1] Rework `src/components/HomepageFeatures/index.tsx` to use `BentoGridContainer` and `BentoGridItem` (FR-001, FR-002)
- [ ] T008 [P] [US1] Update `src/components/HomepageFeatures/styles.module.css` for specific feature bento layout variations and aesthetic (FR-005, SC-001)
- [ ] T009 [US1] Rework `src/components/HomepageCurriculum/index.tsx` to use `BentoGridContainer` and `BentoGridItem` (FR-001, FR-003)
- [ ] T010 [P] [US1] Update `src/components/HomepageCurriculum/styles.module.css` for specific curriculum bento layout variations and aesthetic (FR-005, SC-001)
- [ ] T011 [US1] Update `src/pages/index.tsx` to ensure proper integration of reworked `HomepageFeatures` and `HomepageCurriculum` (FR-001)

---

## Phase 4: User Story 2 - Enhanced User Engagement with Unique Animations (Priority: P1)

**Goal**: Implement non-generic animations (on-scroll, hover) for bento grid items.

**Independent Test**: Scroll and hover over bento items to verify animation triggers and smoothness.

### Implementation for User Story 2

- [ ] T012 [US2] Implement `src/components/AnimatedContent/index.tsx` using `react-intersection-observer` for on-scroll reveal animations (FR-004, SC-002)
- [ ] T013 [P] [US2] Define unique animation keyframes and classes in `src/styles/animations.module.css` (e.g., subtle parallax, slide-in effects) (FR-004)
- [ ] T014 [US2] Integrate `AnimatedContent` into `HomepageFeatures` items, applying on-scroll animations (FR-004)
- [ ] T015 [US2] Integrate `AnimatedContent` into `HomepageCurriculum` items, applying on-scroll animations (FR-004)
- [ ] T016 [P] [US2] Add advanced hover effects (e.g., 3D tilt, subtle glow) to `BentoGridItem` in `src/components/BentoGrid/styles.module.css` (FR-004, SC-002)

---

## Phase 5: User Story 3 - Seamless Responsive Experience (Priority: P1)

**Goal**: Ensure bento layouts and animations are fully responsive across devices.

**Independent Test**: Resize browser to mobile/tablet sizes. Verify layout adaptation and animation performance.

### Implementation for User Story 3

- [ ] T017 [US3] Refine responsive media queries in `src/components/BentoGrid/styles.module.css` for mobile and tablet adaptations (FR-006, SC-003)
- [ ] T018 [US3] Optimize animation performance for mobile devices in `src/styles/animations.module.css` (FR-007)

---

## Final Phase: Polish & Cross-Cutting Concerns

- [ ] T019 Run `npm install react-intersection-observer`
- [ ] T020 Run `npm run start` and visually verify all bento layouts, animations, and responsiveness (SC-001, SC-002, SC-003, SC-005)
- [ ] T021 Run `npm run build` and check Lighthouse Performance/Accessibility scores (>90%) (SC-004, SC-005)

---

## Dependencies & Execution Order

- **Phase 1 -> Phase 2 -> Phase 3 -> Phase 4 -> Phase 5 -> Final Phase**
- Within phases, parallel tasks are indicated with `[P]`.

## Parallel Execution Examples

- T005 (BentoGrid index.tsx) and T006 (BentoGrid styles) can be done in parallel.
- T007-T008 (HomepageFeatures rework) and T009-T010 (HomepageCurriculum rework) can be parallelized after BentoGrid components are ready.

## Implementation Strategy

- **MVP**: Focus on establishing the core bento grid layout for both sections (Phase 2 & 3).
- **Incremental**: Gradually add animations (Phase 4), ensuring performance at each step.
- **Polish**: Final responsive adjustments and performance audits (Phase 5 & Final Phase).
