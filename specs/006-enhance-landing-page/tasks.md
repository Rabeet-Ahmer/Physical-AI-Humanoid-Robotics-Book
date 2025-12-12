---
description: "Task list for Enhance Landing Page"
---

# Tasks: Enhance Landing Page

**Input**: Design documents from `/specs/006-enhance-landing-page/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md (decisions)

**Tests**: Validation is primarily visual and structural. `npm run start` and `npm run build` are key verification steps.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each aspect of the landing page.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus Project**: `src/` at repository root
- **CSS**: `src/css/custom.css`
- **Pages**: `src/pages/`
- **Components**: `src/components/`

---

## Phase 1: Setup (Shared Infrastructure)

> This phase prepares the styling and component structure.

- [ ] T001 Import 'Merriweather' font and configure CSS variables for "Academic & Clean" style in `src/css/custom.css` (FR-004)
- [ ] T002 Create directory structure for new components: `src/components/HomepageHeader` and `src/components/HomepageCurriculum`

**Checkpoint**: Fonts are loaded, directories exist.

---

## Phase 2: User Story 1 - Grasp Value Proposition Immediately (Priority: P1)

**Goal**: Create a compelling Hero section with a clear value proposition.

**Independent Test**: Visit homepage. Verify Title, Subtitle, CTA, and visual style of the Hero section.

### Implementation for User Story 1

- [ ] T003 [US1] Create `HomepageHeader` component in `src/components/HomepageHeader/index.tsx` implementing the Hero layout (FR-001)
- [ ] T004 [US1] Style `HomepageHeader` with academic typography and layout in `src/components/HomepageHeader/styles.module.css` (FR-004)
- [ ] T005 [US1] Update `src/pages/index.tsx` to use the new `HomepageHeader` component instead of the default header (FR-001)

---

## Phase 3: User Story 2 - Explore Key Features & Curriculum (Priority: P1)

**Goal**: Display detailed breakdown of features and modules.

**Independent Test**: Scroll down. Verify Features Grid (3 columns) and Curriculum list (4 modules) are present and linked.

### Implementation for User Story 2

- [ ] T006 [US2] Update `src/components/HomepageFeatures/index.tsx` to display the specific 3-column grid (ROS 2, Digital Twin, VLA) with appropriate icons/text (FR-002)
- [ ] T007 [P] [US2] Create `HomepageCurriculum` component in `src/components/HomepageCurriculum/index.tsx` to list the 4 modules (FR-003)
- [ ] T008 [P] [US2] Style `HomepageCurriculum` to match the academic theme in `src/components/HomepageCurriculum/styles.module.css` (FR-004)
- [ ] T009 [US2] Integrate `HomepageCurriculum` into `src/pages/index.tsx` below the features section (FR-003)

---

## Phase 4: User Story 3 - Visual Consistency (Priority: P2)

**Goal**: Ensure the overall aesthetic matches the "Academic & Clean" requirement.

**Independent Test**: Visual inspection. Verify serif headings, whitespace, and color palette.

### Implementation for User Story 3

- [ ] T010 [US3] Refine global typography in `src/css/custom.css` to ensure all H1-H6 use Merriweather (FR-004)
- [ ] T011 [US3] Adjust global spacing and colors in `src/css/custom.css` to align with the "Clean" aesthetic (white background, dark gray text) (FR-004)
- [ ] T012 [P] [US3] Add animations/transitions to the Hero elements (FR-004)

---

## Final Phase: Polish & Cross-Cutting Concerns

- [ ] T013 Verify mobile responsiveness of Hero, Features, and Curriculum sections (FR-005)
- [ ] T014 Run `npm run build` to ensure no build errors exist
- [ ] T015 Perform accessibility check (Lighthouse) aiming for score > 90 (SC-002)

---

## Dependencies & Execution Order

- **Phase 1 -> Phase 2 -> Phase 3 -> Phase 4**
- **Phase 2 (Hero)** and **Phase 3 (Features/Curriculum)** can be developed in parallel by different developers if needed, merging into `index.tsx` at the end.

## Parallel Execution Examples

- T007 (Curriculum Component) and T006 (Features Update) modify different files and can run concurrently.
- T012 (Animations) can be done anytime after the components exist.

## Implementation Strategy

- **MVP**: Complete Phase 1 and 2 to get the new visual identity and Hero up.
- **Incremental**: Add the detailed Features and Curriculum sections next.
- **Polish**: Final visual tweaks and animations.
