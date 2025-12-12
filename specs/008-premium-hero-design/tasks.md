---
description: "Task list for Premium Hero Section Design"
---

# Tasks: Premium Hero Section Design

**Input**: Design documents from `/specs/008-premium-hero-design/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md (decisions), data-model.md (component props), quickstart.md (verification)

**Tests**: Primarily visual and structural verification. `npm run start`, `npm run build`, and Lighthouse audits.

**Organization**: Tasks are grouped by user story and component to ensure modular development.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Static Assets**: `static/img/`
- **React Components**: `src/components/`
- **CSS**: `src/css/custom.css`
- **Landing Page**: `src/pages/index.tsx`

---

## Phase 1: Setup (Assets & Dependencies)

> This phase prepares external assets and installs required libraries.

- [ ] T001 Download hero image from `https://stockcake.com/i/futuristic-robot-portrait_1029060_1100434` and save as `static/img/hero-book-cover.jpg` (FR-010)
- [ ] T002 Install `react-tsparticles` and `tsparticles`: `npm install react-tsparticles tsparticles` (Research: Animated Background)
- [ ] T003 Create component directory: `src/components/BackgroundAnimation`
- [ ] T004 Create component directory: `src/components/FeatureBadge`
- [ ] T005 Create component directory: `src/components/CTAButton`

**Checkpoint**: All assets are ready, dependencies installed, and new component directories created.

---

## Phase 2: User Story 1 - Engaging First Impression (Priority: P1)

**Goal**: Implement the two-column hero layout with image, title, subtitle, badge, feature badges, and CTA buttons, including background animation.

**Independent Test**: Visit homepage. Verify all visual and functional elements of the hero section on desktop.

### Implementation for User Story 1

- [ ] T006 [US1] Implement `src/components/BackgroundAnimation/index.tsx` using `react-tsparticles` for glowing particles/light bloom (FR-008, SC-007)
- [ ] T007 [P] [US1] Style `src/components/BackgroundAnimation/styles.module.css` for background effects and positioning (FR-008)
- [ ] T008 [US1] Rework `src/components/HomepageHeader/index.tsx` to implement the two-column layout (Left: image, Right: content) (FR-001)
- [ ] T009 [P] [US1] Update `src/components/HomepageHeader/styles.module.css` for the two-column layout, responsiveness, and premium aesthetic (FR-001, FR-009)
- [ ] T010 [US1] Add `static/img/hero-book-cover.jpg` to the left column of `HomepageHeader` with rounded corners and subtle shadow (FR-002, SC-006)
- [ ] T011 [P] [US1] Add "PHYSICAL AI BOOK" badge to `HomepageHeader` right column (FR-003)
- [ ] T012 [P] [US1] Add "AI Native Robotics" main title to `HomepageHeader` right column (FR-004)
- [ ] T013 [P] [US1] Add "Learn Physical AI & Humanoid Robotics the modern way" subtitle to `HomepageHeader` right column (FR-005)
- [ ] T014 [P] [US1] Create `src/components/FeatureBadge/index.tsx` to implement the rounded pill badge styling (FR-006)
- [ ] T015 [P] [US1] Style `src/components/FeatureBadge/styles.module.css` for dark gray background, soft shadow, and light inner border (FR-006)
- [ ] T016 [US1] Integrate three `FeatureBadge` components into `HomepageHeader` with specified text and emojis (✨, 🤝, 🎯) (FR-006)
- [ ] T017 [P] [US1] Create `src/components/CTAButton/index.tsx` for primary and secondary CTAs (FR-007)
- [ ] T018 [P] [US1] Style `src/components/CTAButton/styles.module.css` for white/dark glass buttons with correct padding, rounded corners, and icons (FR-007)
- [ ] T019 [US1] Integrate two `CTAButton` components ("Start Reading →", "Explore Project 🎓") into `HomepageHeader` (FR-007)
- [ ] T020 [US1] Update `src/pages/index.tsx` to use the reworked `HomepageHeader` and `BackgroundAnimation` components (FR-001)

---

## Phase 3: User Story 3 - Responsive and Modern Experience (Priority: P1)

**Goal**: Ensure the Hero section adapts gracefully to different screen sizes and maintains a premium, futuristic aesthetic.

**Independent Test**: Resize browser from desktop to mobile widths. Verify stacking, centering, and overall look.

### Implementation for User Story 3

- [ ] T021 [US3] Implement responsive adjustments in `src/components/HomepageHeader/styles.module.css` for mobile stacked layout (FR-001)
- [ ] T022 [US3] Refine glass-morphism effects in `src/components/FeatureBadge/styles.module.css` and `src/components/CTAButton/styles.module.css` to ensure visual quality (FR-009)
- [ ] T023 [US3] Ensure clean, high-contrast white headings in `src/components/HomepageHeader/styles.module.css` for both light and dark themes (FR-008)

---

## Final Phase: Polish & Cross-Cutting Concerns

- [ ] T024 Perform `npm install react-tsparticles tsparticles` to make sure all dependencies for the hero section are installed
- [ ] T025 Run `npm run start` and visually verify the complete hero section on multiple screen sizes and in both light/dark modes (SC-001, SC-002, SC-003, SC-004, SC-006, SC-007)
- [ ] T026 Run `npm run build` to ensure no build errors and check Lighthouse Performance/Accessibility scores (SC-005)

---

## Dependencies & Execution Order

- **Phase 1 -> Phase 2 -> Phase 3 -> Final Phase**
- Within Phase 2, tasks for different sub-components (`FeatureBadge`, `CTAButton`) can be parallelized after `HomepageHeader` structure is established.

## Parallel Execution Examples

- T014-T015 (FeatureBadge component) can be done in parallel with T017-T018 (CTAButton component).
- T006-T007 (BackgroundAnimation component) can be developed independently of T008-T009 (HomepageHeader rework).

## Implementation Strategy

- **MVP**: Focus on getting the basic layout (FR-001), core content (FR-003, FR-004, FR-005), and CTAs (FR-007) implemented first.
- **Incremental**: Add image (FR-002), feature badges (FR-006), and then atmospheric effects (FR-008) and overall mood refinements (FR-009).
- **Verification**: Utilize `quickstart.md` checklist at each major iteration.
