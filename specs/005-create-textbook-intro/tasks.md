---
description: "Task list for Create Textbook Introduction"
---

# Tasks: Create Textbook Introduction

**Input**: Design documents from `/specs/005-create-textbook-intro/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md (decisions)

**Tests**: No specific automated tests requested. Validation is manual via `npm run start` and visual inspection.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each aspect of the introduction.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus Project**: `docs/` at repository root
- **Target File**: `docs/intro.md`

---

## Phase 1: Setup (Project Initialization)

> This phase ensures the environment is ready for content replacement.

- [ ] T001 Verify `docs/intro.md` exists and is writable (SC-004)
- [ ] T002 Verify Mermaid support in Docusaurus configuration (SC-002)

---

## Phase 2: Foundational (Content Structure)

**Purpose**: Establish the structure of the introduction file before filling in detailed content.

- [ ] T003 Clean `docs/intro.md`: Remove all existing placeholder text (including "E=mc^2" and dummy diagrams) (SC-004)
- [ ] T004 Create skeletal structure in `docs/intro.md` with headers: "Concept Overview", "Prerequisites", "Course Structure", "Philosophy" (FR-007)

**Checkpoint**: `docs/intro.md` should be clean with empty sections ready for content.

---

## Phase 3: User Story 1 - Understand Course Scope & Definition (Priority: P1)

**Goal**: Define "Physical AI" and the "Brain-Body-Nervous System" analogy.

**Independent Test**: Navigate to "Course Introduction" -> "Concept Overview". Verify definition and Mermaid diagram.

### Implementation for User Story 1

- [ ] T005 [US1] Write "Concept Overview" section in `docs/intro.md`: Define "Physical AI" vs "Generative AI" (FR-001)
- [ ] T006 [US1] Write "System Analogy" section in `docs/intro.md`: Explain Brain (AI) / Nervous System (ROS 2) / Body (Robot) (FR-002)
- [ ] T007 [US1] Implement Mermaid diagram in `docs/intro.md` visualizing the "Brain-Body-Nervous System" architecture (FR-005, SC-002)

---

## Phase 4: User Story 2 - Assess Prerequisites (Priority: P1)

**Goal**: Clearly list hardware and software requirements.

**Independent Test**: Navigate to "Prerequisites". Verify Ubuntu 22.04, ROS 2 Humble, and NVIDIA GPU requirements.

### Implementation for User Story 2

- [ ] T008 [US2] Write "Prerequisites" section in `docs/intro.md` using Admonitions (`:::warning` / `:::info`) (FR-004)
- [ ] T009 [P] [US2] List OS requirements (Ubuntu 22.04/24.04) and Middleware (ROS 2 Humble/Jazzy) in `docs/intro.md` (FR-004)
- [ ] T010 [P] [US2] List Hardware requirements (NVIDIA GPU, RAM) and alternative paths (Sim-only) in `docs/intro.md` (FR-004)

---

## Phase 5: User Story 3 - Navigate the Learning Path (Priority: P1)

**Goal**: Outline the 4-module structure.

**Independent Test**: Navigate to "Course Structure". Verify list of Modules 1-4 with summaries.

### Implementation for User Story 3

- [ ] T011 [US3] Write "Course Structure" section in `docs/intro.md`: List Module 1 (Nervous System), Module 2 (Digital Twin), Module 3 (AI Brain), Module 4 (VLA) (FR-003, SC-003)
- [ ] T012 [P] [US3] Add 1-sentence summary for each module in `docs/intro.md` (FR-003)

---

## Final Phase: Polish & Cross-Cutting Concerns

- [ ] T013 Verify total word count > 500 words (SC-001)
- [ ] T014 Run `npm run start` and visually verify formatting of Mermaid diagrams and Admonitions
- [ ] T015 Run `npm run build` to ensure no build errors exist

---

## Dependencies & Execution Order

- **Phase 1 -> Phase 2 -> Phase 3 -> Phase 4 -> Phase 5**
- All content tasks modify the same file (`docs/intro.md`), so logical sequential editing is preferred to avoid merge conflicts if parallelized, though sections can be appended independently.

## Parallel Execution Examples

- **Within Phase 4 (Prerequisites)**: T009 (Software) and T010 (Hardware) can be written independently.
- **Within Phase 5 (Structure)**: T012 (Summaries) can be drafted while T011 (List) is being structured.

## Implementation Strategy

- **MVP**: Complete User Story 1 (Concept & Definition) first as it sets the tone.
- **Incremental**: Add Prerequisites (US2) and Structure (US3) sequentially.
- **Verification**: Check the render after each User Story completion.
