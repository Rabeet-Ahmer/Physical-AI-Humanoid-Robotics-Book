---
phr_id: 003
title: Generate Tasks Module 2 Digital Twin
stage: tasks
date_iso: 2025-12-10
surface: agent
model: gemini-1.5-pro-a
feature: module-2-digital-twin
branch: 003-module-2-digital-twin
user: user
command: /sp.tasks
labels: ["task-generation", "project-management", "content-creation"]
links:
  spec: specs/003-module-2-digital-twin/spec.md
  plan: specs/003-module-2-digital-twin/plan.md
  tasks: specs/003-module-2-digital-twin/tasks.md
  ticket: null
  adr: null
  pr: null
files_yaml:
  - specs/003-module-2-digital-twin/tasks.md
tests_yaml: []
---
## Prompt Text

```text
1. **Setup**: Run `.specify/scripts/bash/check-prerequisites.sh --json` from repo root and parse FEATURE_DIR and AVAILABLE_DOCS list. All paths must be absolute. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. **Load design documents**: Read from FEATURE_DIR:
   - **Required**: plan.md (tech stack, libraries, structure), spec.md (user stories with priorities)
   - **Optional**: data-model.md (entities), contracts/ (API endpoints), research.md (decisions), quickstart.md (test scenarios)
   - Note: Not all projects have all documents. Generate tasks based on what's available.

3. **Execute task generation workflow**:
   - Load plan.md and extract tech stack, libraries, project structure
   - Load spec.md and extract user stories with their priorities (P1, P2, P3, etc.)
   - If data-model.md exists: Extract entities and map to user stories
   - If contracts/ exists: Map endpoints to user stories
   - If research.md exists: Extract decisions for setup tasks
   - Generate tasks organized by user story (see Task Generation Rules below)
   - Generate dependency graph showing user story completion order
   - Create parallel execution examples per user story
   - Validate task completeness (each user story has all needed tasks, independently testable)

4. **Generate tasks.md**: Use `.specify.specify/templates/tasks-template.md` as structure, fill with:
   - Correct feature name from plan.md
   - Phase 1: Setup tasks (project initialization)
   - Phase 2: Foundational tasks (blocking prerequisites for all user stories)
   - Phase 3+: One phase per user story (in priority order from spec.md)
   - Each phase includes: story goal, independent test criteria, tests (if requested), implementation tasks
   - Final Phase: Polish & cross-cutting concerns
   - All tasks must follow the strict checklist format (see Task Generation Rules below)
   - Clear file paths for each task
   - Dependencies section showing story completion order
   - Parallel execution examples per story
   - Implementation strategy section (MVP first, incremental delivery)

5. **Report**: Output path to generated tasks.md and summary:
   - Total task count
   - Task count per user story
   - Parallel opportunities identified
   - Independent test criteria for each story
   - Suggested MVP scope (typically just User Story 1)
   - Format validation: Confirm ALL tasks follow the checklist format (checkbox, ID, labels, file paths)
```

## Response Text

The tasks for "Module 2: The Digital Twin" have been generated and saved to `specs/003-module-2-digital-twin/tasks.md`.

**Task Summary**:
- **Total Tasks**: 33
- **Phase 1 (Setup)**: 3 tasks
- **Phase 2 (Foundational)**: 2 tasks
- **Phase 3 (User Story 1 - Intro)**: 6 tasks
- **Phase 4 (User Story 2 - Gazebo)**: 6 tasks
- **Phase 5 (User Story 3 - Unity)**: 6 tasks
- **Phase 6 (User Story 4 - Sensors)**: 6 tasks
- **Phase 7 (Polish)**: 4 tasks

**Execution Strategy**:
- **MVP Scope**: Complete Phase 1 & 2 (Setup/Foundation) + Phase 3 (User Story 1: Course Introduction).
- **Parallelization**: After Phase 2 (Foundation), all 4 user stories (Intro, Gazebo, Unity, Sensors) are designed to be implemented in parallel if resources allow, as they touch separate files.
- **Validation**: Each user story has an independent test criteria (reviewing the generated markdown against requirements).

**Format Check**:
- All tasks follow the `[ ] [ID] [P?] [Story] Description` format.
- File paths are included for all implementation tasks.

---
## Outcome
- **Evaluation**: Tasks successfully generated and organized by user story. Ready for implementation.
- **Next Steps**: Begin execution with Phase 1 tasks.
