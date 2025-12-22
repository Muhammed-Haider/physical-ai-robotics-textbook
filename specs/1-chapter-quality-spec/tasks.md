# Tasks: Core Chapter Quality Specification

**Feature**: `1-chapter-quality-spec`
**Branch**: `1-chapter-quality-spec`
**Date**: 2025-12-22
**Spec**: [spec.md](./spec.md)
**Plan**: [plan.md](./plan.md)

This document outlines the tasks required to finalize the Core Chapter Quality Specification.

---

## Phase 1: Foundational Tasks

- [x] T001 Locate and document the paths to the 8 pedagogical agent prompt files in `.claude/subagents/` in `specs/1-chapter-quality-spec/research.md`.

---

## Phase 2: User Story 1 (Validate Chapter Quality)

**Goal**: As an Author, I want my chapter manuscript to be automatically validated against the 8 core pedagogical principles, so that I can ensure it meets the project's non-negotiable quality standards before it can be integrated into the textbook.

**Independent Test**: A chapter can be submitted to the validation process and receive a pass/fail result with detailed feedback.

### Implementation Tasks

- [x] T002 [US1] Formalize the `Chapter` and `Validation Report` data models from `data-model.md` into JSON Schema files in `specs/1-chapter-quality-spec/schemas/`.
- [x] T003 [P] [US1] Create a sample valid chapter manuscript file (`sample-chapter-valid.md`) for testing purposes in `specs/1-chapter-quality-spec/samples/`.
- [x] T004 [P] [US1] Create a sample invalid chapter manuscript file (`sample-chapter-invalid.md`) that intentionally violates at least 3 of the 8 principles in `specs/1-chapter-quality-spec/samples/`.
- [x] T005 [US1] Manually create an expected `Validation Report` in JSON format for the invalid sample chapter in `specs/1-chapter-quality-spec/samples/expected-report.json`.

---

## Phase 3: Polish

- [x] T006 Review the completed specification, data schemas, and samples for clarity, consistency, and completeness in the `specs/1-chapter-quality-spec/` directory.

---

## Dependencies

- User Story 1 (Phase 2) depends on the completion of the Foundational Task (Phase 1).

## Parallel Execution

- Tasks T003 and T004 can be worked on in parallel.

## Implementation Strategy

The goal is to produce a complete and testable specification. The MVP is the set of formalized schemas and sample files that can be used to guide the development and testing of the validation agents and the validation workflow.
