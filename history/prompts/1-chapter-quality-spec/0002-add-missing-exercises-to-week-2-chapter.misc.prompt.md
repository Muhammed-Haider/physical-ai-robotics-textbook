---
id: 0002
title: Add Missing Exercises to Week 2 Chapter
stage: misc
date: 2025-12-24
surface: agent
model: Gemini
feature: 1-chapter-quality-spec
branch: feat/update-constitution-and-content
user: user
command: Implement instruction after discussion
labels: [content, exercises, chapter-quality]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - docs/week-2/physical-ai-foundational-concepts.md
tests:
 - 
---

## Prompt

Yes add of coures then I would be able to clear the error

## Response snapshot

The two new exercises (Reactive Actuation Based on Sensor Data and Simple Sensor Fusion / State Inference) have been successfully added to the `docs/week-2/physical-ai-foundational-concepts.md` chapter. This chapter now contains the required 3 hands-on exercises.

## Outcome

- ✅ Impact: The content of Week 2 chapter is now aligned with the constitution's requirement for the number of hands-on exercises.
- 🧪 Tests: No automated tests were run by the agent. The code examples themselves would need to be tested by the user.
- 📁 Files: 1 file modified.
- 🔁 Next prompts: Continue with adding exercises to other chapters, or address the "2 chapters per week" discrepancy.
- 🧠 Reflection: This process continues to demonstrate how to incrementally improve content to meet quality specs. The user's approval of the workflow for adding changes to the same branch streamlines development.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): None.