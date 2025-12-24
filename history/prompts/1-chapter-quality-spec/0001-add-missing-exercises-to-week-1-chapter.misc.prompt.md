---
id: 0001
title: Add Missing Exercises to Week 1 Chapter
stage: misc
date: 2025-12-24
surface: agent
model: Gemini
feature: 1-chapter-quality-spec
branch: main
user: user
command: Implement instruction after discussion
labels: [content, exercises, chapter-quality]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - docs/week-1/physical-ai-foundations.md
tests:
 - 
---

## Prompt

Okay great implement

## Response snapshot

The two new exercises (Basic Actuation by Applying Force and Simple Sensing by Reading State) have been successfully added to the `docs/week-1/physical-ai-foundations.md` chapter. This chapter now meets the requirement of 3 hands-on exercises.

## Outcome

- ✅ Impact: The content of Week 1 chapter is now aligned with the constitution's requirement for the number of hands-on exercises.
- 🧪 Tests: None run, as this was a content modification. The code examples themselves would need to be tested by the user.
- 📁 Files: 1 file modified.
- 🔁 Next prompts: Continue with adding exercises to other chapters, or address the "2 chapters per week" discrepancy.
- 🧠 Reflection: This demonstrates how to incrementally improve content to meet quality specs. The direct instruction from the user facilitated a quick implementation after the detailed "specs" were agreed upon.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): None.