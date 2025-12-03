---
id: 002
title: View Project Constitution
stage: constitution
date: 2025-12-03
surface: agent
model: gemini-pro
feature: none
branch: master
user: '[USER]'
command: /sp.constitution what is in the constitution
labels: [constitution, view]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - None
---

## Prompt

/sp.constitution what is in the constitution

## Response snapshot

Displayed the project constitution from `.specify/memory/constitution.md`.

## Outcome

- ✅ Impact: Low. This was a read-only operation.
- 🧪 Tests: N/A
- 📁 Files: 0 files modified
- 🔁 Next prompts: The user can now ask further questions about the constitution.
- 🧠 Reflection: The process of creating a PHR is a bit manual. It would be better if the script handled more of the file creation and content filling.
