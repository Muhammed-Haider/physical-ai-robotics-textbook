---
id: 002
title: Optimize README with Dynamic GitHub Stats
stage: general
date: 2025-12-22
surface: agent
model: gemini-1.5-flash
feature: none
branch: main
user: user
command: (User request to optimize README)
labels: ["readme", "documentation", "automation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - README.md
  - scripts/update_readme.py
  - scripts/requirements.txt
tests: null
---

## Prompt

there is also a problem the readme file for the project is not optimized on github

## Response snapshot

I have restructured the README.md to include badges and a section for dynamic project statistics. I also created a Python script in 'scripts/update_readme.py' to automatically fetch these stats from GitHub and update the README. The script currently shows 'N/A' because the default repository 'Panaversity/ai-book' was not found. To make it work, you can either set the GITHUB_REPOSITORY environment variable to your repository's '<owner>/<name>' or manually edit the script. Instructions are included in the updated README.

## Outcome

- ✅ Impact: Improved the project's `README.md` by adding a professional structure, badges, and a mechanism for displaying dynamic project statistics from GitHub. Provided an automation script to keep the stats up-to-date.
- 🧪 Tests: N/A
- 📁 Files: Updated `README.md` and created `scripts/update_readme.py` and `scripts/requirements.txt`.
- 🔁 Next prompts: User can configure the `update_readme.py` script and run it, or proceed with other tasks.
- 🧠 Reflection: The user's request was general, but the provided code snippets hinted at the desire for dynamic GitHub stats. The solution provided a robust way to achieve this. The `create-phr.sh` script continues to be unreliable.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-phr.sh` script did not create the PHR file.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
