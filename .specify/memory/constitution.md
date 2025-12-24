<!-- Sync Impact Report -->
<!-- Version change: 2.0.0 -> 2.0.1 (Patch) -->
<!-- Description: Corrected the FILE ORGANIZATION section to reflect the actual project structure, replacing references to '.claude' and 'CLAUDE.md' with the correct '.gemini' directory and 'GEMINI.md' file. -->
<!-- Templates requiring updates: -->
<!--   - .specify/templates/plan-template.md: ⚠️ pending -->
<!-- Follow-up TODOs: None. -->
📜 PHYSICAL AI TEXTBOOK - LEAN CONSTITUTION v2.0.1

**Effective Date:** December 24, 2025  
**Goal:** Complete 13-week curriculum TODAY using Gemini CLI + SpecKit

---

## 🎯 IDENTITY & PURPOSE

**Project:** Physical AI & Humanoid Robotics: An AI-Native Textbook  
**Mission:** Create a world-class digital textbook teaching Physical AI and Humanoid Robotics through:
- Pedagogically sound content
- Beautiful Apple/Vercel-level design
- Interactive RAG chatbot
- Personalized learning paths

**Educational Foundation:** Piaget, Vygotsky, Papert, Dewey, Montessori, Csíkszentmihályi, Minsky, Bruner

---

## 📦 SCOPE

### ✅ IN SCOPE
**13-Week Curriculum:**
- Weeks 1-2: Physical AI Foundations
- Weeks 3-5: ROS 2 Fundamentals  
- Weeks 6-7: Gazebo & Unity Simulation
- Weeks 8-10: NVIDIA Isaac Platform
- Weeks 11-12: Humanoid Robot Development
- Week 13: Conversational Robotics

**Technical Coverage:**
- ROS 2 (Humble/Iron), Gazebo, NVIDIA Isaac Sim, Unity
- Python implementations, VLA models, URDF/SDF
- SLAM, navigation, manipulation

**Learning Resources:**
- Theory + analogies
- Minimum 3 hands-on exercises per chapter
- Interactive simulations
- Real-world case studies
- Self-check assessments

**Interactive Features:**
- RAG chatbot (Q&A)
- User authentication
- Progress tracking
- Urdu translation toggle
- Bookmarking/notes

### ❌ OUT OF SCOPE
- Hardware assembly instructions
- Non-Python languages (C++ mentioned only)
- Advanced math beyond prerequisites
- Non-humanoid robotics
- RTOS/embedded programming

**Audience:** 18-35 year olds with Python + linear algebra basics

**Tech Stack:**
- Platform: Docusaurus 3.x
- Deployment: GitHub Pages
- Database: Neon Serverless Postgres
- Vector Store: Qdrant Cloud
- AI: OpenAI API (GPT-4, Whisper)
- Auth: Better Auth

---

## ⚡ WORKFLOW (Ship Fast)

### 1. **Content Creation**
```
Write chapter → Test code → Deploy → Fix bugs
```

**Per Chapter:**
1. Create specification (5 learning outcomes, 3+ exercises, prerequisites)
2. Generate content using AI agents
3. Test all code examples locally
4. Add to Docusaurus
5. Build and deploy

**Timeline:** 1-2 hours per chapter max

### 2. **Validation (Helpers, Not Gatekeepers)**

Use these 8 AI agents to **improve** content, not block it:
- Developmental Staging Agent
- Constructivist Activity Agent
- Motivational Immersion Agent
- Creative Synthesis Agent
- Modular Mind Agent
- Contextual Humanity Agent
- Technology Critique Agent
- Reflective Assessment Agent

**Rule:** Apply feedback if useful, ship anyway if needed.  
**Target:** 7.0/10 score ideal, but 6.0+ acceptable to ship fast

### 3. **Design Validation**
- Visual Design Architect (Apple-level aesthetics)
- Typography Agent (Vercel-level readability)
- Interactive Experience Agent (smooth interactions)

**Rule:** Good enough > perfect. Ship and iterate.

### 4. **Testing**
**Required:**
- Code examples run without errors
- Links work (no 404s)
- Site loads on mobile + desktop
- Basic accessibility (can navigate with keyboard)

**Nice to Have (post-launch):**
- Lighthouse score >90
- WCAG AA compliance
- Performance optimization

---

## 🎨 QUALITY STANDARDS

### Minimum Viable Quality (MVP)
✅ Content is technically accurate  
✅ Code examples execute successfully  
✅ Site is navigable and readable  
✅ RAG chatbot responds to queries  
✅ No broken links  

### Aspirational Quality (Post-Launch)
- Pedagogical validation score ≥7.5/10
- Visual Design score ≥8.0/10
- Lighthouse Performance ≥90
- 4.5/5 star learner feedback

**Ship now, polish later.**

---

## 🏗️ FILE ORGANIZATION

```
physical-ai-book/
├── .gemini/
│   ├── agents/              # Agent definitions
│   ├── commands/            # Command definitions
│   └── skills/              # Skill definitions
├── docs/
│   ├── intro.md
│   ├── week-01/ ... week-13/
│   └── resources/
├── src/
│   ├── components/          # React components
│   ├── css/
│   │   └── custom.css       # Design system
│   └── pages/
├── static/
│   ├── img/
│   └── files/
├── GEMINI.md                # Gemini CLI context
├── CONSTITUTION.md          # This file
├── docusaurus.config.js
└── sidebars.js
```

**Naming:**
- Chapters: `week-XX-topic.md`
- Images: `week-XX-description.png`
- Variables: camelCase (JS), kebab-case (CSS)

---

## 👤 DECISION MAKING

**Decision Maker:** You (or single designated lead)

**No committees. No approvals. No bureaucracy.**

If stuck: consult AI agents → make decision → ship.

---

## 🔒 ETHICS & LICENSING

### Core Principles
- Accessible to all (WCAG basic compliance)
- Free and open access
- All sources cited
- Safety warnings included
- Inclusive, gender-neutral language

### Data & Privacy
- Minimal data collection
- Transparent privacy policy
- User control over data
- No selling user info
- Secure storage

### Licensing
- **Content:** CC BY-NC-SA 4.0
- **Code:** MIT License
- **Assets:** Individual attribution

---

## 🧪 TEST DRIVEN DEVELOPMENT (TDD)

**All code follows TDD:**
1. Write tests first
2. Tests fail (red)
3. Write code to pass (green)
4. Refactor while keeping tests green

**For book code examples:**
- Test locally before adding to chapter
- Include test files in repo
- Document how to run tests

---

## 🚀 TODAY'S CHECKLIST

### Phase 1: Setup (30 min)
- [ ] Initialize Docusaurus project
- [ ] Set up design system (colors, typography)
- [ ] Configure Gemini CLI + SpecKit
- [ ] Prepare AI agent prompts

### Phase 2: Content Sprint (8-10 hours)
- [ ] Week 1: Physical AI Foundations (1 chapter)
- [ ] Week 2: Physical AI Foundations cont. (1 chapter)
- [ ] Week 3: ROS 2 Basics (1 chapter)
- [ ] Week 4: ROS 2 Intermediate (1 chapter)
- [ ] Week 5: ROS 2 Advanced (1 chapter)
- [ ] Week 6: Gazebo Simulation (1 chapter)
- [ ] Week 7: Unity Integration (1 chapter)
- [ ] Week 8: Isaac Sim Basics (1 chapter)
- [ ] Week 9: Isaac ROS (1 chapter)
- [ ] Week 10: Isaac Advanced (1 chapter)
- [ ] Week 11: Humanoid Design (1 chapter)
- [ ] Week 12: Humanoid Control (1 chapter)
- [ ] Week 13: Conversational Robotics (1 chapter)

**Per chapter:** Spec (10min) → Generate (30min) → Test (15min) → Deploy (5min)

### Phase 3: Integration (1-2 hours)
- [ ] Set up RAG chatbot
- [ ] Configure authentication
- [ ] Add navigation/sidebars
- [ ] Test all links

### Phase 4: Deploy (30 min)
- [ ] Build site
- [ ] Deploy to GitHub Pages
- [ ] Verify production site
- [ ] Monitor for errors

---

## 📊 SUCCESS METRICS

**Today's Definition of Done:**
- ✅ All 13 weeks have content (26+ chapters)
- ✅ All code examples tested
- ✅ Site builds without errors
- ✅ RAG chatbot functional
- ✅ Deployed and accessible

**Post-Launch (Next Week):**
- Gather feedback from 10+ beta testers
- Fix critical bugs
- Improve based on usage

**Long-term (Later):**
- 1000+ active learners
- 4.5/5 star rating
- Community contributions

---

## 🔄 ITERATION PROCESS

**After launch:**
1. Collect user feedback
2. Prioritize fixes (critical → nice-to-have)
3. Update content weekly
4. Improve AI agents based on learnings

**Version control:**
- `main`: Production
- `develop`: Active work
- `feature/X`: New chapters

**Commit format:** `feat(week-03): Add ROS 2 basics chapter`

---

## 🛡️ RISK MANAGEMENT

**If things go wrong:**

| Risk | Solution |
|------|----------|
| Can't finish all 13 weeks today | Ship Weeks 1-5 as MVP, mark rest "Coming Soon" |
| Code examples don't work | Use pseudocode + links to working repos |
| RAG chatbot fails | Deploy without it, add later |
| Design not perfect | Use Docusaurus defaults, iterate post-launch |
| AI agents too slow | Skip validation, manual review only |

**Rule:** Imperfect shipped > perfect in progress

---

## 🎓 EDUCATIONAL THEORY (Quick Ref)

Apply these principles as you create content:

- **Piaget:** Stage-appropriate complexity
- **Vygotsky:** Scaffold from known → unknown
- **Papert:** Learn by building robots
- **Dewey:** Connect theory to practice
- **Montessori:** Self-directed exercises
- **Csíkszentmihályi:** Balance challenge/skill for flow
- **Minsky:** Build mental models progressively
- **Bruner:** Spiral curriculum (revisit concepts)

---

## 📝 AMENDMENT PROCESS

**To change this constitution:**
1. Propose change with rationale
2. Update document
3. Increment version number
4. Document in changelog

**No approval needed.** Just do it if it helps ship.

---

## 🏁 VERSION HISTORY

- **v2.0.2** (2025-12-24): Updated chapter count to "1 chapter" per week in Content Sprint section.
- **v2.0.1** (2025-12-24): Corrected file paths to match project structure.
- **v2.0** (2025-12-24): Lean constitution for rapid completion.
- **v1.0.2** (2025-12-01): Original complex version
- **v1.0.1** (2025-11-30): Added TDD
- **v1.0** (2025-11-30): Initial constitution

---

## 💪 MINDSET

> "Done is better than perfect."  
> "Ship first, polish later."  
> "AI agents are helpers, not gatekeepers."  
> "One decision maker, zero committees."  
> "Today's MVP beats tomorrow's perfection."

---

**Now go build. You've got this. 🚀**

---

END OF CONSTITUTION

**This is a living document.** Update as needed to keep shipping fast.