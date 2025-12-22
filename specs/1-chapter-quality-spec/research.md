# Research: Pedagogical Agent Definition and Referencing

**Date**: 2025-12-22
**Feature**: `1-chapter-quality-spec`
**Input**: `plan.md` - "NEEDS CLARIFICATION: How are the 8 pedagogical agents defined, versioned, and referenced by this specification?"

## Decision

The 8 pedagogical agents are defined as individual AI prompt chains, stored as markdown files in the `.gemini/agents/` directory.

The mapping from the functional requirements in the specification to the agent files is as follows:

| Functional Requirement | Agent File | Notes |
| :--- | :--- | :--- |
| FR-001 (Developmental Staging) | `.gemini/agents/developmental-staging-agent.md` | Direct match |
| FR-002 (Constructivist Activity)| `.gemini/agents/constructivist-chapter-analyzer.md` | Direct match |
| FR-003 (Motivational Immersion)| `.gemini/agents/motivational-immersion-auditor.md` | Direct match |
| FR-004 (Creative Synthesis) | `.gemini/agents/curriculum-creativity-auditor.md` | Assumed match |
| FR-005 (Modular Mind) | *Not found* | No direct file match found. |
| FR-006 (Contextual Humanity) | `.gemini/agents/contextual-humanity-validator.md` | Direct match |
| FR-007 (Technology Critique) | `.gemini/agents/technology-critique-agent.md` | Direct match |
| FR-008 (Reflective Assessment) | `.gemini/agents/reflective-assessment.md` | Direct match |

This core quality specification (`1-chapter-quality-spec`) will reference these agents by their names. The automated validation workflow (`002-automate-validation-workflow`) is responsible for loading and executing the corresponding agent prompt chain.

Versioning of the agents will be handled through git version control of their respective prompt files.

## Rationale

This approach provides a clear separation of concerns:
- **This spec (`1-chapter-quality-spec`)** defines *what* rules to enforce (the 8 principles).
- **The agent prompt files** define *how* each rule is enforced (the specific AI prompts and logic).
- **The validation workflow** is the execution engine that brings them together.

This separation allows for independent updates. The specification can remain stable while the underlying agent prompts can be refined and improved over time. Using git for versioning is a simple and effective way to track changes to the agents.

## Alternatives Considered

- **Embedding agent logic directly in this spec**: This would make the specification file very large, complex, and difficult to maintain. It would also tightly couple the *what* with the *how*.
- **Using a database to store agent definitions**: This adds unnecessary complexity for this stage of the project. File-based definitions managed via git are sufficient and align with the project's current structure.
