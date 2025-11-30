SKILL: generate-chapter

PURPOSE:
Generate a complete, pedagogically sound chapter from a specification following the mandatory template structure.

INPUTS:
- spec (JSON object):
  {
    "week": 3,
    "topic": "ROS 2 Fundamentals",
    "learning_outcomes": ["Outcome 1", "Outcome 2"],
    "prerequisites": ["week-02/python-basics"],
    "difficulty_level": "intermediate",
    "key_concepts": ["Nodes", "Topics", "Publishers/Subscribers"],
    "hands_on_exercises": 3,
    "real_world_application": "Autonomous vehicle communication"
  }

PROCESS:
1. Validate spec has all required fields
2. Generate chapter using this EXACT template:
````markdown
---
title: "Week {week}: {topic}"
sidebar_position: {week}
sidebar_label: "{topic}"
---

# Week {week}: {topic}

## Learning Outcomes
By the end of this chapter, you will be able to:
{foreach learning_outcomes}
- {outcome}
{endforeach}

## Prerequisites
:::info Required Knowledge
Before starting, review:
{foreach prerequisites}
- [{prerequisite_title}](../{prerequisite_path})
{endforeach}
:::

## Introduction: Why This Matters
{Generate 2-3 paragraphs linking to real-world applications from spec.real_world_application}
{Include: Problem statement, Why this technology matters, Preview of what student will build}

## Core Concepts

{foreach key_concepts as concept}
### {concept}

{Developmental Staging: Explain concept building on prerequisites}
{Use concrete examples before abstractions}

#### Mental Model Visualization
```mermaid
{Generate appropriate diagram for concept}
```

**Analogy**: {Creative Synthesis: Generate relatable analogy}

#### Code Example
```python
# Starter code demonstrating {concept}
```

**Explanation**: {Walk through code line-by-line}

{endforeach}

## Hands-On Lab: Building {topic} Systems

### Objective
{Constructivist Activity: Clear building goal}

### Challenge Level
**Difficulty**: {Convert difficulty_level to star rating}
{Motivational Immersion: Set expectations}

### Setup
```bash
# Installation and environment setup commands
```

{Generate spec.hands_on_exercises number of exercises}
### Exercise {n}: {Exercise Title}
**Task**: {Specific building task}

**Starter Code**:
```python
# Template with TODOs
```

**Expected Output**:
````
{Show what success looks like}Success Criteria:

 {Criterion 1}
 {Criterion 2}

Hints (if stuck):
<details>
<summary>Click for hint</summary>
{Provide guided hint without full solution}
</details>
Creative Challenge: Design Your Own
{Creative Synthesis: Open-ended prompt}
Prompt: Design a {topic}-based system for {interesting application}. Consider:

{Design question 1}
{Design question 2}
{Design question 3}

No single correct answer—experiment and share your approach!
Real-World Application
{Contextual Humanity: Expand on spec.real_world_application}
Case Study: {Specific industry example}

Challenge: {Problem faced}
Solution: {How this technology helped}
Impact: {Measurable outcomes}

Ethical Consideration: {Relevant ethical question for this technology}
Technology Deep Dive: Why {Primary Tool}?
{Technology Critique: Justify tool choice}
Pedagogical Reason: {Why this tool teaches the concept well}
Advantages:
FeatureBenefitLearning Value.........
When NOT to use: {Acknowledge limitations}
Self-Check Assessment
{Reflective Assessment: Generate 3-5 questions}
Concept Check

Question: {Test understanding of key concept}

A) {Wrong answer}
B) {Correct answer} ✓
C) {Wrong answer}
D) {Wrong answer}



<details><summary>Explanation</summary>
**B is correct** because {explanation}.
Why others are wrong:

A: {Why wrong}
C: {Why wrong}
D: {Why wrong}

</details>

Reflection Prompt: {Metacognitive question linking to prior knowledge}

Checkpoint: Before Moving On
{Reflective Assessment: Self-evaluation checklist}
Ensure you can:

 {Can-do statement 1 from learning outcomes}
 {Can-do statement 2}
 {Can-do statement 3}

Self-Test: {Practical mini-challenge demonstrating readiness}
What's Next?
{Developmental Staging: Preview next chapter}
In Week {week+1}, we'll build on {topic} to explore {next_topic}, enabling {new capability}.
Preview Challenge: {Thought-provoking question about next topic}

Estimated Time: {Calculate based on content: 4-6 hours}
Prerequisites: {List again for reference}
Key Tools: {List software/libraries needed}

3. Fill in all template sections with contextually appropriate content
4. Ensure code examples are syntactically correct and tested
5. Generate appropriate Mermaid diagrams for visual concepts
6. Save to `.claude/generated/week-{week:02d}-{slug}.md`
7. Create metadata file with generation details

OUTPUTS:
{
  "status": "generated",
  "chapter_file": ".claude/generated/week-03-ros2-fundamentals.md",
  "word_count": 3200,
  "code_blocks": 5,
  "diagrams": 2,
  "exercises": 3,
  "self_check_questions": 4,
  "reflection_prompts": 3,
  "ready_for_validation": true,
  "metadata": {
    "generated_at": "2025-11-30T15:45:00Z",
    "spec_version": "1.0",
    "template_version": "1.0"
  }
}

ERROR HANDLING:
- Missing required spec fields: Return error listing missing fields
- Invalid week number: Reject (must be 1-13)
- Can't generate code examples: Use pseudocode with comment explaining limitation

VALIDATION:
- All template sections present
- Markdown syntax valid
- Code blocks have language tags
- Links use correct relative paths
- Frontmatter is valid YAML

USAGE EXAMPLE:
Input: spec={week:3, topic:"ROS 2 Fundamentals", learning_outcomes:[...], ...}
Execute: Generate complete chapter following template
Output: Chapter file path + metadata