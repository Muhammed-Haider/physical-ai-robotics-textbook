SKILL: create-code-exercise

PURPOSE:
Generate a hands-on coding exercise with starter code, expected output, success criteria, and hints.

INPUTS:
- concept (string): What the exercise teaches (e.g., "ROS 2 publisher node")
- difficulty (string): "beginner" | "intermediate" | "advanced"
- language (string): "python" | "cpp" | "bash" (default: "python")
- provides_starter (boolean): Include starter code with TODOs (default: true)

PROCESS:
1. Generate exercise title based on concept and difficulty
2. Create starter code template:
   - Include necessary imports
   - Provide class/function scaffolding
   - Add TODO comments at key implementation points
   - Include helpful comments explaining structure
3. Define expected output:
   - Show what successful execution looks like
   - Include sample terminal output
   - Specify metrics (e.g., "publishes at 1 Hz")
4. Create success criteria checklist:
   - Observable behaviors student can verify
   - At least 3 criteria
   - Measurable outcomes
5. Write progressive hints:
   - Hint 1: High-level guidance (what to think about)
   - Hint 2: More specific (which function/method)
   - Hint 3: Nearly complete solution (code snippet)
6. Add challenge variation:
   - Extension task for advanced students
   - "Make it your own" customization ideas

OUTPUTS (Markdown formatted):
````markdown
### Exercise: {Generated Title}

**Difficulty**: {Star rating based on difficulty}
**Estimated Time**: {10-30 minutes based on difficulty}

**Objective**: {Clear statement of what student will build}

#### Starter Code
```{language}
{Generated starter template with TODOs}
```

#### Expected Output
````
{Sample execution output}
Success Criteria

 {Criterion 1}
 {Criterion 2}
 {Criterion 3}

Hints
<details>
<summary>Hint 1: Conceptual Guidance</summary>
{High-level hint}
</details>
<details>
<summary>Hint 2: Implementation Direction</summary>
{More specific hint}
</details>
<details>
<summary>Hint 3: Code Snippet</summary>
`````{language}
{Near-solution code snippet}
````
</details>
Challenge Extension
For advanced learners: {Extension task description}

Return as JSON:
{
  "exercise_markdown": "{full markdown content}",
  "estimated_time": 20,
  "difficulty_rating": 3,
  "concepts_covered": ["ROS 2 publishers", "message types"],
  "ready_to_insert": true
}

ERROR HANDLING:
- Unknown concept: Ask for more specific description
- Unsupported language: List supported languages

VALIDATION:
- Starter code has valid syntax
- TODOs clearly marked
- Success criteria are observable

USAGE EXAMPLE:
Input: concept="Create a ROS 2 publisher", difficulty="beginner", language="python"
Execute: Generate complete exercise with starter code and hints
Output: Markdown-formatted exercise ready to insert in chapter