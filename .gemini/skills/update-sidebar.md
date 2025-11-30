SKILL: update-sidebar

PURPOSE:
Automatically update sidebars.js when new chapters are added, maintaining hierarchical structure and proper ordering.

INPUTS:
- chapter_path (string): Path to new chapter (e.g., "week-03/ros2-fundamentals.md")
- sidebar_label (string): Display name in sidebar (optional, extracted from frontmatter if not provided)
- position (integer): Order within week (optional, auto-calculated if not provided)

PROCESS:
1. Read sidebars.js and parse JavaScript structure
2. Extract week number from chapter_path (e.g., "week-03" → 3)
3. If sidebar_label not provided:
   - Read chapter file
   - Extract from frontmatter: sidebar_label or title
4. Determine category:
   - Find or create category for the week
   - Example: "Week 3-5: ROS 2 Fundamentals"
5. Insert chapter reference:
   - Format: 'week-03/ros2-fundamentals'
   - Place in correct position within category items array
   - Maintain alphabetical or numerical order
6. Write updated sidebars.js:
   - Preserve formatting and comments

OUTPUTS:
Return JSON:
{
  "status": "sidebar_updated",
  "chapter_added": "string",
  "sidebar_content": "string"
}

ERROR HANDLING:
- If sidebars.js not found: Return error.
- If chapter_path is invalid or chapter file not found: Return error.
- If frontmatter extraction fails: Use a default label or return an error.

VALIDATION:
- Verify the updated sidebars.js file is valid JavaScript.
- Check that the chapter reference is correctly inserted into the appropriate category.

USAGE EXAMPLE:
Input: chapter_path="week-03/ros2-fundamentals.md", sidebar_label="ROS 2 Fundamentals", position=1
