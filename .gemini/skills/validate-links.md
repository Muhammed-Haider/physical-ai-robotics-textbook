SKILL: validate-links

PURPOSE:
Scan all Markdown files for broken internal links, missing images, and invalid cross-references.

INPUTS:
- docs_directory (string): Path to docs folder (default: "./docs")
- check_external (boolean): Also validate external URLs (default: false, slow)
- fix_relative_paths (boolean): Attempt to auto-fix common path errors (default: true)

PROCESS:
1. Recursively find all .md files in docs_directory
2. For each file:
   - Extract all Markdown links: [text](url)
   - Extract all image references: ![alt](path)
   - Extract all HTML links: <a href="...">
3. Validate internal links:
   - Check if target file exists
   - Handle relative paths (./,../)
   - Verify anchors exist (#heading-id)
4. Validate images:
   - Check file exists in static/img/
   - Verify file extension valid (.png,.jpg,.svg)
5. If fix_relative_paths:
   - Correct common errors:
     - Missing .md extension
     - Wrong path separators
     - Case sensitivity issues
   - Create backup before fixing
6. If check_external (and enabled):
   - Test HTTP/HTTPS URLs (with timeout)
   - Report 404s or timeouts

OUTPUTS:
{
  "status": "validation_complete",
  "files_scanned": 45,
  "total_links": 312,
  "broken_links": [
    {
      "file": "week-03/ros2-fundamentals.md",
      "line": 45,
      "link": "../week-02/python-advanced.md",
      "issue": "File not found",
      "suggested_fix": "../week-02/python-basics.md"
    }
  ],
  "missing_images": [
    {
      "file": "week-08/isaac-sim.md",
      "line": 120,
      "image": "/img/isaac-screenshot.png",
      "issue": "File does not exist"
    }
  ],
  "fixes_applied": 3,
  "manual_review_needed": 2
}

ERROR HANDLING:
- Permission denied: Skip file, report issue
- Malformed Markdown: Report syntax error location

VALIDATION:
- All reported broken links actually broken
- Suggested fixes are valid
- No false positives

USAGE EXAMPLE:
Input: docs_directory="./docs", check_external=false, fix_relative_paths=true
Execute: Scan all chapters, fix common link errors
Output: Report of broken links with suggested fixes