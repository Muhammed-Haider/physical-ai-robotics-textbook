SKILL: docusaurus-init

PURPOSE:
Initialize a complete Docusaurus project structure for the Physical AI & Humanoid Robotics textbook with proper configuration, folder hierarchy, and deployment setup.

INPUTS:
- project_name (string): Name of the project directory (default: "physical-ai-book")
- github_username (string): GitHub username for deployment
- repo_name (string): GitHub repository name

PROCESS:
1. Run: npx create-docusaurus@latest {project_name} classic --typescript false
2. Navigate to project directory
3. Update docusaurus.config.js:
   - Set title: "Physical AI & Humanoid Robotics"
   - Set tagline: "Bridging Digital Intelligence and Physical Embodiment"
   - Set url: "https://{github_username}.github.io"
   - Set baseUrl: "/{repo_name}/"
   - Set organizationName: {github_username}
   - Set projectName: {repo_name}
   - Set deploymentBranch: "gh-pages"
   - Enable docs-only mode (set to true in preset options)
   - Configure theme: dark mode default, syntax highlighting for Python/bash/yaml
4. C5. Update sidebars.js with hierarchical structure:
````javascript
   module.exports = {
     tutorialSidebar: [
       'intro',
       {
         type: 'category',
         label: 'Week 1-2: Physical AI Foundations',
         items: ['week-01/intro', 'week-02/intro'],
       },
       // ... (structure for all 13 weeks)
     ],
   };
````
6. Create .gitignore with:7. Initialize git repository
8. Create package.json scripts:
   - "deploy": "docusaurus deploy"
   - "validate-all": "node scripts/validate-chapters.js"
9. Create intro.md with course overview, prerequisites, structure

OUTPUTS:
Return JSON:
{
  "status": "initialized",
  "project_path": "./physical-ai-book",
  "structure_created": true,
  "git_initialized": true,
  "ready_for_content": true,
  "next_steps": [
    "Run 'npm install' to install dependencies",
    "Run 'npm start' to test local server",
    "Create GitHub repository and push"
  ]
}

ERROR HANDLING:
- If directory exists: Ask user to confirm overwrite or choose new name
- If npm/npx not available: Return error with installation instructions
- If git not available: Skip git init, warn user

VALIDATION:
- Verify docusaurus.config.js syntax is valid
- Check all folders created successfully
- Ensure intro.md is valid Markdown

USAGE EXAMPLE:
Input: project_name="physical-ai-book", github_username="myusername", repo_name="physical-ai-textbook"
Execut