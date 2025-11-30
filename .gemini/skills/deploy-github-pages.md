SKILL: deploy-github-pages

PURPOSE:
Deploy the Docusaurus site to GitHub Pages with proper configuration and verification.

INPUTS:
- github_username (string): GitHub username for deployment URL construction.
- repo_name (string): GitHub repository name for deployment URL construction.
- build_first (boolean): Whether to run a clean build before deployment (default: true).
- verify_links (boolean): Whether to check for broken links before deployment (default: true).

PROCESS:
1. If `build_first` is true:
   - Run: `npm run clear` (clear Docusaurus cache)
   - Run: `npm run build` (build the Docusaurus site)
   - Verify that `build/index.html` exists to confirm successful build.
2. If `verify_links` is true:
   - Run link validation on the build output (e.g., using `docusaurus deploy --dry-run --check-links` if available or similar mechanism).
   - Report any broken links found.
   - If broken links are found, ask the user for confirmation to proceed with deployment.
3. Verify Git configuration:
   - Check if remote 'origin' exists using `git remote -v`.
   - Verify the current branch has commits using `git log -1`.
   - (Implicit) Ensure GitHub credentials are configured for `npm run deploy` to push.
4. Deploy:
   - Run: `npm run deploy` (Docusaurus will use `deployBranch`, `organizationName`, and `projectName` from `docusaurus.config.js`).
   - Monitor the command output for errors during deployment.
5. Post-deploy verification:
   - Wait for a short period (e.g., 30 seconds) to allow GitHub Pages to update.
   - Attempt to fetch the constructed deployed URL (`https://{github_username}.github.io/{repo_name}/`) to verify accessibility.
   - Report the deployment status, including the live URL.

OUTPUTS:
Return JSON:
{
  "status": "deployed" | "build_failed" | "link_check_failed" | "git_error" | "deploy_failed" | "verification_failed",
  "build_successful": boolean,
  "broken_links_found": boolean,
  "deployed_url": string | null,
  "error_message": string | null,
  "next_steps": [
    "Verify the deployed site in your browser",
    "Share the deployed URL"
  ] | null
}

ERROR HANDLING:
- If `npm run build` fails: Set status to "build_failed", `build_successful` to false, capture error messages.
- If link validation fails and the user chooses not to proceed: Set status to "link_check_failed".
- If Git remote 'origin' is not configured or no commits exist: Set status to "git_error", `error_message` with details.
- If `npm run deploy` command fails: Set status to "deploy_failed", capture error messages.
- If the deployed URL is not reachable after deployment: Set status to "verification_failed", `error_message` with details.

VALIDATION:
- Verify `package.json` contains `build`, `clear`, and `deploy` scripts.
- Check for the existence of the `build` directory after a successful build.

USAGE EXAMPLE:
Input: github_username="myusername", repo_name="physical-ai-textbook", build_first=true, verify_links=true
