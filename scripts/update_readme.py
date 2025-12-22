import os
import re
import requests

def fetch_github_stats(repo_owner, repo_name):
    """Fetches star, fork, and latest release data from GitHub API."""
    stats = {
        "stars": "N/A",
        "forks": "N/A",
        "latest_release": "N/A"
    }
    
    # Fetch repo data (stars and forks)
    repo_url = f"https://api.github.com/repos/{repo_owner}/{repo_name}"
    try:
        response = requests.get(repo_url)
        response.raise_for_status()
        data = response.json()
        stats["stars"] = data.get("stargazers_count", "N/A")
        stats["forks"] = data.get("forks_count", "N/A")
    except requests.exceptions.RequestException as e:
        print(f"Error fetching repo data: {e}")

    # Fetch release data
    release_url = f"https://api.github.com/repos/{repo_owner}/{repo_name}/releases/latest"
    try:
        response = requests.get(release_url)
        response.raise_for_status()
        data = response.json()
        stats["latest_release"] = data.get("tag_name", "N/A")
    except requests.exceptions.RequestException as e:
        # It's common for projects not to have a release, so we handle 404 gracefully
        if e.response and e.response.status_code == 404:
            print("No latest release found.")
            stats["latest_release"] = "None"
        else:
            print(f"Error fetching release data: {e}")

    return stats

def update_readme(repo_owner, repo_name):
    """Updates the README.md file with the latest GitHub stats."""
    stats = fetch_github_stats(repo_owner, repo_name)
    
    try:
        with open("README.md", "r") as f:
            content = f.read()
    except FileNotFoundError:
        print("Error: README.md not found.")
        return

    # Replace placeholders
    content = re.sub(r"{{STARS}}", str(stats["stars"]), content)
    content = re.sub(r"{{FORKS}}", str(stats["forks"]), content)
    content = re.sub(r"{{LATEST_RELEASE}}", str(stats["latest_release"]), content)
    
    # Also update the GitHub repo link
    github_url = f"https://github.com/{repo_owner}/{repo_name}"
    content = re.sub(r"https://github.com/<your-org>/<your-repo>", github_url, content)
    content = re.sub(r"github/stars/<your-org>/<your-repo>", f"github/stars/{repo_owner}/{repo_name}", content)


    try:
        with open("README.md", "w") as f:
            f.write(content)
        print("README.md updated successfully with the latest stats.")
    except IOError as e:
        print(f"Error writing to README.md: {e}")


if __name__ == "__main__":
    # Extract from environment variables or use a default
    # In a GitHub Action, these would be pre-filled
    REPO_OWNER = os.getenv("GITHUB_REPOSITORY_OWNER", "Panaversity")
    REPO_NAME = os.getenv("GITHUB_REPOSITORY_NAME", "ai-book")
    
    # A bit of logic to handle the full GITHUB_REPOSITORY variable if it exists
    repo_var = os.getenv("GITHUB_REPOSITORY")
    if repo_var:
        parts = repo_var.split('/')
        if len(parts) == 2:
            REPO_OWNER, REPO_NAME = parts

    update_readme(REPO_OWNER, REPO_NAME)
