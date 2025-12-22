# Physical AI & Humanoid Robotics: An AI-Native Textbook

[![Read the Docs](https://img.shields.io/badge/read-the_docs-blue)](https://muhammed-haider.github.io/physical-ai-robotics-textbook/)
[![Project Status](https://img.shields.io/badge/status-active-brightgreen.svg)](https://github.com/Muhammed-Haider/physical-ai-robotics-textbook)
[![GitHub Stars](https://img.shields.io/github/stars/Muhammed-Haider/physical-ai-robotics-textbook.svg?style=social&label=Star&maxAge=2592000)](https://github.com/Muhammed-Haider/physical-ai-robotics-textbook/stargazers)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE.md)

This repository contains the source for the **Physical AI & Humanoid Robotics**, an AI-native digital textbook designed to teach the principles of building robots that can interact with the physical world.

> **Note**: This README is automatically updated with the latest project stats.

---

## Project Statistics

- **Stars**: `0`
- **Forks**: `0`
- **Latest Release**: `N/A`

## Local Development

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator.

### Prerequisites

- Node.js (version >= 20.0)
- yarn

### Installation & Running

```bash
# Install dependencies
yarn

# Start the local development server
yarn start
```

This command starts a local development server and opens a browser window. Most changes are reflected live without restarting the server.

### Build

```bash
# Build the static site for production
yarn build
```

This command generates static content into the `build` directory.

## Deployment

This project is deployed to GitHub Pages. The following command will build the website and push it to the `gh-pages` branch.

```bash
# Deploy with SSH
USE_SSH=true yarn deploy

# Or deploy with a specific user
GIT_USER=<Your GitHub username> yarn deploy
```

<details>
<summary>How to Update README Stats</summary>

To update the statistics in this README, run the following commands:

```bash
pip install -r scripts/requirements.txt
python scripts/update_readme.py
```

This can be automated by running it in a GitHub Action on a schedule or on push events.
</details>

## Contributing

We welcome contributions to this project! Please review our [Project Constitution](.specify/memory/constitution.md) for detailed guidelines on our development workflow, quality standards, and ethical principles.
