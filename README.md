# Physical AI & Humanoid Robotics: An AI-Native Textbook

[![Project Status](https://img.shields.io/badge/status-active-brightgreen.svg)](https://github.com/Panaversity/ai-book)
[![GitHub Stars](https://img.shields.io/github/stars/Panaversity/ai-book.svg?style=social&label=Star&maxAge=2592000)](https://github.com/Panaversity/ai-book/stargazers)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE.md)

This repository contains the source code for the "Physical AI & Humanoid Robotics: An AI-Native Textbook" project. This is an AI-native digital textbook designed to teach Physical AI and Humanoid Robotics.

> **Note**: This README is automatically updated with the latest project stats by a script.

---

## Project Statistics

- **Stars**: `N/A`
- **Forks**: `N/A`
- **Latest Release**: `N/A`
- **Project Link**: [https://github.com/Panaversity/ai-book](https://github.com/Panaversity/ai-book)

## Project Status

This project is currently under active development.

## Getting Started

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator.

### Prerequisites

- Node.js (version >= 20.0)
- yarn

### Installation

```bash
yarn
```

### Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

### Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

This project is deployed to GitHub Pages. The following command will build the website and push it to the `gh-pages` branch.

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

## Update README Stats

To update the statistics in this README, run the following commands:

```bash
pip install -r scripts/requirements.txt
python scripts/update_readme.py
```

This can be automated by running it in a GitHub Action on a schedule or on push events.

## Contributing

We welcome contributions to this project! Please review our [Project Constitution](.specify/memory/constitution.md) for detailed guidelines on our development workflow, quality standards, and ethical principles.
