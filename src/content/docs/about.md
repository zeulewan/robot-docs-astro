---
title: About This Site
description: How this documentation site is built and maintained.
---

Built with [Astro Starlight](https://starlight.astro.build/), a documentation framework optimized for fast, accessible, SEO-friendly docs sites.

## Stack

| Component | Tool |
|---|---|
| Framework | [Astro](https://astro.build/) + [Starlight](https://starlight.astro.build/) |
| Blog | [starlight-blog](https://github.com/HiDeoo/starlight-blog) plugin |
| Diagrams | [Mermaid](https://mermaid.js.org/) via [@pasqal-io/starlight-client-mermaid](https://github.com/pasqal-io/starlight-client-mermaid) |
| Hosting | [GitHub Pages](https://pages.github.com/) via GitHub Actions |
| Development | [Claude Code](https://claude.ai/) (AI-assisted) |

## Source

[zeulewan/robot-docs](https://github.com/zeulewan/robot-docs) on GitHub. Content lives in `src/content/docs/` as Markdown files. Push to `main` triggers a GitHub Actions workflow that builds and deploys to Pages.
