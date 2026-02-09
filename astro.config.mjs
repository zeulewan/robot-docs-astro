// @ts-check
import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';
import starlightBlog from 'starlight-blog';
import starlightClientMermaid from '@pasqal-io/starlight-client-mermaid';

// https://astro.build/config
export default defineConfig({
	site: 'https://zeulewan.github.io',
	base: '/robot-docs/',
	integrations: [
		starlight({
			title: 'Robot Docs',
			customCss: ['./src/styles/custom.css'],
			social: [{ icon: 'github', label: 'GitHub', href: 'https://github.com/zeulewan/robot-docs' }],
			plugins: [
				starlightClientMermaid(),
				starlightBlog({
					title: 'Blog',
					authors: {
						zeul: {
							name: 'Zeul',
						},
					},
				}),
			],
			sidebar: [
				{
					label: 'Overview',
					items: [
						{ label: 'Getting Started', slug: 'getting-started' },
						{ label: 'Current Setup', slug: 'current-setup' },
						{ label: 'Current Progress', slug: 'current-progress' },
						{ label: 'Architecture', slug: 'architecture' },
						{ label: 'About This Site', slug: 'about' },
					],
				},
				{
					label: 'Isaac Sim',
					items: [
						{ label: 'Overview', slug: 'isaac-sim' },
					],
				},
				{
					label: 'Isaac ROS',
					items: [
						{ label: 'Container Setup', slug: 'isaac-ros' },
						{ label: 'Dockerfile Plan', slug: 'isaac-ros/dockerfile-plan' },
					],
				},
				{
					label: 'Workstation',
					items: [
						{ label: 'Hardware & Specs', slug: 'workstation/hardware' },
						{ label: 'GPU & Sunshine', slug: 'workstation/gpu-config' },
						{ label: 'X11 Display', slug: 'workstation/x11-display' },
					],
				},
				{
					label: 'Networking',
					items: [
						{ label: 'Topology', slug: 'networking/topology' },
						{ label: 'Tailscale', slug: 'networking/tailscale' },
						{ label: 'NAT Troubleshooting', slug: 'networking/nat' },
					],
				},
			],
		}),
	],
});
