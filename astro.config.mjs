// @ts-check
import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';
import starlightBlog from 'starlight-blog';

// https://astro.build/config
export default defineConfig({
	site: 'https://zeulewan.github.io',
	base: '/robot-docs/',
	integrations: [
		starlight({
			title: 'Robot Docs',
			social: [{ icon: 'github', label: 'GitHub', href: 'https://github.com/zeulewan/robot-docs' }],
			plugins: [
				starlightBlog({
					title: 'Setup Log',
					authors: {
						zeul: {
							name: 'Zeul',
						},
					},
				}),
			],
			sidebar: [
				{ label: 'Getting Started', slug: 'getting-started' },
				{ label: 'Current Setup', slug: 'current-setup' },
				{ label: 'Current Progress', slug: 'current-progress' },
				{ label: 'Architecture', slug: 'architecture' },
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
