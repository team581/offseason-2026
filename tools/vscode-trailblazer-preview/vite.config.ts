import { defineConfig } from 'vite-plus';
export default defineConfig({
	pack: {
		entry: ['extension.ts'],
		format: ['cjs'],
		outDir: 'out',
		target: 'node20',
		dts: false,
		deps: { neverBundle: ['vscode'] },
	},
	build: {
		outDir: 'media',
		emptyOutDir: false,
		lib: {
			entry: 'media/preview.ts',
			formats: ['iife'],
			name: 'TrailblazerPreview',
			fileName: () => 'preview.bundle.js',
		},
	},
});
