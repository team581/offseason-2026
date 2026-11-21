import { defineConfig } from 'oxfmt';
export default defineConfig({
	singleQuote: true,
	useTabs: true,
	printWidth: 120,
	ignorePatterns: ['**/CHANGELOG.md', '**/node_modules/**', '**/out/**', 'media/preview.bundle.js'],
});
