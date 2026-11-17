import { defineConfig } from 'oxlint';
export default defineConfig({
	categories: {
		correctness: 'error',
		perf: 'error',
	},
	ignorePatterns: ['**/out/**', 'media/preview.bundle.js', 'test/**'],
	rules: {
		'no-await-in-loop': 'off',
		'no-unused-expressions': [
			'error',
			{
				allowShortCircuit: true,
				allowTernary: true,
			},
		],
		'typescript/no-non-null-asserted-optional-chain': 'error',
	},
	options: {
		typeAware: true,
		typeCheck: true,
	},
});
