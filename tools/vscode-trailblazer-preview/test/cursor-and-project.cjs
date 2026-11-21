const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const {
	analyzeJavaSourceSpans,
	discoverWorkspaceRoot,
	isTrailblazerAutoFile,
	resolveCursorTarget,
	resolveGitBaselineSource,
} = require('../project.ts');

const repoRoot = path.resolve(__dirname, '../../..');
const rightTrenchPath = path.join(repoRoot, 'comp-bot/src/main/java/frc/robot/autos/RightTrenchAuto.java');
const leftTrenchPath = path.join(repoRoot, 'comp-bot/src/main/java/frc/robot/autos/LeftTrenchAuto.java');
const abstractBasePath = path.join(repoRoot, 'comp-bot/src/main/java/frc/robot/autos/TrailblazerAuto.java');

void test('discovers Gradle workspace root from Trailblazer auto files and detects auto classes', () => {
	assert.equal(discoverWorkspaceRoot(rightTrenchPath), repoRoot);
	assert.equal(isTrailblazerAutoFile(rightTrenchPath), true);
	assert.equal(isTrailblazerAutoFile(leftTrenchPath), true);
	assert.equal(isTrailblazerAutoFile(abstractBasePath), false);
	assert.throws(() => discoverWorkspaceRoot(path.join(os.tmpdir(), 'NonExistentAuto.java')), /No Gradle workspace/);
});

void test('resolves cursor line to active segment and resolves Git branch baseline source', () => {
	const source = fs.readFileSync(rightTrenchPath, 'utf8');
	const segments = analyzeJavaSourceSpans(source);
	assert.equal(segments.length, 3);
	assert.deepEqual(
		segments.map((s) => s.id),
		['right-trench-cycle-1', 'right-trench-cycle-2', 'right-trench-cycle-3'],
	);

	const lines = source.split(/\r?\n/);
	const lineFor = (snippet) => {
		const index = lines.findIndex((line) => line.includes(snippet));
		assert.ok(index >= 0, `Missing source line for ${snippet}`);
		return index + 1;
	};
	for (const segment of segments) {
		assert.deepEqual(resolveCursorTarget(segments, segment.declarationStartLine, false, null), {
			scope: 'SEGMENT',
			segmentId: segment.id,
		});
	}
	assert.deepEqual(resolveCursorTarget(segments, lineFor('drive(CYCLE_2_SEGMENT)'), false, null), {
		scope: 'SEGMENT',
		segmentId: 'right-trench-cycle-2',
	});
	assert.deepEqual(resolveCursorTarget(segments, lineFor('public final class RightTrenchAuto'), false, null), {
		scope: 'WHOLE_AUTO',
		segmentId: null,
	});
	assert.deepEqual(resolveCursorTarget(segments, segments[1].declarationStartLine, true, segments[1].id), {
		scope: 'WHOLE_AUTO',
		segmentId: null,
	});

	const baseline = resolveGitBaselineSource(repoRoot, rightTrenchPath, source);
	assert.ok(typeof baseline === 'string' && baseline.includes('class RightTrenchAuto'));
});
