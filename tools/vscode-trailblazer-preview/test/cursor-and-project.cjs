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

	// Inside CYCLE_1_SEGMENT (line 52)
	assert.deepEqual(resolveCursorTarget(segments, 52, false, null), {
		scope: 'SEGMENT',
		segmentId: 'right-trench-cycle-1',
	});

	// Inside CYCLE_2_SEGMENT (line 66)
	assert.deepEqual(resolveCursorTarget(segments, 66, false, null), {
		scope: 'SEGMENT',
		segmentId: 'right-trench-cycle-2',
	});

	// Inside CYCLE_3_SEGMENT (line 80)
	assert.deepEqual(resolveCursorTarget(segments, 80, false, null), {
		scope: 'SEGMENT',
		segmentId: 'right-trench-cycle-3',
	});

	// Inside case INTAKE_CYCLE_2 -> drive(CYCLE_2_SEGMENT) (line 145)
	assert.deepEqual(resolveCursorTarget(segments, 145, false, null), {
		scope: 'SEGMENT',
		segmentId: 'right-trench-cycle-2',
	});

	// Inside shared helper returnToTrench (line 116) while coming from cycle 3
	assert.deepEqual(resolveCursorTarget(segments, 116, false, 'right-trench-cycle-3'), {
		scope: 'SEGMENT',
		segmentId: 'right-trench-cycle-3',
	});

	// Outside any segment (line 20, class header) -> WHOLE_AUTO
	assert.deepEqual(resolveCursorTarget(segments, 20, false, null), {
		scope: 'WHOLE_AUTO',
		segmentId: null,
	});

	// Pinned Whole Auto ignores cursor movement inside CYCLE_2_SEGMENT
	assert.deepEqual(resolveCursorTarget(segments, 66, true, 'right-trench-cycle-2'), {
		scope: 'WHOLE_AUTO',
		segmentId: null,
	});

	// Git baseline source resolution returns valid Java source for tracked auto file
	const baseline = resolveGitBaselineSource(repoRoot, rightTrenchPath, source);
	assert.ok(typeof baseline === 'string' && baseline.includes('class RightTrenchAuto'));
});
