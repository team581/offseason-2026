const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const { runInNewContext } = require('node:vm');
const { WarmPreviewWorkerClient } = require('../worker.ts');
const {
	createDefaultDisturbance,
	formatDisturbanceJunitSnippet,
	formatDisturbanceSummary,
} = require('../media/preview.ts');
const { createScreenTransform, findNearestTrajectorySample, hitTestDisturbanceGizmo } = require('../media/renderer.ts');

void test('disturbance helpers create defaults, summaries, JUnit snippets, and gizmo hit tests', () => {
	const push = createDefaultDisturbance('PUSH', 1.25);
	assert.equal(push.kind, 'PUSH');
	assert.equal(push.timestampSeconds, 1.25);
	assert.ok(push.durationSeconds > 0);
	assert.match(formatDisturbanceSummary(push), /1\.25s/);

	const jump = createDefaultDisturbance('VISION_JUMP', 2.5);
	assert.equal(jump.kind, 'VISION_JUMP');
	assert.equal(jump.durationSeconds, 0);
	assert.match(formatDisturbanceSummary(jump), /2\.50s/);

	const start = createDefaultDisturbance('START_OFFSET', 1.0);
	assert.equal(start.kind, 'START_OFFSET');
	assert.equal(start.timestampSeconds, 0);
	assert.match(formatDisturbanceSummary(start), /0\.00s/);

	const snippet = formatDisturbanceJunitSnippet('RightTrenchAuto', 'cycle-1', [push, jump]);
	assert.match(snippet, /RightTrenchAuto/);
	assert.match(snippet, /PUSH/);
	assert.match(snippet, /VISION_JUMP/);

	const sampleResult = {
		trajectory: [
			{ timestampSeconds: 0, x: 2.0, y: 2.0, headingRadians: 0 },
			{ timestampSeconds: 1.0, x: 4.0, y: 2.0, headingRadians: 0 },
		],
		appliedDisturbances: [
			{
				id: jump.id,
				kind: 'VISION_JUMP',
				startTimeSeconds: 1.0,
				endTimeSeconds: 1.0,
				beforePose: { x: 4.0, y: 2.0, headingRadians: 0, headingDegrees: 0 },
				afterPose: { x: 4.5, y: 2.8, headingRadians: 0.2, headingDegrees: 11.46 },
				fieldDxMeters: 0.5,
				fieldDyMeters: 0.8,
				fieldDThetaDegrees: 11.46,
				fieldDvxMetersPerSec: 0,
				fieldDvyMetersPerSec: 0,
				fieldDOmegaDegreesPerSec: 0,
			},
		],
	};

	const nearest = findNearestTrajectorySample(sampleResult, 3.9, 2.1);
	assert.equal(nearest?.sample.timestampSeconds, 1.0);

	const tf = createScreenTransform(960, 540, 1, { centerX: 4.0, centerY: 2.4, spanX: 6, spanY: 4 });
	const anchorHit = hitTestDisturbanceGizmo(sampleResult, tf, tf.toScreenX(4.0), tf.toScreenY(2.0));
	assert.deepEqual(anchorHit, { eventId: jump.id, part: 'anchor' });
	const handleHit = hitTestDisturbanceGizmo(sampleResult, tf, tf.toScreenX(4.5), tf.toScreenY(2.8));
	assert.deepEqual(handleHit, { eventId: jump.id, part: 'handle' });
});

void test('disturbance suite webview drawer adds events, posts setDisturbances, and renders recovery summary', () => {
	const elements = new Map();
	const listeners = new Map();
	const postedMessages = [];
	const addButtons = ['PUSH', 'VISION_JUMP', 'START_OFFSET'].map((kind) => {
		const b = makeElement(`add-${kind}`);
		b.dataset.addKind = kind;
		return b;
	});

	function makeElement(name) {
		const attrs = new Map();
		const children = [];
		const el = {
			name,
			dataset: {},
			style: {},
			hidden: false,
			disabled: false,
			checked: false,
			textContent: '',
			value: '',
			clientWidth: 600,
			clientHeight: 300,
			offsetWidth: 120,
			children,
			setAttribute(k, v) {
				attrs.set(k, String(v));
			},
			getAttribute(k) {
				return attrs.get(k) ?? null;
			},
			removeAttribute(k) {
				attrs.delete(k);
			},
			toggleAttribute(k, force) {
				if (force) attrs.set(k, '');
				else attrs.delete(k);
			},
			replaceChildren(...nodes) {
				children.length = 0;
				children.push(...nodes);
			},
			append(...nodes) {
				children.push(...nodes);
			},
			contains(other) {
				return other === el || children.includes(other);
			},
			focus() {},
			select() {},
			setPointerCapture() {},
			hasPointerCapture() {
				return false;
			},
			getBoundingClientRect: () => ({ left: 0, top: 0, width: 600, height: 300 }),
			getContext: () => new Proxy({}, { get: () => () => {} }),
			addEventListener(type, cb) {
				listeners.set(`${name}:${type}`, cb);
			},
		};
		return el;
	}

	function querySelector(sel) {
		if (!elements.has(sel)) {
			elements.set(sel, makeElement(sel));
		}
		return elements.get(sel);
	}

	querySelector('#speed-popover').hidden = true;
	querySelector('#options-popover').hidden = true;
	querySelector('#timeline-hover-line').hidden = true;
	querySelector('#timeline-tooltip').hidden = true;

	let nextFrame;
	runInNewContext(fs.readFileSync(path.join(__dirname, '../media/preview.bundle.js'), 'utf8'), {
		document: {
			body: { dataset: {} },
			addEventListener() {},
			querySelector,
			querySelectorAll(sel) {
				if (sel === '[data-add-kind]') return addButtons;
				return [];
			},
			createElement(tag) {
				return makeElement(tag);
			},
		},
		window: {
			addEventListener(type, cb) {
				if (!listeners.has(`win:${type}`)) listeners.set(`win:${type}`, []);
				listeners.get(`win:${type}`).push(cb);
			},
			requestAnimationFrame(cb) {
				nextFrame = cb;
				return 1;
			},
		},
		Image: class {},
		acquireVsCodeApi: () => ({
			getState: () => undefined,
			setState() {},
			postMessage: (m) => postedMessages.push(m),
		}),
		performance: { now: () => 1000 },
		setTimeout: () => 1,
		clearTimeout() {},
	});

	assert.equal(querySelector('#disturbance-drawer').hidden, true);
	querySelector('#btn-disturbances').onclick();
	assert.equal(querySelector('#disturbance-drawer').hidden, false);

	// Click + Push button
	addButtons[0].onclick();
	const setMsg = postedMessages.find((m) => m.type === 'setDisturbances');
	assert.ok(setMsg);
	assert.equal(setMsg.disturbances.length, 1);
	assert.equal(setMsg.disturbances[0].kind, 'PUSH');
	assert.equal(querySelector('#disturbance-count-badge').textContent, '1');
	assert.equal(querySelector('#disturbance-count-badge').hidden, false);

	// Deliver disturbed preview result + nominal baseline
	const baseResult = {
		schemaVersion: 1,
		autoClass: 'TestAuto',
		scope: 'SEGMENT',
		activeSegmentId: 'seg-1',
		segments: [],
		robotConfigId: '581',
		alliance: 'Blue',
		seed: 1,
		totalDurationSeconds: 2.0,
		phaseTimings: { compileMs: 0, planMs: 1, simMs: 1, totalMs: 2 },
		buildIdentity: { sourceHash: 'a', classpathHash: 'b', compiledAtEpochMs: 0 },
		robotFootprint: {
			wheelBaseMeters: 0.5,
			trackWidthMeters: 0.5,
			halfLengthMeters: 0.35,
			halfWidthMeters: 0.35,
			bumperLengthMeters: 0.7,
			bumperWidthMeters: 0.7,
			intakeExtensionMeters: 0.2,
			frontIntakeExtentMeters: 0.55,
		},
		startPose: { x: 2, y: 2, headingRadians: 0, headingDegrees: 0 },
		waypoints: [],
		mathPath: { totalLengthMeters: 2, pieces: [], anchors: [], sampledPoses: [] },
		keepouts: { marginMeters: 0.1, hardPolygons: [], marginPolygons: [] },
		trajectory: [
			{
				timestampSeconds: 0,
				segmentId: 'seg-1',
				segmentIndex: 0,
				x: 2,
				y: 2,
				headingRadians: 0,
				headingDegrees: 0,
				vxMetersPerSec: 0,
				vyMetersPerSec: 0,
				omegaRadiansPerSec: 0,
				commandedSpeedMetersPerSec: 1,
				achievedSpeedMetersPerSec: 0,
				maxSpeedMetersPerSec: 4,
				slowdownRatio: 0.25,
				paceSecondsPerMeter: 1,
				heatmapNormalized: 0,
				progressMeters: 0,
				limitingFactor: 'TOP_SPEED',
				status: 'RUNNING',
				diagnostic: '',
			},
			{
				timestampSeconds: 2.0,
				segmentId: 'seg-1',
				segmentIndex: 0,
				x: 4,
				y: 2,
				headingRadians: 0,
				headingDegrees: 0,
				vxMetersPerSec: 0,
				vyMetersPerSec: 0,
				omegaRadiansPerSec: 0,
				commandedSpeedMetersPerSec: 0,
				achievedSpeedMetersPerSec: 0,
				maxSpeedMetersPerSec: 4,
				slowdownRatio: 0,
				paceSecondsPerMeter: 5,
				heatmapNormalized: 0,
				progressMeters: 2,
				limitingFactor: 'GOAL_DECELERATION',
				status: 'AT_GOAL',
				diagnostic: '',
			},
		],
		passedMarkers: [],
		headingArrivals: [],
		dominantLimitingFactor: 'TOP_SPEED',
		minSlowdownRatio: 0.5,
		diagnostics: [],
	};
	const disturbedResult = {
		...baseResult,
		totalDurationSeconds: 2.45,
		relatchCount: 1,
		minClearanceMeters: 0.22,
		maxCrossTrackErrorMeters: 0.48,
		appliedDisturbances: [
			{
				id: setMsg.disturbances[0].id,
				kind: 'PUSH',
				startTimeSeconds: 0.4,
				endTimeSeconds: 0.65,
				beforePose: { x: 2.5, y: 2.0, headingRadians: 0, headingDegrees: 0 },
				afterPose: { x: 2.5, y: 2.35, headingRadians: 0, headingDegrees: 0 },
				fieldDxMeters: 0,
				fieldDyMeters: 0.35,
				fieldDThetaDegrees: 0,
				fieldDvxMetersPerSec: -0.6,
				fieldDvyMetersPerSec: 1.4,
				fieldDOmegaDegreesPerSec: 25,
			},
		],
		replannedPaths: [
			{
				segmentId: 'seg-1',
				geometryGeneration: 2,
				timestampSeconds: 0.68,
				poses: [
					{ x: 2.5, y: 2.35, headingRadians: 0, headingDegrees: 0 },
					{ x: 4.0, y: 2.0, headingRadians: 0, headingDegrees: 0 },
				],
			},
		],
	};

	for (const cb of listeners.get('win:message') ?? []) {
		cb({
			data: {
				type: 'previewResult',
				sourceRevision: 0,
				result: disturbedResult,
				baselineResult: baseResult,
				baselineKind: 'disturbance',
				disturbances: setMsg.disturbances,
				diffMode: false,
			},
		});
	}

	assert.equal(querySelector('#diff-delta-badge').hidden, false);
	assert.equal(querySelector('#diff-delta-badge').textContent, 'Δ +0.45 s');
	assert.equal(querySelector('#disturbance-summary').hidden, false);
	assert.equal(querySelector('#disturbance-summary').children.length, 3);
	assert.equal(querySelector('#timeline-events').children.length, 1);
	if (nextFrame) nextFrame(1016);
});

void test('warm-JVM worker executes RightTrenchAuto with PUSH and VISION_JUMP disturbances cleanly', async () => {
	const repoRoot = path.resolve(__dirname, '../../..');
	const rightTrenchPath = path.join(repoRoot, 'comp-bot/src/main/java/frc/robot/autos/RightTrenchAuto.java');
	const sourceText = fs.readFileSync(rightTrenchPath, 'utf8');
	const client = new WarmPreviewWorkerClient(repoRoot);

	try {
		await client.ensureStarted(rightTrenchPath);
		const nominal = await client.requestPreview({
			sourceFile: rightTrenchPath,
			sourceContent: sourceText,
			scope: 'SEGMENT',
			segmentId: 'right-trench-cycle-1',
			alliance: 'Blue',
			robotConfigId: '581',
			disturbances: [],
		});
		assert.equal(nominal.ok, true);
		assert.equal(nominal.result.appliedDisturbances.length, 0);

		const push = createDefaultDisturbance('PUSH', 0.6);
		push.dyMeters = 0.45;
		push.dvyMetersPerSec = 1.6;
		const jump = createDefaultDisturbance('VISION_JUMP', 1.4);

		const disturbed = await client.requestPreview({
			sourceFile: rightTrenchPath,
			sourceContent: sourceText,
			scope: 'SEGMENT',
			segmentId: 'right-trench-cycle-1',
			alliance: 'Blue',
			robotConfigId: '581',
			disturbances: [push, jump],
		});
		assert.equal(disturbed.ok, true);
		assert.equal(disturbed.result.appliedDisturbances.length, 2);
		assert.equal(disturbed.result.relatchCount, 0);
		assert.equal(disturbed.result.replannedPaths.length, 0);
		assert.ok(disturbed.result.passedMarkers.some((m) => m.marker === 'PREPARE_BUMP'));
		assert.equal(disturbed.result.phaseTimings.compileMs, 0);
	} finally {
		client.dispose();
	}
});
