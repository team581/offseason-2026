const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const { WarmPreviewWorkerClient } = require('../worker.ts');
const { findWaypointForSourceLine } = require('../media/preview.ts');

const repoRoot = path.resolve(__dirname, '../../..');
const rightTrenchPath = path.join(repoRoot, 'comp-bot/src/main/java/frc/robot/autos/RightTrenchAuto.java');

void test('warm-JVM stdio worker executes single-segment, whole-auto, diff baseline, compile error stale recovery, and clean shutdown', async () => {
	const sourceText = fs.readFileSync(rightTrenchPath, 'utf8');
	const lines = sourceText.split(/\r?\n/);
	const lineFor = (snippet) => {
		const index = lines.findIndex((line) => line.includes(snippet));
		assert.ok(index >= 0, `Missing source line for ${snippet}`);
		return index + 1;
	};
	const client = new WarmPreviewWorkerClient(repoRoot);

	try {
		const coldStart = performance.now();
		await client.ensureStarted();
		assert.equal(client.isRunning(), true);

		// First authored point of cycle one round-trips to the same waypoint.
		const r1 = await client.requestPreview({
			sourceFile: rightTrenchPath,
			sourceContent: sourceText,
			scope: 'CURSOR',
			cursorLine: lineFor('.through(bluePose(10.089'),
			alliance: 'Blue',
			robotConfigId: '581',
		});
		const coldMs = performance.now() - coldStart;
		assert.equal(r1.ok, true);
		assert.ok(r1.result);
		assert.equal(r1.result.scope, 'SEGMENT');
		assert.equal(r1.result.activeSegmentId, 'right-trench-cycle-1');
		assert.ok(coldMs < 3000, `Expected cold preview < 3000ms, got ${coldMs.toFixed(1)}ms`);
		assert.ok(r1.result.robotFootprint.frontIntakeExtentMeters > r1.result.robotFootprint.halfLengthMeters);
		const firstLine = lineFor('.through(bluePose(10.089');
		const first = findWaypointForSourceLine(r1.result.waypoints, firstLine);
		assert.equal(first, r1.result.waypoints[0]);
		assert.equal(first.sourceLine, firstLine);

		// Cycle two begins with the trench helper and ends with the shared finish.
		const warmStart = performance.now();
		const r2 = await client.requestPreview({
			sourceFile: rightTrenchPath,
			sourceContent: sourceText,
			scope: 'CURSOR',
			cursorLine: lineFor('.through(bluePose(10.428'),
			alliance: 'Blue',
			robotConfigId: '581',
		});
		const warmMs = performance.now() - warmStart;
		assert.equal(r2.ok, true);
		assert.ok(r2.result);
		assert.equal(r2.result.scope, 'SEGMENT');
		assert.equal(r2.result.activeSegmentId, 'right-trench-cycle-2');
		assert.ok(warmMs < 1000, `Expected warm preview < 1000ms, got ${warmMs.toFixed(1)}ms`);
		const cycleTwo = r2.result.waypoints;
		assert.equal(cycleTwo.length, 12);
		assert.equal(cycleTwo[0].sourceLine, lineFor('.through(bluePose(13.2'));
		assert.equal(cycleTwo[3].sourceLine, lineFor('.through(bluePose(10.428'));
		assert.equal(cycleTwo.at(-1).sourceLine, lineFor('.stopAt(SHOOT_POINT'));
		assert.equal(
			findWaypointForSourceLine(cycleTwo, lineFor('.tolerance(Rotation2d.fromDegrees(12)')),
			cycleTwo.at(-2),
		);
		assert.equal(findWaypointForSourceLine(cycleTwo, lineFor('returnToTrench("right-trench-cycle-2"')), null);

		// 3. Whole Auto toggle on Red alliance -> runs all 3 segments and emits NEAR_REVERSAL_CORNER warning
		const rWhole = await client.requestPreview({
			sourceFile: rightTrenchPath,
			sourceContent: sourceText,
			scope: 'WHOLE_AUTO',
			alliance: 'Red',
			robotConfigId: '581',
		});
		assert.equal(rWhole.ok, true);
		assert.ok(rWhole.result);
		assert.equal(rWhole.result.scope, 'WHOLE_AUTO');
		assert.equal(rWhole.result.alliance, 'Red');
		verifyWebview(r1.result, r2.result, rWhole.result);
		assert.ok(rWhole.result.diagnostics.some((d) => d.code === 'NEAR_REVERSAL_CORNER'));

		// A helper call in an untaken branch must not shift the executed goal provenance.
		const withBranch = sourceText.replace(
			'    return Trailblazer.segment(name)',
			'    if (name.isEmpty()) {\n' +
				'      return Trailblazer.segment(name).through(0.0, 0.0);\n' +
				'    }\n' +
				'    return Trailblazer.segment(name)',
		);
		const branchPreview = await client.requestPreview({
			sourceFile: rightTrenchPath,
			sourceContent: withBranch,
			scope: 'SEGMENT',
			segmentId: r2.result.activeSegmentId,
			alliance: 'Blue',
			robotConfigId: '581',
		});
		assert.equal(branchPreview.ok, true);
		assert.equal(branchPreview.result.waypoints[0].sourceLine, lineFor('.through(bluePose(13.2') + 3);
		assert.equal(branchPreview.result.waypoints[3].sourceLine, lineFor('.through(bluePose(10.428'));

		// 4. Compile error returns stale: true and COMPILE_ERROR diagnostic
		const broken = sourceText.replace('.mark(Marker.PREPARE_BUMP));', '.mark(Marker.PREPARE_BUMP)');
		const rBroken = await client.requestPreview({
			sourceFile: rightTrenchPath,
			sourceContent: broken,
			scope: 'CURSOR',
			cursorLine: lineFor('.through(bluePose(10.089'),
			alliance: 'Blue',
			robotConfigId: '581',
		});
		assert.equal(rBroken.ok, false);
		assert.equal(rBroken.stale, true);
		assert.equal(rBroken.errorCode, 'COMPILE_ERROR');
		assert.ok(Array.isArray(rBroken.diagnostics) && rBroken.diagnostics.length > 0);
	} finally {
		client.dispose();
		assert.equal(client.isRunning(), false);
	}
});

function verifyWebview(firstCycle, secondCycle, wholeAuto) {
	const { runInNewContext } = require('node:vm');
	const { createScreenTransform, WAYPOINT_PALETTE } = require('../media/renderer.ts');
	const { boundsToViewport, computeAutoCropBounds } = require('../schema.ts');
	const width = 1000;
	const height = 560;
	const messages = [];
	const elements = new Map();
	const listeners = new Map();
	const timers = new Map();
	let now = 0;
	let timerId = 0;
	let nextFrame;
	let paints = [];
	let arc;

	const context = new Proxy(
		{
			beginPath() {
				arc = null;
			},
			arc(x, y, radius) {
				arc = { x, y, radius };
			},
			stroke() {
				if (arc) paints.push({ ...arc, color: this.strokeStyle });
			},
			fill() {
				if (arc) paints.push({ ...arc, color: this.fillStyle });
			},
		},
		{ get: (target, key) => target[key] ?? (() => {}) },
	);

	function element(name) {
		if (!elements.has(name)) {
			elements.set(name, {
				dataset: {},
				clientWidth: width,
				clientHeight: height,
				setAttribute() {},
				replaceChildren() {},
				append() {},
				getBoundingClientRect: () => ({ left: 0, top: 0, width, height }),
				getContext: () => context,
				addEventListener: (type, callback) => listeners.set(`${name}:${type}`, callback),
			});
		}
		return elements.get(name);
	}

	runInNewContext(fs.readFileSync(path.join(__dirname, '../media/preview.bundle.js'), 'utf8'), {
		document: {
			body: { dataset: {} },
			querySelector: element,
			querySelectorAll: () => [],
			createElement: () => ({ ...element('button'), dataset: {} }),
		},
		window: {
			addEventListener: (type, callback) => listeners.set(type, callback),
			requestAnimationFrame: (callback) => {
				nextFrame = callback;
				return 1;
			},
		},
		Image: class {},
		acquireVsCodeApi: () => ({
			getState: () => undefined,
			postMessage: (message) => messages.push(message),
		}),
		performance: { now: () => now },
		setTimeout: (callback, delay) => {
			timers.set(++timerId, { callback, at: now + delay });
			return timerId;
		},
		clearTimeout: (id) => timers.delete(id),
	});

	function send(data) {
		listeners.get('message')({ data });
	}

	function frame(at) {
		for (const [id, timer] of timers) {
			if (timer.at <= at) {
				now = timer.at;
				timers.delete(id);
				timer.callback();
			}
		}
		now = at;
		paints = [];
		const render = nextFrame;
		nextFrame = undefined;
		assert.ok(render, 'The canvas scheduled a frame');
		render(at);
	}

	function fitPosition(result, waypoint) {
		const viewport = boundsToViewport(computeAutoCropBounds(result, 1));
		const transform = createScreenTransform(width, height, 1, viewport);
		return { x: transform.toScreenX(waypoint.x), y: transform.toScreenY(waypoint.y) };
	}

	function assertHighlightedAt({ x, y }) {
		assert.ok(
			paints.some(
				(paint) =>
					paint.color === WAYPOINT_PALETTE.stroke && paint.radius === 5 && Math.hypot(paint.x - x, paint.y - y) < 0.01,
			),
			`Expected a highlighted waypoint at (${x}, ${y})`,
		);
	}

	function click({ x, y }) {
		listeners.get('#field-canvas:click')({ clientX: x, clientY: y });
	}

	const first = firstCycle.waypoints[0];
	const fitFirst = fitPosition(firstCycle, first);
	send({ type: 'previewResult', result: firstCycle, sourceRevision: 0, cursorLine: first.sourceLine });
	frame(700);
	assertHighlightedAt(fitFirst);
	assert.ok(paints.some((paint) => paint.color === WAYPOINT_PALETTE.highlightHalo));
	assert.ok(paints.some((paint) => paint.color === WAYPOINT_PALETTE.wedgeStroke));

	click(fitFirst);
	assert.equal(messages.at(-1).type, 'revealLine');
	assert.equal(messages.at(-1).line, first.sourceLine);
	frame(1000);
	assertHighlightedAt({ x: width / 2, y: height / 2 });
	frame(2699);
	assertHighlightedAt({ x: width / 2, y: height / 2 });
	frame(3000);
	assertHighlightedAt(fitFirst);

	const inline = secondCycle.waypoints[3];
	const fitInline = fitPosition(secondCycle, inline);
	send({ type: 'previewResult', result: secondCycle, sourceRevision: 0, cursorLine: inline.sourceLine });
	frame(3700);
	assertHighlightedAt(fitInline);
	send({
		type: 'cursorLineChanged',
		cursorLine: inline.sourceLine - 1,
		segmentId: inline.segmentId,
		sourceRevision: 0,
	});
	frame(3710);
	assert.equal(
		paints.some((paint) => paint.color === WAYPOINT_PALETTE.highlightHalo),
		false,
	);

	send({ type: 'sourceEdited', sourceRevision: 1, editedWaypointLine: inline.sourceLine });
	frame(4000);
	assertHighlightedAt({ x: width / 2, y: height / 2 });
	send({ type: 'sourceEdited', sourceRevision: 2, editedWaypointLine: inline.sourceLine });
	send({ type: 'previewStale', sourceRevision: 2 });
	frame(5999);
	assertHighlightedAt({ x: width / 2, y: height / 2 });
	frame(6300);
	assertHighlightedAt(fitInline);
	send({ type: 'previewResult', result: secondCycle, sourceRevision: 2, cursorLine: inline.sourceLine });
	frame(6600);
	assertHighlightedAt(fitInline);

	const wholeFirst = wholeAuto.waypoints[0];
	const fitWhole = fitPosition(wholeAuto, wholeFirst);
	send({
		type: 'previewResult',
		result: wholeAuto,
		sourceRevision: 2,
		cursorLine: wholeFirst.sourceLine,
		pinnedWholeAuto: true,
	});
	frame(7300);
	assertHighlightedAt(fitWhole);
	click(fitWhole);
	frame(7600);
	assertHighlightedAt({ x: width / 2, y: height / 2 });
	frame(9600);
	assertHighlightedAt(fitWhole);
}
