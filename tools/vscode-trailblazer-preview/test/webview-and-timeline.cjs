const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const { renderWebview } = require('../webview.ts');
const { heatmapColor } = require('../schema.ts');
const {
	EDIT_ZOOM_HOLD_MS,
	PATH_REVEAL_MS,
	WAYPOINT_PALETTE,
	createScreenTransform,
	describeWaypointGlyph,
	hitTestWaypoint,
} = require('../media/renderer.ts');
const {
	TimelineController,
	findWaypointForSourceLine,
	formatTimelineTime,
	parseTimelineTime,
} = require('../media/preview.ts');

void test('webview template has no PIN word badge, includes Diff and timeline controls', () => {
	const template = fs.readFileSync(path.join(__dirname, '../media/preview.html'), 'utf8');
	const css = fs.readFileSync(path.join(__dirname, '../media/preview.css'), 'utf8');
	const html = renderWebview(template, {
		cspSource: 'https://vscode-resource.test',
		nonce: 'nonce-123',
		stylesheet: 'https://vscode-resource.test/preview.css?a=1&b=2',
		script: 'https://vscode-resource.test/preview".bundle.js',
		fieldImage: 'https://vscode-resource.test/field.png',
	});

	assert.match(html, /script-src 'nonce-nonce-123'/);
	assert.match(html, /nonce="nonce-123"/);
	assert.match(html, /preview\.css\?a=1&amp;b=2/);
	assert.match(html, /preview&quot;\.bundle\.js/);
	assert.doesNotMatch(html, /\{\{/);

	// Verify PIN word badge is removed from top bar while Diff and Play buttons are present
	assert.doesNotMatch(html, /id="pin-badge"/);
	assert.doesNotMatch(html, />PIN</);
	assert.match(html, /id="btn-diff"/);
	assert.match(html, /id="diff-delta-badge"/);
	assert.match(html, /id="btn-play"/);
	assert.match(html, /id="btn-speed"/);
	assert.match(html, /id="btn-cursor-time"/);
	assert.match(html, /id="timeline-slider"/);
	assert.match(html, /id="timeline-playhead"/);
	assert.match(html, /id="timeline-hover-line"/);
	assert.match(html, /id="timeline-tooltip"/);

	assert.match(css, /\.timeline-bar\s*\{[^}]*height:\s*56px/);
	assert.match(css, /\.timeline-bar\s*\{[^}]*gap:\s*4px/);
	assert.match(css, /\.timeline-bar\s*\{[^}]*padding:\s*0\s+8px/);
	assert.match(css, /\.timeline-slider\s*\{[^}]*padding:\s*0\s+8px/);
	assert.match(css, /\.timeline-alert-dot\s*\{[^}]*height:\s*6px/);
	assert.match(css, /\.timeline-track\s*\{[^}]*height:\s*8px/);

	// Zero-fluff check: no paragraphs (<p>), headings (<h1..h6>), or instructional sentences
	assert.doesNotMatch(html, /<p[\s>]/i);
	assert.doesNotMatch(html, /<h[1-6][\s>]/i);
	assert.ok(PATH_REVEAL_MS < 1000);
	assert.equal(EDIT_ZOOM_HOLD_MS, 2000);
});

void test('waypoint glyphs distinguish orange position from cyan heading and support source selection', () => {
	const wpThrough = {
		index: 0,
		segmentId: 'seg-1',
		segmentIndex: 0,
		id: 'goal-0',
		passType: 'THROUGH',
		x: 8.744,
		y: 6.049,
		withinMeters: 0.3,
		hasHeadingTarget: true,
		targetHeadingRadians: (-115 * Math.PI) / 180,
		targetHeadingDegrees: -115,
		headingToleranceRadians: (25 * Math.PI) / 180,
		headingToleranceDegrees: 25,
		explicitHeadingTolerance: true,
		gatePolicy: 'SOFT',
		turnDirection: 'SHORTEST',
		anchorOffsetMeters: 0,
		marker: null,
		closestPassMeters: 0.1,
		sourceLine: 52,
		sourceEndLine: 53,
	};

	const withHeadingAndTol = describeWaypointGlyph(wpThrough, true);
	assert.equal(withHeadingAndTol.radiusMeters, 0.3);
	assert.equal(withHeadingAndTol.hasArrow, true);
	assert.equal(withHeadingAndTol.hasToleranceWedge, true);
	assert.equal(withHeadingAndTol.isHighlighted, true);
	assert.equal(withHeadingAndTol.strokeColor, '#ff891f');
	assert.equal(withHeadingAndTol.arrowColor, '#38bdf8');
	assert.equal(withHeadingAndTol.wedgeFillColor, WAYPOINT_PALETTE.wedgeFill);
	assert.ok(Math.abs(withHeadingAndTol.arrowAngleRadians - (-115 * Math.PI) / 180) < 1e-6);
	assert.ok(Math.abs(withHeadingAndTol.wedgeStartRadians - (-140 * Math.PI) / 180) < 1e-6);
	assert.ok(Math.abs(withHeadingAndTol.wedgeEndRadians - (-90 * Math.PI) / 180) < 1e-6);

	// Canvas click hit-testing & source-line matching
	const tf = createScreenTransform(960, 540, 1, { centerX: 8.744, centerY: 6.049, spanX: 6, spanY: 4 });
	const sx = tf.toScreenX(8.744);
	const sy = tf.toScreenY(6.049);
	const hit = hitTestWaypoint({ waypoints: [wpThrough] }, tf, sx + 3, sy - 2);
	assert.equal(hit?.index, 0);
	assert.equal(findWaypointForSourceLine([wpThrough], 52)?.index, 0);
	assert.equal(findWaypointForSourceLine([wpThrough], 53)?.index, 0);
	assert.equal(findWaypointForSourceLine([wpThrough], 54), null);
	const sameHelperOtherSegment = { ...wpThrough, segmentId: 'seg-2', index: 2 };
	const shared = [wpThrough, sameHelperOtherSegment];
	assert.equal(findWaypointForSourceLine(shared, 52), null);
	assert.equal(findWaypointForSourceLine(shared, 52, 'seg-2'), sameHelperOtherSegment);

	assert.notEqual(heatmapColor(0), heatmapColor(1));
});

void test('timeline controller supports hover-to-scrub, click-to-pin, and 1.0x real-time playback', () => {
	const timeline = new TimelineController();
	timeline.setTotalDuration(4.0);

	// Free hover follows pointer
	assert.equal(timeline.isPinned(), false);
	assert.equal(timeline.hoverAtTime(2.4), 2.4);
	assert.equal(timeline.activeTimeSeconds(), 2.4);

	// Click at 2.40 s pins the cursor
	timeline.clickAtTime(2.4);
	assert.equal(timeline.isPinned(), true);
	assert.equal(timeline.activeTimeSeconds(), 2.4);

	// Moving pointer away keeps cursor pinned at 2.40 s
	timeline.hoverAtTime(3.8);
	assert.equal(timeline.activeTimeSeconds(), 2.4);
	timeline.clearHover();
	assert.equal(timeline.activeTimeSeconds(), 2.4);

	// Play button starts 1.0x real-time playback from current pinned time (2.40 s)
	assert.equal(timeline.togglePlay(1000), true);
	assert.equal(timeline.isPlaying(), true);
	assert.ok(Math.abs(timeline.stepPlayback(1500) - 2.9) < 1e-6);
	assert.ok(Math.abs(timeline.stepPlayback(3000) - 4.0) < 1e-6);
	// Automatically stops at totalDurationSeconds (4.0 s)
	assert.equal(timeline.isPlaying(), false);

	// Clicking Play when at the end restarts from 0.00 s
	assert.equal(timeline.togglePlay(4000), true);
	assert.equal(timeline.activeTimeSeconds(), 0);
	assert.ok(Math.abs(timeline.stepPlayback(4750) - 0.75) < 1e-6);

	// Playback speed multiplier and scroll-to-scrub delta seeking
	timeline.setSpeed(2);
	assert.equal(timeline.getSpeed(), 2);
	assert.ok(Math.abs(timeline.stepPlayback(5000) - 1.25) < 1e-6);
	assert.equal(timeline.changeSpeed(-1), 1);
	timeline.seekByFractionDelta(0.25, 5000);
	assert.ok(Math.abs(timeline.activeTimeSeconds() - 2.25) < 1e-6);

	assert.equal(formatTimelineTime(0), '0:00.00');
	assert.equal(formatTimelineTime(65.5), '1:05.50');
	assert.equal(parseTimelineTime('1:05.5'), 65.5);
	assert.equal(parseTimelineTime('90'), 90);
	assert.equal(parseTimelineTime('invalid'), undefined);
});

void test('timeline bar webview supports scroll-to-scrub, pointer drag, fault dots, speed menu, and cursor time', () => {
	const { runInNewContext } = require('node:vm');
	const elements = new Map();
	const listeners = new Map();
	const speedOptions = [0.25, 0.5, 1, 2, 4, 8].map((speed) => {
		const el = makeElement(`speed-${speed}`);
		el.dataset.speed = String(speed);
		return el;
	});

	function makeElement(name) {
		const attrs = new Map();
		const children = [];
		const elListeners = new Map();
		const el = {
			name,
			dataset: {},
			style: {},
			hidden: false,
			disabled: false,
			textContent: '',
			value: '',
			clientWidth: 400,
			clientHeight: 8,
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
			getBoundingClientRect: () => ({ left: 0, top: 0, width: 400, height: 24 }),
			getContext: () =>
				new Proxy(
					{},
					{
						get: () => () => {},
					},
				),
			addEventListener(type, cb) {
				elListeners.set(type, cb);
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

	// Start with popovers hidden
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
				return sel === '.speed-option' ? speedOptions : [];
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
			postMessage() {},
		}),
		performance: { now: () => 1000 },
		setTimeout: () => 1,
		clearTimeout() {},
	});

	const sampleResult = {
		schemaVersion: 1,
		autoClass: 'TestAuto',
		scope: 'WHOLE_AUTO',
		activeSegmentId: 'seg-1',
		segments: [],
		robotConfigId: '581',
		alliance: 'Blue',
		seed: 1,
		totalDurationSeconds: 4.0,
		phaseTimings: { compileMs: 1, planMs: 1, simMs: 1, totalMs: 3 },
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
				vxMetersPerSec: 1,
				vyMetersPerSec: 0,
				omegaRadiansPerSec: 0,
				commandedSpeedMetersPerSec: 1,
				achievedSpeedMetersPerSec: 1,
				maxSpeedMetersPerSec: 4,
				slowdownRatio: 0.25,
				paceSecondsPerMeter: 1,
				heatmapNormalized: 0.25,
				progressMeters: 0,
				limitingFactor: 'CURVATURE',
				status: 'OK',
				diagnostic: '',
			},
			{
				timestampSeconds: 4,
				segmentId: 'seg-1',
				segmentIndex: 0,
				x: 4,
				y: 2,
				headingRadians: 0,
				headingDegrees: 0,
				vxMetersPerSec: 3,
				vyMetersPerSec: 0,
				omegaRadiansPerSec: 0,
				commandedSpeedMetersPerSec: 3,
				achievedSpeedMetersPerSec: 3,
				maxSpeedMetersPerSec: 4,
				slowdownRatio: 0.75,
				paceSecondsPerMeter: 0.33,
				heatmapNormalized: 0.75,
				progressMeters: 2,
				limitingFactor: 'NONE',
				status: 'OK',
				diagnostic: '',
			},
		],
		passedMarkers: [],
		headingArrivals: [],
		dominantLimitingFactor: 'NONE',
		minSlowdownRatio: 0.25,
		diagnostics: [
			{
				severity: 'WARNING',
				code: 'NEAR_REVERSAL_CORNER',
				message: 'Sharp turn',
				segmentId: 'seg-1',
				targetGoal: null,
				timestampSeconds: 2.0,
				line: 42,
			},
		],
	};

	for (const cb of listeners.get('win:message')) {
		cb({ data: { type: 'previewResult', result: sampleResult, sourceRevision: 1 } });
	}
	nextFrame?.(1000);

	// Fault rendered above track & track shifts down
	assert.equal(querySelector('#timeline-scrubber').dataset.hasAlerts, 'true');
	assert.equal(querySelector('#timeline-alerts').children.length, 1);
	assert.equal(querySelector('#timeline-alerts').children[0].style.left, '50.000%');
	assert.equal(querySelector('#duration-readout').textContent, '0:04.00');
	assert.equal(querySelector('#btn-cursor-time').textContent, '0:04.00');

	// Pointer hover shows hover line and fault tooltip
	listeners.get('#timeline-slider:pointermove')({ clientX: 200, pointerId: 1 });
	assert.equal(querySelector('#timeline-hover-line').hidden, false);
	assert.equal(querySelector('#timeline-hover-line').style.left, '50.000%');
	assert.equal(querySelector('#timeline-tooltip').hidden, false);

	// Horizontal wheel scroll scrubs timeline and prevents default browser navigation
	let prevented = false;
	listeners.get('#timeline-slider:wheel')({
		deltaX: 100,
		preventDefault() {
			prevented = true;
		},
	});
	assert.equal(prevented, true);
	// Natural scrolling: +deltaX scrolls backward (-100 / 400 = -0.25 of 4.0s = -1.0s from 4.0s -> 3.0s)
	assert.equal(querySelector('#btn-cursor-time').textContent, '0:03.00');
	assert.equal(querySelector('#timeline-playhead').style.left, '75.000%');

	// Selecting 2x speed updates display
	speedOptions[3].onclick();
	assert.equal(querySelector('#speed-value').textContent, '2x');
});
