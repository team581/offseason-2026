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
const { TimelineController, findWaypointForSourceLine } = require('../media/preview.ts');

void test('webview template has no PIN word badge, includes Diff and Play buttons, and pads timeline away from panel edges', () => {
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

	// Verify timeline bar has >= 16px horizontal padding away from VS Code panel edges
	assert.match(css, /\.timeline-bar\s*\{[^}]*padding:\s*4px\s+18px/);

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
});
