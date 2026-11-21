const test = require('node:test');
const assert = require('node:assert/strict');
const {
	FIELD_2026_CONFIG,
	SCHEMA_VERSION,
	boundsToViewport,
	computeAutoCropBounds,
	computeWaypointViewport,
	fieldMetersToImagePixels,
	imagePixelsToFieldMeters,
	parsePreviewResult,
	zoomViewportAtCursor,
} = require('../schema.ts');
const { CameraAnimator } = require('../media/preview.ts');
const { CAMERA_TRANSITION_MS, createScreenTransform } = require('../media/renderer.ts');

function sampleMinimalResult() {
	return {
		schemaVersion: SCHEMA_VERSION,
		autoClass: 'frc.robot.autos.RightTrenchAuto',
		scope: 'SEGMENT',
		activeSegmentId: 'right-trench-cycle-1',
		segments: [],
		robotConfigId: '581',
		alliance: 'Blue',
		seed: 581,
		totalDurationSeconds: 5.2,
		phaseTimings: { compileMs: 12, planMs: 1, simMs: 4, totalMs: 17 },
		buildIdentity: { sourceHash: 'abc', classpathHash: 'def', compiledAtEpochMs: 1 },
		robotFootprint: {
			wheelBaseMeters: 22.0 * 0.0254,
			trackWidthMeters: 23.5 * 0.0254,
			halfLengthMeters: 14.0 * 0.0254,
			halfWidthMeters: 14.75 * 0.0254,
			bumperLengthMeters: 28.0 * 0.0254,
			bumperWidthMeters: 29.5 * 0.0254,
			intakeExtensionMeters: 11.8 * 0.0254,
			frontIntakeExtentMeters: 25.8 * 0.0254,
		},
		startPose: { x: 8.3, y: 4.0, headingRadians: 0, headingDegrees: 0 },
		waypoints: [
			{
				index: 0,
				segmentId: 'right-trench-cycle-1',
				segmentIndex: 0,
				id: 'goal-0',
				passType: 'THROUGH',
				x: 10.5,
				y: 5.5,
				withinMeters: 0.3,
				hasHeadingTarget: true,
				targetHeadingRadians: -2.007128,
				targetHeadingDegrees: -115,
				headingToleranceRadians: 0.436332,
				headingToleranceDegrees: 25,
				explicitHeadingTolerance: true,
				gatePolicy: 'SOFT',
				turnDirection: 'SHORTEST',
				anchorOffsetMeters: 0,
				marker: null,
				closestPassMeters: 0.12,
				sourceLine: 52,
			},
			{
				index: 1,
				segmentId: 'right-trench-cycle-1',
				segmentIndex: 0,
				id: 'goal-1',
				passType: 'STOP',
				x: 13.6,
				y: 7.6,
				withinMeters: 0,
				hasHeadingTarget: true,
				targetHeadingRadians: 0,
				targetHeadingDegrees: 0,
				headingToleranceRadians: 0.0349,
				headingToleranceDegrees: 2,
				explicitHeadingTolerance: false,
				gatePolicy: 'SOFT',
				turnDirection: 'SHORTEST',
				anchorOffsetMeters: 0,
				marker: null,
				closestPassMeters: 0.01,
				sourceLine: 60,
			},
		],
		mathPath: {
			totalLengthMeters: 6.5,
			pieces: [],
			anchors: [2.5, 6.5],
			sampledPoses: [
				{
					segmentId: 'right-trench-cycle-1',
					s: 0,
					x: 8.3,
					y: 4.0,
					headingRadians: 0,
					headingDegrees: 0,
					curvature: 0,
				},
				{
					segmentId: 'right-trench-cycle-1',
					s: 6.5,
					x: 13.6,
					y: 7.6,
					headingRadians: 0,
					headingDegrees: 0,
					curvature: 0,
				},
			],
		},
		keepouts: { marginMeters: 0.04, hardPolygons: [], marginPolygons: [] },
		trajectory: [
			{
				timestampSeconds: 0,
				segmentId: 'right-trench-cycle-1',
				segmentIndex: 0,
				x: 8.3,
				y: 4.0,
				headingRadians: 0,
				headingDegrees: 0,
				vxMetersPerSec: 0,
				vyMetersPerSec: 0,
				omegaRadiansPerSec: 0,
				commandedSpeedMetersPerSec: 1.5,
				achievedSpeedMetersPerSec: 0,
				maxSpeedMetersPerSec: 4.02,
				slowdownRatio: 0.37,
				paceSecondsPerMeter: 5,
				heatmapNormalized: 0,
				progressMeters: 0,
				limitingFactor: 'MOTOR_ACCELERATION',
				status: 'RUNNING',
				diagnostic: '',
			},
			{
				timestampSeconds: 5.2,
				segmentId: 'right-trench-cycle-1',
				segmentIndex: 0,
				x: 13.6,
				y: 7.6,
				headingRadians: 0,
				headingDegrees: 0,
				vxMetersPerSec: 0,
				vyMetersPerSec: 0,
				omegaRadiansPerSec: 0,
				commandedSpeedMetersPerSec: 0,
				achievedSpeedMetersPerSec: 0,
				maxSpeedMetersPerSec: 4.02,
				slowdownRatio: 0,
				paceSecondsPerMeter: 5,
				heatmapNormalized: 0,
				progressMeters: 6.5,
				limitingFactor: 'GOAL_DECELERATION',
				status: 'AT_GOAL',
				diagnostic: '',
			},
		],
		passedMarkers: [],
		headingArrivals: [],
		dominantLimitingFactor: 'CORNER_CENTRIPETAL',
		minSlowdownRatio: 0.42,
		diagnostics: [],
	};
}

void test('parsePreviewResult validates schemaVersion and CompTunerConstants + 3-in bumper + 2026 intake footprint', () => {
	const valid = sampleMinimalResult();
	const parsed = parsePreviewResult(JSON.stringify(valid));
	assert.equal(parsed.schemaVersion, 1);
	assert.equal(parsed.activeSegmentId, 'right-trench-cycle-1');
	assert.ok(Math.abs(parsed.robotFootprint.wheelBaseMeters - 22.0 * 0.0254) < 1e-6);
	assert.ok(Math.abs(parsed.robotFootprint.trackWidthMeters - 23.5 * 0.0254) < 1e-6);
	assert.ok(Math.abs(parsed.robotFootprint.bumperLengthMeters - 28.0 * 0.0254) < 1e-6);
	assert.ok(Math.abs(parsed.robotFootprint.bumperWidthMeters - 29.5 * 0.0254) < 1e-6);
	assert.ok(Math.abs(parsed.robotFootprint.intakeExtensionMeters - 11.8 * 0.0254) < 1e-6);
	assert.ok(Math.abs(parsed.robotFootprint.frontIntakeExtentMeters - 25.8 * 0.0254) < 1e-6);

	assert.throws(() => parsePreviewResult('{}'), /Missing schemaVersion/);
	assert.throws(
		() => parsePreviewResult(JSON.stringify({ ...valid, schemaVersion: 2 })),
		/Unsupported schemaVersion 2/,
	);
});

void test('2026 field wall-blue coordinate calibration maps Blue and Red landmarks pixel-accurately', () => {
	const originPx = fieldMetersToImagePixels(0, 0);
	assert.equal(originPx.uPx, 3378);
	assert.equal(originPx.vPx, 95);

	const farCornerPx = fieldMetersToImagePixels(FIELD_2026_CONFIG.fieldLengthMeters, FIELD_2026_CONFIG.fieldWidthMeters);
	assert.ok(Math.abs(farCornerPx.uPx - 524) < 1e-9);
	assert.ok(Math.abs(farCornerPx.vPx - 1489) < 1e-9);

	for (const pt of [
		{ x: 4.625, y: 4.034663 },
		{ x: 11.915394, y: 4.034663 },
		{ x: 12.1, y: 7.6 },
	]) {
		const px = fieldMetersToImagePixels(pt.x, pt.y);
		const back = imagePixelsToFieldMeters(px.uPx, px.vPx);
		assert.ok(Math.abs(back.xMeters - pt.x) < 1e-9);
		assert.ok(Math.abs(back.yMeters - pt.y) < 1e-9);
	}
});

void test('auto-crop, waypoint zoom framing, and cursor-centered scroll-wheel zoom preserve pointer coordinates', () => {
	const sample = sampleMinimalResult();
	sample.robotFootprint = {
		wheelBaseMeters: 0,
		trackWidthMeters: 0,
		halfLengthMeters: 0,
		halfWidthMeters: 0,
		bumperLengthMeters: 0,
		bumperWidthMeters: 0,
		intakeExtensionMeters: 0,
		frontIntakeExtentMeters: 0,
	};
	const bounds = computeAutoCropBounds(sample, 1.0);
	assert.ok(Math.abs(bounds.minX - 7.3) < 1e-6);
	assert.ok(Math.abs(bounds.maxX - 14.6) < 1e-6);
	assert.ok(Math.abs(bounds.minY - 3.0) < 1e-6);
	assert.ok(Math.abs(bounds.maxY - 8.6) < 1e-6);

	// Waypoint zoom framing centers on the waypoint
	const wpVp = computeWaypointViewport({ x: 9.5, y: 4.5, withinMeters: 0.3 });
	assert.equal(wpVp.centerX, 9.5);
	assert.equal(wpVp.centerY, 4.5);
	assert.ok(wpVp.spanX >= 2.4 && wpVp.spanX <= 3.0);

	// Scroll-wheel zoom to cursor preserves exact field coordinate under the pointer
	const vp0 = boundsToViewport(bounds);
	const widthPx = 960;
	const heightPx = 540;
	const cursorX = 720;
	const cursorY = 180;
	const tf0 = createScreenTransform(widthPx, heightPx, 1, vp0);
	const fieldUnderCursorX = tf0.toFieldX(cursorX);
	const fieldUnderCursorY = tf0.toFieldY(cursorY);

	const vpZoomed = zoomViewportAtCursor(vp0, widthPx, heightPx, cursorX, cursorY, 0.65);
	const tf1 = createScreenTransform(widthPx, heightPx, 1, vpZoomed);
	assert.ok(Math.abs(tf1.toFieldX(cursorX) - fieldUnderCursorX) < 1e-9);
	assert.ok(Math.abs(tf1.toFieldY(cursorY) - fieldUnderCursorY) < 1e-9);
	assert.ok(vpZoomed.spanX < vp0.spanX);

	// Smooth camera transition
	const animator = new CameraAnimator(vp0);
	animator.animateTo(wpVp, 100);
	assert.equal(animator.isTransitioning(), true);
	const mid = animator.sample(100 + CAMERA_TRANSITION_MS * 0.5);
	assert.ok(mid.centerX < vp0.centerX && mid.centerX > wpVp.centerX);
	const end = animator.sample(100 + CAMERA_TRANSITION_MS + 10);
	assert.equal(animator.isTransitioning(), false);
	assert.ok(Math.abs(end.centerX - wpVp.centerX) < 1e-9);
});
