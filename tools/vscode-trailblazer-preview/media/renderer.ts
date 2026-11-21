import {
	FIELD_2026_CONFIG,
	type AppliedDisturbanceEntry,
	type CameraViewport,
	type DisturbanceEvent,
	type DisturbanceKind,
	type PreviewResult,
	type RobotFootprint,
	type TrajectorySample,
	type WaypointEntry,
	easeOutQuint,
	heatmapColor,
	sampleTrajectoryAtTime,
} from '../schema.ts';

export const CAMERA_TRANSITION_MS = 280;
export const PATH_REVEAL_MS = 650;
export const EDIT_ZOOM_HOLD_MS = 2000;

export const WAYPOINT_PALETTE = {
	stroke: '#ff891f',
	fill: 'rgba(255, 137, 31, 0.16)',
	arrow: '#38bdf8',
	wedgeFill: 'rgba(56, 189, 248, 0.36)',
	wedgeStroke: 'rgba(56, 189, 248, 0.90)',
	highlightHalo: 'rgba(255, 137, 31, 0.9)',
} as const;

export interface WaypointGlyphGeometry {
	x: number;
	y: number;
	radiusMeters: number;
	hasArrow: boolean;
	arrowAngleRadians: number | null;
	arrowLengthMeters: number;
	hasToleranceWedge: boolean;
	wedgeStartRadians: number | null;
	wedgeEndRadians: number | null;
	strokeColor: string;
	fillColor: string;
	arrowColor: string;
	wedgeFillColor: string;
	isHighlighted: boolean;
}

/**
 * Computes the exact geometric and high-contrast color primitives for a waypoint glyph:
 * - Bright orange positional marks and cyan rotation marks
 * - Outer circle of radius `withinMeters`
 * - Center dot at `(x, y)`
 * - Interior heading arrow along `targetHeadingRadians` (when `hasHeadingTarget` is true)
 * - Triangular rotational tolerance wedge `[theta - tol, theta + tol]` when `headingToleranceRadians` is set
 */
export function describeWaypointGlyph(wp: WaypointEntry, isHighlighted = false): WaypointGlyphGeometry {
	const radiusMeters = Math.max(0.03, wp.withinMeters);
	const hasArrow = wp.hasHeadingTarget && wp.targetHeadingRadians !== null && Number.isFinite(wp.targetHeadingRadians);
	const arrowAngleRadians = hasArrow ? wp.targetHeadingRadians : null;
	const arrowLengthMeters = hasArrow ? Math.max(0.12, Math.min(0.35, radiusMeters * 0.85)) : 0;
	const hasToleranceWedge =
		hasArrow &&
		wp.headingToleranceRadians !== null &&
		Number.isFinite(wp.headingToleranceRadians) &&
		wp.headingToleranceRadians > 0;
	const wedgeStartRadians =
		hasToleranceWedge && arrowAngleRadians !== null && wp.headingToleranceRadians !== null
			? arrowAngleRadians - wp.headingToleranceRadians
			: null;
	const wedgeEndRadians =
		hasToleranceWedge && arrowAngleRadians !== null && wp.headingToleranceRadians !== null
			? arrowAngleRadians + wp.headingToleranceRadians
			: null;

	const palette = WAYPOINT_PALETTE;

	return {
		x: wp.x,
		y: wp.y,
		radiusMeters,
		hasArrow,
		arrowAngleRadians,
		arrowLengthMeters,
		hasToleranceWedge,
		wedgeStartRadians,
		wedgeEndRadians,
		strokeColor: palette.stroke,
		fillColor: palette.fill,
		arrowColor: palette.arrow,
		wedgeFillColor: palette.wedgeFill,
		isHighlighted,
	};
}

export interface ScreenTransform {
	widthCssPx: number;
	heightCssPx: number;
	dpr: number;
	scalePxPerMeter: number;
	viewport: CameraViewport;
	toScreenX: (fieldX: number) => number;
	toScreenY: (fieldY: number) => number;
	toFieldX: (screenX: number) => number;
	toFieldY: (screenY: number) => number;
}

/** Builds the field-meter <-> CSS-pixel screen transform for a canvas and camera viewport. */
export function createScreenTransform(
	widthCssPx: number,
	heightCssPx: number,
	dpr: number,
	viewport: CameraViewport,
): ScreenTransform {
	const w = Math.max(1, widthCssPx);
	const h = Math.max(1, heightCssPx);
	const scaleX = w / Math.max(0.5, viewport.spanX);
	const scaleY = h / Math.max(0.5, viewport.spanY);
	const scalePxPerMeter = Math.min(scaleX, scaleY);
	const cx = w * 0.5;
	const cy = h * 0.5;

	return {
		widthCssPx: w,
		heightCssPx: h,
		dpr: Math.max(1, dpr),
		scalePxPerMeter,
		viewport,
		toScreenX: (fieldX: number) => cx + (fieldX - viewport.centerX) * scalePxPerMeter,
		toScreenY: (fieldY: number) => cy - (fieldY - viewport.centerY) * scalePxPerMeter,
		toFieldX: (screenX: number) => viewport.centerX + (screenX - cx) / scalePxPerMeter,
		toFieldY: (screenY: number) => viewport.centerY - (screenY - cy) / scalePxPerMeter,
	};
}

/** Hit-tests a CSS pixel click coordinate against the rendered waypoints on the field canvas. */
export function hitTestWaypoint(
	result: PreviewResult | null,
	transform: ScreenTransform,
	screenX: number,
	screenY: number,
): WaypointEntry | null {
	if (!result || result.waypoints.length === 0) return null;
	let bestCenterHit: WaypointEntry | null = null;
	let bestCenterDist = Number.POSITIVE_INFINITY;
	let bestCircleHit: WaypointEntry | null = null;
	let bestCircleDist = Number.POSITIVE_INFINITY;

	for (const wp of result.waypoints) {
		const cx = transform.toScreenX(wp.x);
		const cy = transform.toScreenY(wp.y);
		const distPx = Math.hypot(screenX - cx, screenY - cy);
		const circleRadiusPx = Math.max(12, wp.withinMeters * transform.scalePxPerMeter);

		if (distPx <= 14 && distPx < bestCenterDist) {
			bestCenterDist = distPx;
			bestCenterHit = wp;
		}
		if (distPx <= circleRadiusPx && distPx < bestCircleDist) {
			bestCircleDist = distPx;
			bestCircleHit = wp;
		}
	}
	return bestCenterHit ?? bestCircleHit;
}

export interface PickPreviewState {
	kind: DisturbanceKind;
	x: number;
	y: number;
	headingRadians: number;
	targetX?: number;
	targetY?: number;
}

export interface RenderFieldOptions {
	result: PreviewResult | null;
	baselineResult?: PreviewResult | null;
	diffMode?: boolean;
	disturbances?: readonly DisturbanceEvent[];
	selectedDisturbanceId?: string | null;
	pickPreview?: PickPreviewState | null;
	viewport: CameraViewport;
	revealProgress: number;
	activeTimeSeconds: number | null;
	activeWaypointIndex?: number | null;
	animationClockMs?: number;
	fieldImage: HTMLImageElement | null;
	stale: boolean;
}

/** Finds the closest sample on a trajectory to `(fieldX, fieldY)`. */
export function findNearestTrajectorySample(
	result: PreviewResult | null,
	fieldX: number,
	fieldY: number,
): { sample: TrajectorySample; distMeters: number } | null {
	if (!result || result.trajectory.length === 0) return null;
	let best: TrajectorySample | null = null;
	let bestDist = Number.POSITIVE_INFINITY;
	for (const s of result.trajectory) {
		const d = Math.hypot(s.x - fieldX, s.y - fieldY);
		if (d < bestDist) {
			bestDist = d;
			best = s;
		}
	}
	return best ? { sample: best, distMeters: bestDist } : null;
}

/** Computes the field-space anchor & handle endpoint for an applied or authored disturbance. */
export function getDisturbanceEndpoints(applied: AppliedDisturbanceEntry): {
	anchorX: number;
	anchorY: number;
	handleX: number;
	handleY: number;
} {
	const anchorX = applied.beforePose.x;
	const anchorY = applied.beforePose.y;
	if (applied.kind === 'PUSH') {
		const dispNorm = Math.hypot(applied.fieldDxMeters, applied.fieldDyMeters);
		if (dispNorm > 0.03) {
			return {
				anchorX,
				anchorY,
				handleX: anchorX + applied.fieldDxMeters,
				handleY: anchorY + applied.fieldDyMeters,
			};
		}
		// Friction-only push: show velocity impulse scaled by 0.25s so handle remains visible/draggable
		return {
			anchorX,
			anchorY,
			handleX: anchorX + applied.fieldDvxMetersPerSec * 0.25,
			handleY: anchorY + applied.fieldDvyMetersPerSec * 0.25,
		};
	}
	return {
		anchorX,
		anchorY,
		handleX: applied.afterPose.x,
		handleY: applied.afterPose.y,
	};
}

/** Hit-tests canvas pointer coordinates against disturbance handles and anchors. */
export function hitTestDisturbanceGizmo(
	result: PreviewResult | null,
	transform: ScreenTransform,
	screenX: number,
	screenY: number,
): { eventId: string; part: 'handle' | 'anchor' } | null {
	const appliedList = result?.appliedDisturbances ?? [];
	if (appliedList.length === 0) return null;
	for (let i = appliedList.length - 1; i >= 0; i--) {
		const applied = appliedList[i];
		const pts = getDisturbanceEndpoints(applied);
		const hx = transform.toScreenX(pts.handleX);
		const hy = transform.toScreenY(pts.handleY);
		if (Math.hypot(screenX - hx, screenY - hy) <= 12) {
			return { eventId: applied.id, part: 'handle' };
		}
		const ax = transform.toScreenX(pts.anchorX);
		const ay = transform.toScreenY(pts.anchorY);
		if (Math.hypot(screenX - ax, screenY - ay) <= 12) {
			return { eventId: applied.id, part: 'anchor' };
		}
	}
	return null;
}

/**
 * Renders the 2026 field background, keepouts, optional branch diff overlay, analytical PathBuilder
 * manifold, speed-heatmap closed-loop trajectory, swept/active bumper + 2026 max intake footprint,
 * and waypoint glyphs with active highlight animation onto `canvas`.
 */
export function renderFieldCanvas(canvas: HTMLCanvasElement, options: RenderFieldOptions): ScreenTransform {
	const dpr = typeof window !== 'undefined' && window.devicePixelRatio ? window.devicePixelRatio : 1;
	const cssWidth = canvas.clientWidth || canvas.width || 960;
	const cssHeight = canvas.clientHeight || canvas.height || 540;
	const targetBackingW = Math.max(1, Math.round(cssWidth * dpr));
	const targetBackingH = Math.max(1, Math.round(cssHeight * dpr));
	if (canvas.width !== targetBackingW || canvas.height !== targetBackingH) {
		canvas.width = targetBackingW;
		canvas.height = targetBackingH;
	}

	const transform = createScreenTransform(cssWidth, cssHeight, dpr, options.viewport);
	const ctx = canvas.getContext('2d');
	if (!ctx) return transform;

	ctx.save();
	ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
	ctx.imageSmoothingEnabled = true;
	ctx.imageSmoothingQuality = 'high';

	// 1. High-resolution 2026 AdvantageScope field image (wall-blue calibration)
	drawFieldBackdrop(ctx, transform, cssWidth, cssHeight, options.fieldImage, options.stale);

	const result = options.result;
	if (!result) {
		ctx.restore();
		return transform;
	}

	const reveal = easeOutQuint(options.revealProgress);
	const clockMs = options.animationClockMs ?? 0;

	// 2. Keepout Polygons (K_margin dashed + K_hard solid)
	drawKeepoutPolygons(ctx, transform, result.keepouts);

	// 3. Git Branch Diff / Undisturbed Baseline Overlay
	if (options.baselineResult && (options.diffMode || (result.appliedDisturbances?.length ?? 0) > 0)) {
		drawBaselineDiffOverlay(ctx, transform, options.baselineResult);
	}

	// 4. Raw Waypoint Polyline
	drawWaypointPolyline(ctx, transform, result);

	// 5. Swept Bumper + 2026 Intake Footprint Envelope along the revealed trajectory
	const footprint = result.robotFootprint;
	const maxRevealTime = result.totalDurationSeconds * reveal;
	drawSweptFootprints(ctx, transform, result.trajectory, footprint, maxRevealTime);

	// 6. Mathematical Path Outcome (PathBuilder analytical lines + fillets)
	const maxRevealS = result.mathPath.totalLengthMeters * reveal;
	drawMathPath(ctx, transform, result.mathPath, maxRevealS, 1);

	// 7. Real Closed-Loop Trajectory colored as a continuous speed/pace heatmap
	drawHeatmapTrajectory(ctx, transform, result.trajectory, maxRevealTime, 3.5, 1);

	// 7b. Start-offset PathBuilder Geometries
	if (result.replannedPaths && result.replannedPaths.length > 0) {
		drawReplannedPaths(ctx, transform, result.replannedPaths, maxRevealTime);
	}

	// 8. Waypoint Glyphs (animated active highlight)
	for (const wp of result.waypoints) {
		const isHighlighted = options.activeWaypointIndex === wp.index;
		drawWaypointGlyph(ctx, transform, wp, reveal, isHighlighted, clockMs);
	}

	// 8b. Disturbance Overlays (Push Vectors, Vision Jumps, Start Offsets) & Gizmos
	if (result.appliedDisturbances && result.appliedDisturbances.length > 0) {
		drawDisturbanceOverlays(
			ctx,
			transform,
			result.appliedDisturbances,
			footprint,
			options.selectedDisturbanceId ?? null,
			maxRevealTime,
		);
	}

	// 9. Active Robot Bumper + 2026 Max Intake Extension Footprint at current timeline timestamp
	const displayTime =
		options.revealProgress < 1 ? maxRevealTime : (options.activeTimeSeconds ?? result.totalDurationSeconds);
	const activeSample = sampleTrajectoryAtTime(result.trajectory, displayTime);
	const poseToDraw = activeSample
		? { x: activeSample.x, y: activeSample.y, headingRadians: activeSample.headingRadians }
		: {
				x: result.startPose.x,
				y: result.startPose.y,
				headingRadians: result.startPose.headingRadians,
			};

	drawRobotFootprintWithIntake(ctx, transform, poseToDraw.x, poseToDraw.y, poseToDraw.headingRadians, footprint, true);
	drawCollisionRingIfNegative(ctx, transform, activeSample, poseToDraw, footprint);

	// 10. Interactive Pick / Drag Preview
	if (options.pickPreview) {
		drawPickPreview(ctx, transform, options.pickPreview, footprint);
	}

	ctx.restore();
	return transform;
}

/** Draws the dark canvas background, 2026 field image (or 1m calibration grid), and perimeter border. */
export function drawFieldBackdrop(
	ctx: CanvasRenderingContext2D,
	transform: ScreenTransform,
	widthCssPx: number,
	heightCssPx: number,
	fieldImage: HTMLImageElement | null,
	stale = false,
): void {
	ctx.fillStyle = '#0b0e14';
	ctx.fillRect(0, 0, widthCssPx, heightCssPx);

	const cfg = FIELD_2026_CONFIG;
	const originScreenX = transform.toScreenX(0);
	const originScreenY = transform.toScreenY(0);
	const fieldEndScreenX = transform.toScreenX(cfg.fieldLengthMeters);
	const fieldEndScreenY = transform.toScreenY(cfg.fieldWidthMeters);

	if (fieldImage && fieldImage.complete && fieldImage.naturalWidth > 0) {
		const [leftPx, topPx] = cfg.topLeft;
		const [rightPx, bottomPx] = cfg.bottomRight;
		const pxPerMeterX = (rightPx - leftPx) / cfg.fieldLengthMeters;
		const pxPerMeterY = (bottomPx - topPx) / cfg.fieldWidthMeters;

		ctx.save();
		ctx.translate(originScreenX, originScreenY);
		ctx.scale(-transform.scalePxPerMeter / pxPerMeterX, -transform.scalePxPerMeter / pxPerMeterY);
		ctx.globalAlpha = stale ? 0.55 : 0.88;
		ctx.drawImage(fieldImage, -rightPx, -topPx);
		ctx.restore();
	} else {
		ctx.save();
		ctx.strokeStyle = 'rgba(148, 163, 184, 0.12)';
		ctx.lineWidth = 1;
		for (let xm = 0; xm <= cfg.fieldLengthMeters; xm += 1.0) {
			const sx = transform.toScreenX(xm);
			ctx.beginPath();
			ctx.moveTo(sx, fieldEndScreenY);
			ctx.lineTo(sx, originScreenY);
			ctx.stroke();
		}
		for (let ym = 0; ym <= cfg.fieldWidthMeters; ym += 1.0) {
			const sy = transform.toScreenY(ym);
			ctx.beginPath();
			ctx.moveTo(originScreenX, sy);
			ctx.lineTo(fieldEndScreenX, sy);
			ctx.stroke();
		}
		ctx.restore();
	}

	ctx.save();
	ctx.strokeStyle = 'rgba(148, 163, 184, 0.45)';
	ctx.lineWidth = 1.25;
	ctx.strokeRect(originScreenX, fieldEndScreenY, fieldEndScreenX - originScreenX, originScreenY - fieldEndScreenY);
	ctx.restore();
}

/** Draws K_margin (dashed) and K_hard (filled solid) keepout polygons. */
export function drawKeepoutPolygons(
	ctx: CanvasRenderingContext2D,
	transform: ScreenTransform,
	keepouts: PreviewResult['keepouts'],
): void {
	ctx.save();
	ctx.setLineDash([4, 4]);
	ctx.strokeStyle = 'rgba(244, 63, 94, 0.42)';
	ctx.lineWidth = 1;
	for (const poly of keepouts.marginPolygons) {
		if (poly.vertices.length < 3) continue;
		ctx.beginPath();
		poly.vertices.forEach((v, idx) => {
			const sx = transform.toScreenX(v.x);
			const sy = transform.toScreenY(v.y);
			if (idx === 0) ctx.moveTo(sx, sy);
			else ctx.lineTo(sx, sy);
		});
		ctx.closePath();
		ctx.stroke();
	}

	ctx.setLineDash([]);
	ctx.fillStyle = 'rgba(244, 63, 94, 0.14)';
	ctx.strokeStyle = 'rgba(244, 63, 94, 0.65)';
	ctx.lineWidth = 1.25;
	for (const poly of keepouts.hardPolygons) {
		if (poly.vertices.length < 3) continue;
		ctx.beginPath();
		poly.vertices.forEach((v, idx) => {
			const sx = transform.toScreenX(v.x);
			const sy = transform.toScreenY(v.y);
			if (idx === 0) ctx.moveTo(sx, sy);
			else ctx.lineTo(sx, sy);
		});
		ctx.closePath();
		ctx.fill();
		ctx.stroke();
	}
	ctx.restore();
}

/** Draws the dashed raw waypoint polyline connecting startPose to each waypoint. */
export function drawWaypointPolyline(
	ctx: CanvasRenderingContext2D,
	transform: ScreenTransform,
	result: PreviewResult,
): void {
	if (result.waypoints.length === 0) return;
	ctx.save();
	ctx.setLineDash([5, 5]);
	ctx.strokeStyle = 'rgba(148, 163, 184, 0.42)';
	ctx.lineWidth = 1.2;
	ctx.beginPath();
	ctx.moveTo(transform.toScreenX(result.startPose.x), transform.toScreenY(result.startPose.y));
	for (const wp of result.waypoints) {
		ctx.lineTo(transform.toScreenX(wp.x), transform.toScreenY(wp.y));
	}
	ctx.stroke();
	ctx.restore();
}

/** Draws evenly strided swept robot + intake footprints along the trajectory up to `maxRevealTime`. */
export function drawSweptFootprints(
	ctx: CanvasRenderingContext2D,
	transform: ScreenTransform,
	trajectory: readonly TrajectorySample[],
	footprint: RobotFootprint,
	maxRevealTime: number,
): void {
	if (trajectory.length === 0) return;
	ctx.save();
	ctx.fillStyle = 'rgba(20, 184, 166, 0.05)';
	ctx.strokeStyle = 'rgba(45, 212, 191, 0.16)';
	ctx.lineWidth = 1;
	const stride = Math.max(1, Math.floor(trajectory.length / 26));
	for (let i = 0; i < trajectory.length; i += stride) {
		const s = trajectory[i];
		if (s.timestampSeconds > maxRevealTime + 1e-6) break;
		drawRobotFootprintWithIntake(ctx, transform, s.x, s.y, s.headingRadians, footprint, false);
	}
	ctx.restore();
}

/** Draws the analytical PathBuilder dashed centerline up to arc length `maxRevealS`. */
export function drawMathPath(
	ctx: CanvasRenderingContext2D,
	transform: ScreenTransform,
	mathPath: PreviewResult['mathPath'],
	maxRevealS: number,
	alpha = 1,
): void {
	const mathPoses = mathPath.sampledPoses;
	if (mathPoses.length <= 1) return;
	ctx.save();
	ctx.globalAlpha = alpha;
	ctx.strokeStyle = '#38bdf8';
	ctx.lineWidth = 2.0;
	ctx.setLineDash([7, 4]);
	ctx.beginPath();
	let started = false;
	for (const mp of mathPoses) {
		if (mp.s > maxRevealS + 1e-6) break;
		const sx = transform.toScreenX(mp.x);
		const sy = transform.toScreenY(mp.y);
		if (!started) {
			ctx.moveTo(sx, sy);
			started = true;
		} else {
			ctx.lineTo(sx, sy);
		}
	}
	ctx.stroke();
	ctx.restore();
}

/** Draws the closed-loop trajectory colored by speed heatmap up to `maxRevealTime`. */
export function drawHeatmapTrajectory(
	ctx: CanvasRenderingContext2D,
	transform: ScreenTransform,
	trajectory: readonly TrajectorySample[],
	maxRevealTime: number,
	lineWidth = 3.5,
	alpha = 1,
	tipOverride?: { x: number; y: number; heatmapNormalized: number } | null,
): void {
	if (trajectory.length <= 1) return;
	ctx.save();
	ctx.globalAlpha = alpha;
	ctx.lineCap = 'round';
	ctx.lineJoin = 'round';
	ctx.lineWidth = lineWidth;
	let lastIncluded: TrajectorySample | null = null;
	for (let i = 1; i < trajectory.length; i++) {
		const prev = trajectory[i - 1];
		const curr = trajectory[i];
		if (curr.timestampSeconds > maxRevealTime + 1e-6) {
			lastIncluded = prev;
			break;
		}
		ctx.strokeStyle = heatmapColor(curr.heatmapNormalized);
		ctx.beginPath();
		ctx.moveTo(transform.toScreenX(prev.x), transform.toScreenY(prev.y));
		ctx.lineTo(transform.toScreenX(curr.x), transform.toScreenY(curr.y));
		ctx.stroke();
		lastIncluded = curr;
	}
	if (
		tipOverride &&
		lastIncluded &&
		Math.hypot(tipOverride.x - lastIncluded.x, tipOverride.y - lastIncluded.y) > 1e-4
	) {
		ctx.strokeStyle = heatmapColor(tipOverride.heatmapNormalized);
		ctx.beginPath();
		ctx.moveTo(transform.toScreenX(lastIncluded.x), transform.toScreenY(lastIncluded.y));
		ctx.lineTo(transform.toScreenX(tipOverride.x), transform.toScreenY(tipOverride.y));
		ctx.stroke();
	}
	ctx.restore();
}

/** Draws a dashed red collision warning ring around the robot when clearance is negative. */
export function drawCollisionRingIfNegative(
	ctx: CanvasRenderingContext2D,
	transform: ScreenTransform,
	sample: TrajectorySample | null,
	pose: { x: number; y: number },
	footprint: RobotFootprint,
): void {
	if (!sample || typeof sample.clearanceMeters !== 'number' || sample.clearanceMeters >= 0) return;
	const rx = transform.toScreenX(pose.x);
	const ry = transform.toScreenY(pose.y);
	const radiusPx = Math.max(
		18,
		Math.hypot(footprint.halfLengthMeters, footprint.halfWidthMeters) * transform.scalePxPerMeter + 4,
	);
	ctx.save();
	ctx.strokeStyle = '#f43f5e';
	ctx.lineWidth = 2.4;
	ctx.setLineDash([4, 3]);
	ctx.beginPath();
	ctx.arc(rx, ry, radiusPx, 0, Math.PI * 2);
	ctx.stroke();
	ctx.restore();
}

export function drawReplannedPaths(
	ctx: CanvasRenderingContext2D,
	transform: ScreenTransform,
	replannedPaths: NonNullable<PreviewResult['replannedPaths']>,
	maxRevealTime: number,
) {
	ctx.save();
	for (const rp of replannedPaths) {
		if (rp.timestampSeconds > maxRevealTime + 1e-6 || rp.poses.length < 2) continue;
		ctx.strokeStyle = 'rgba(45, 212, 191, 0.85)';
		ctx.lineWidth = 2.0;
		ctx.setLineDash([5, 4]);
		ctx.beginPath();
		rp.poses.forEach((p, idx) => {
			const sx = transform.toScreenX(p.x);
			const sy = transform.toScreenY(p.y);
			if (idx === 0) ctx.moveTo(sx, sy);
			else ctx.lineTo(sx, sy);
		});
		ctx.stroke();

		const origin = rp.poses[0];
		const ox = transform.toScreenX(origin.x);
		const oy = transform.toScreenY(origin.y);
		ctx.setLineDash([]);
		ctx.fillStyle = '#2dd4bf';
		ctx.strokeStyle = '#0b0e14';
		ctx.lineWidth = 1.4;
		ctx.beginPath();
		ctx.moveTo(ox, oy - 5.5);
		ctx.lineTo(ox + 5.5, oy);
		ctx.lineTo(ox, oy + 5.5);
		ctx.lineTo(ox - 5.5, oy);
		ctx.closePath();
		ctx.fill();
		ctx.stroke();
	}
	ctx.restore();
}

export function drawDisturbanceOverlays(
	ctx: CanvasRenderingContext2D,
	transform: ScreenTransform,
	appliedList: readonly AppliedDisturbanceEntry[],
	footprint: RobotFootprint,
	selectedId: string | null,
	maxRevealTime: number,
) {
	ctx.save();
	for (const applied of appliedList) {
		if (applied.startTimeSeconds > maxRevealTime + 1e-6) continue;
		const isSelected = applied.id === selectedId;
		const color = applied.kind === 'PUSH' ? '#f59e0b' : applied.kind === 'VISION_JUMP' ? '#38bdf8' : '#e879f9';
		const pts = getDisturbanceEndpoints(applied);
		const ax = transform.toScreenX(pts.anchorX);
		const ay = transform.toScreenY(pts.anchorY);
		const hx = transform.toScreenX(pts.handleX);
		const hy = transform.toScreenY(pts.handleY);

		// Ghost robot at beforePose (or afterPose for vision jump)
		ctx.save();
		ctx.strokeStyle = color;
		ctx.fillStyle =
			applied.kind === 'PUSH'
				? 'rgba(245, 158, 11, 0.08)'
				: applied.kind === 'VISION_JUMP'
					? 'rgba(56, 189, 248, 0.09)'
					: 'rgba(232, 121, 249, 0.08)';
		ctx.lineWidth = isSelected ? 1.8 : 1.1;
		ctx.setLineDash([4, 3]);
		drawRobotFootprintWithIntake(
			ctx,
			transform,
			applied.beforePose.x,
			applied.beforePose.y,
			applied.beforePose.headingRadians,
			footprint,
			false,
		);
		if (applied.kind === 'VISION_JUMP') {
			drawRobotFootprintWithIntake(
				ctx,
				transform,
				applied.afterPose.x,
				applied.afterPose.y,
				applied.afterPose.headingRadians,
				footprint,
				false,
			);
		}
		ctx.restore();

		// Vector arrow from anchor to handle
		if (Math.hypot(hx - ax, hy - ay) >= 4) {
			ctx.save();
			if (applied.kind !== 'PUSH') {
				ctx.setLineDash([4, 3]);
			}
			drawScreenArrow(ctx, ax, ay, hx, hy, color, isSelected ? 2.8 : 2.2, 8);
			ctx.restore();
		}

		// Anchor & Handle gizmo nodes
		ctx.save();
		ctx.fillStyle = color;
		ctx.strokeStyle = isSelected ? '#ffffff' : '#0b0e14';
		ctx.lineWidth = isSelected ? 2.0 : 1.4;
		ctx.beginPath();
		ctx.arc(ax, ay, isSelected ? 5.2 : 4.0, 0, Math.PI * 2);
		ctx.fill();
		ctx.stroke();

		ctx.fillStyle = '#0b0e14';
		ctx.strokeStyle = isSelected ? '#ffffff' : color;
		ctx.lineWidth = isSelected ? 2.4 : 1.8;
		ctx.beginPath();
		ctx.arc(hx, hy, isSelected ? 5.8 : 4.5, 0, Math.PI * 2);
		ctx.fill();
		ctx.stroke();
		ctx.restore();
	}
	ctx.restore();
}

function drawPickPreview(
	ctx: CanvasRenderingContext2D,
	transform: ScreenTransform,
	pick: PickPreviewState,
	footprint: RobotFootprint,
) {
	const color = pick.kind === 'PUSH' ? '#f59e0b' : pick.kind === 'VISION_JUMP' ? '#38bdf8' : '#e879f9';
	ctx.save();
	ctx.strokeStyle = color;
	ctx.fillStyle = 'rgba(56, 189, 248, 0.12)';
	ctx.lineWidth = 1.6;
	ctx.setLineDash([4, 3]);
	drawRobotFootprintWithIntake(ctx, transform, pick.x, pick.y, pick.headingRadians, footprint, false);
	if (pick.targetX !== undefined && pick.targetY !== undefined) {
		if (pick.kind !== 'PUSH') {
			drawRobotFootprintWithIntake(ctx, transform, pick.targetX, pick.targetY, pick.headingRadians, footprint, false);
		}
		ctx.setLineDash([]);
		drawScreenArrow(
			ctx,
			transform.toScreenX(pick.x),
			transform.toScreenY(pick.y),
			transform.toScreenX(pick.targetX),
			transform.toScreenY(pick.targetY),
			color,
			2.5,
			8,
		);
	}
	ctx.restore();
}

export function drawBaselineDiffOverlay(
	ctx: CanvasRenderingContext2D,
	transform: ScreenTransform,
	baseline: PreviewResult,
) {
	ctx.save();
	if (baseline.trajectory.length > 1) {
		ctx.setLineDash([6, 4]);
		ctx.strokeStyle = 'rgba(226, 232, 240, 0.65)';
		ctx.lineWidth = 2.4;
		ctx.beginPath();
		baseline.trajectory.forEach((s, idx) => {
			const sx = transform.toScreenX(s.x);
			const sy = transform.toScreenY(s.y);
			if (idx === 0) ctx.moveTo(sx, sy);
			else ctx.lineTo(sx, sy);
		});
		ctx.stroke();
	}
	ctx.setLineDash([2, 3]);
	ctx.strokeStyle = 'rgba(226, 232, 240, 0.55)';
	ctx.fillStyle = 'rgba(226, 232, 240, 0.55)';
	ctx.lineWidth = 1.2;
	for (const wp of baseline.waypoints) {
		const cx = transform.toScreenX(wp.x);
		const cy = transform.toScreenY(wp.y);
		const rPx = Math.max(4, wp.withinMeters * transform.scalePxPerMeter);
		ctx.beginPath();
		ctx.arc(cx, cy, rPx, 0, Math.PI * 2);
		ctx.stroke();
		ctx.beginPath();
		ctx.arc(cx, cy, 2.5, 0, Math.PI * 2);
		ctx.fill();
	}
	ctx.restore();
}

/**
 * Draws the robot footprint matching CompTunerConstants wheel center-to-center + 3-inch bumpers
 * and the forward (+X) 2026 maximum intake extension.
 */
export function drawRobotFootprintWithIntake(
	ctx: CanvasRenderingContext2D,
	transform: ScreenTransform,
	x: number,
	y: number,
	headingRadians: number,
	footprint: RobotFootprint,
	active: boolean,
) {
	const halfL = footprint.halfLengthMeters ?? 0.3556;
	const halfW = footprint.halfWidthMeters ?? 0.37465;
	const wheelHalfX = (footprint.wheelBaseMeters ?? 0.5588) * 0.5;
	const wheelHalfY = (footprint.trackWidthMeters ?? 0.5969) * 0.5;
	const frontIntakeX = footprint.frontIntakeExtentMeters ?? halfL + (footprint.intakeExtensionMeters ?? 0.29972);

	const cos = Math.cos(headingRadians);
	const sin = Math.sin(headingRadians);
	const toWorld = (lx: number, ly: number) => ({
		wx: x + lx * cos - ly * sin,
		wy: y + lx * sin + ly * cos,
	});

	ctx.save();

	// 1. Forward (+X) 2026 max intake extension box
	if (frontIntakeX > halfL + 1e-3) {
		const intakeCorners = [
			toWorld(frontIntakeX, wheelHalfY),
			toWorld(halfL, wheelHalfY),
			toWorld(halfL, -wheelHalfY),
			toWorld(frontIntakeX, -wheelHalfY),
		];
		ctx.beginPath();
		intakeCorners.forEach((c, idx) => {
			const sx = transform.toScreenX(c.wx);
			const sy = transform.toScreenY(c.wy);
			if (idx === 0) ctx.moveTo(sx, sy);
			else ctx.lineTo(sx, sy);
		});
		ctx.closePath();
		if (active) {
			ctx.fillStyle = 'rgba(112, 27, 53, 0.28)';
			ctx.strokeStyle = 'rgba(112, 27, 53, 0.90)';
			ctx.lineWidth = 1.4;
			ctx.setLineDash([3, 3]);
			ctx.fill();
			ctx.stroke();
			ctx.setLineDash([]);
		} else {
			ctx.fill();
			ctx.stroke();
		}
	}

	// 2. Main chassis + 3-inch bumper perimeter rectangle
	const bumperCorners = [
		toWorld(halfL, halfW),
		toWorld(-halfL, halfW),
		toWorld(-halfL, -halfW),
		toWorld(halfL, -halfW),
	];
	ctx.beginPath();
	bumperCorners.forEach((c, idx) => {
		const sx = transform.toScreenX(c.wx);
		const sy = transform.toScreenY(c.wy);
		if (idx === 0) ctx.moveTo(sx, sy);
		else ctx.lineTo(sx, sy);
	});
	ctx.closePath();

	if (active) {
		ctx.fillStyle = 'rgba(112, 27, 53, 0.72)';
		ctx.strokeStyle = '#701b35';
		ctx.lineWidth = 2.0;
		ctx.fill();
		ctx.stroke();

		// 3. CompTunerConstants swerve module wheel center-to-center markers
		ctx.fillStyle = 'rgba(248, 250, 252, 0.85)';
		for (const [mx, my] of [
			[wheelHalfX, wheelHalfY],
			[wheelHalfX, -wheelHalfY],
			[-wheelHalfX, wheelHalfY],
			[-wheelHalfX, -wheelHalfY],
		]) {
			const wpt = toWorld(mx, my);
			ctx.beginPath();
			ctx.arc(transform.toScreenX(wpt.wx), transform.toScreenY(wpt.wy), 2.2, 0, Math.PI * 2);
			ctx.fill();
		}

		// 4. Robot heading vector
		const rx = transform.toScreenX(x);
		const ry = transform.toScreenY(y);
		const tip = toWorld(halfL * 0.88, 0);
		drawScreenArrow(ctx, rx, ry, transform.toScreenX(tip.wx), transform.toScreenY(tip.wy), '#f8fafc', 2.0, 6);
	} else {
		ctx.fill();
		ctx.stroke();
	}

	ctx.restore();
}

export function drawWaypointGlyph(
	ctx: CanvasRenderingContext2D,
	transform: ScreenTransform,
	wp: WaypointEntry,
	reveal: number,
	isHighlighted: boolean,
	clockMs: number,
) {
	const glyph = describeWaypointGlyph(wp, isHighlighted);
	const cx = transform.toScreenX(glyph.x);
	const cy = transform.toScreenY(glyph.y);
	const popScale = 0.92 + 0.08 * reveal;
	const rPx = Math.max(4, glyph.radiusMeters * transform.scalePxPerMeter * popScale);
	const isStop = wp.passType === 'STOP';
	const palette = WAYPOINT_PALETTE;

	ctx.save();
	ctx.globalAlpha = Math.max(0.25, reveal);

	// Animated active waypoint highlight pulse halo
	if (isHighlighted) {
		const phase = ((clockMs % 1100) / 1100) * Math.PI * 2;
		const pulseWave = 0.5 + 0.5 * Math.sin(phase);
		const haloRadiusPx = rPx + 4 + 7 * pulseWave;
		ctx.save();
		ctx.beginPath();
		ctx.arc(cx, cy, haloRadiusPx, 0, Math.PI * 2);
		ctx.strokeStyle = palette.highlightHalo;
		ctx.globalAlpha = 0.35 + 0.45 * (1 - pulseWave);
		ctx.lineWidth = 2.5;
		ctx.stroke();
		ctx.restore();
	}

	// 1. Outer withinMeters circle
	ctx.beginPath();
	ctx.arc(cx, cy, rPx, 0, Math.PI * 2);
	ctx.fillStyle = glyph.fillColor;
	ctx.fill();
	ctx.setLineDash(isStop ? [] : [3, 3]);
	ctx.strokeStyle = '#24170d';
	ctx.lineWidth = isHighlighted ? 4.4 : 3.5;
	ctx.stroke();
	ctx.strokeStyle = glyph.strokeColor;
	ctx.lineWidth = isHighlighted ? 2.4 : 1.5;
	ctx.stroke();
	ctx.setLineDash([]);

	// 2. Rotational tolerance sector wedge ("triangle") inside the circle
	if (glyph.hasToleranceWedge && glyph.wedgeStartRadians !== null && glyph.wedgeEndRadians !== null) {
		const wedgeRadiusPx = Math.max(10, rPx * 0.92);
		const canvasStart = -glyph.wedgeEndRadians;
		const canvasEnd = -glyph.wedgeStartRadians;
		ctx.beginPath();
		ctx.moveTo(cx, cy);
		ctx.arc(cx, cy, wedgeRadiusPx, canvasStart, canvasEnd, false);
		ctx.closePath();
		ctx.fillStyle = glyph.wedgeFillColor;
		ctx.fill();
		ctx.strokeStyle = palette.wedgeStroke;
		ctx.lineWidth = 1.1;
		ctx.stroke();
	}

	// 3. Interior heading arrow starting at center dot
	if (glyph.hasArrow && glyph.arrowAngleRadians !== null) {
		const arrowPx = Math.max(12, Math.min(rPx * 0.88, glyph.arrowLengthMeters * transform.scalePxPerMeter));
		const tipX = cx + Math.cos(glyph.arrowAngleRadians) * arrowPx;
		const tipY = cy - Math.sin(glyph.arrowAngleRadians) * arrowPx;
		drawScreenArrow(ctx, cx, cy, tipX, tipY, glyph.arrowColor, 1.9, 5.5);
	}

	// 4. Center dot at exact (x, y)
	ctx.beginPath();
	ctx.arc(cx, cy, isHighlighted ? 5.0 : 3.6, 0, Math.PI * 2);
	ctx.fillStyle = glyph.strokeColor;
	ctx.fill();
	ctx.strokeStyle = isHighlighted ? '#ffffff' : '#0b0e14';
	ctx.lineWidth = isHighlighted ? 1.6 : 1.0;
	ctx.stroke();

	ctx.restore();
}

function drawScreenArrow(
	ctx: CanvasRenderingContext2D,
	fromX: number,
	fromY: number,
	toX: number,
	toY: number,
	color: string,
	lineWidth: number,
	headLength: number,
) {
	const dx = toX - fromX;
	const dy = toY - fromY;
	const len = Math.hypot(dx, dy);
	if (len < 1e-3) return;
	const angle = Math.atan2(dy, dx);

	ctx.save();
	ctx.strokeStyle = color;
	ctx.fillStyle = color;
	ctx.lineWidth = lineWidth;
	ctx.beginPath();
	ctx.moveTo(fromX, fromY);
	ctx.lineTo(toX, toY);
	ctx.stroke();

	ctx.beginPath();
	ctx.moveTo(toX, toY);
	ctx.lineTo(toX - headLength * Math.cos(angle - Math.PI / 6), toY - headLength * Math.sin(angle - Math.PI / 6));
	ctx.lineTo(toX - headLength * Math.cos(angle + Math.PI / 6), toY - headLength * Math.sin(angle + Math.PI / 6));
	ctx.closePath();
	ctx.fill();
	ctx.restore();
}

export interface TimelineRenderOptions {
	result: PreviewResult | null;
	hoveredTimeSeconds: number | null;
	pinnedTimeSeconds: number | null;
}

/** Renders the bottom timeline track with speed-heatmap bands and passed-marker ticks. */
export function renderTimelineCanvas(canvas: HTMLCanvasElement, options: TimelineRenderOptions): void {
	const dpr = typeof window !== 'undefined' && window.devicePixelRatio ? window.devicePixelRatio : 1;
	const cssWidth = canvas.clientWidth || canvas.width || 900;
	const cssHeight = canvas.clientHeight || canvas.height || 8;
	const backingW = Math.max(1, Math.round(cssWidth * dpr));
	const backingH = Math.max(1, Math.round(cssHeight * dpr));
	if (canvas.width !== backingW || canvas.height !== backingH) {
		canvas.width = backingW;
		canvas.height = backingH;
	}

	const ctx = canvas.getContext('2d');
	if (!ctx) return;

	ctx.save();
	ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
	ctx.clearRect(0, 0, cssWidth, cssHeight);

	const result = options.result;
	if (!result || result.totalDurationSeconds <= 0 || result.trajectory.length === 0) {
		ctx.restore();
		return;
	}

	const totalT = result.totalDurationSeconds;
	const traj = result.trajectory;

	for (let i = 0; i < traj.length; i++) {
		const s: TrajectorySample = traj[i];
		const nextT = i + 1 < traj.length ? traj[i + 1].timestampSeconds : totalT;
		const x0 = (s.timestampSeconds / totalT) * cssWidth;
		const x1 = Math.max(x0 + 1, (nextT / totalT) * cssWidth);
		ctx.fillStyle = heatmapColor(s.heatmapNormalized);
		ctx.globalAlpha = 0.88;
		ctx.fillRect(x0, 0, x1 - x0, cssHeight);
	}
	ctx.globalAlpha = 1;

	ctx.strokeStyle = '#e879f9';
	ctx.lineWidth = 1.5;
	for (const marker of result.passedMarkers) {
		const mx = (marker.timestampSeconds / totalT) * cssWidth;
		ctx.beginPath();
		ctx.moveTo(mx, 0);
		ctx.lineTo(mx, cssHeight);
		ctx.stroke();
	}

	ctx.restore();
}
