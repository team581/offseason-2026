export const SCHEMA_VERSION = 1;

export interface PointEntry {
	x: number;
	y: number;
}

export interface PoseEntry {
	x: number;
	y: number;
	headingRadians: number;
	headingDegrees: number;
}

export interface SourceSpan {
	kind: 'DECLARATION' | 'HELPER_METHOD' | 'STATE_BRANCH';
	name: string;
	startLine: number;
	endLine: number;
}

export interface SegmentDescriptor {
	id: string;
	fieldName: string;
	index: number;
	declarationStartLine: number;
	declarationEndLine: number;
	relatedSpans: SourceSpan[];
}

export interface PhaseTimings {
	compileMs: number;
	planMs: number;
	simMs: number;
	totalMs: number;
}

export interface BuildIdentity {
	sourceHash: string;
	classpathHash: string;
	compiledAtEpochMs: number;
}

export interface RobotFootprint {
	wheelBaseMeters: number;
	trackWidthMeters: number;
	halfLengthMeters: number;
	halfWidthMeters: number;
	bumperLengthMeters: number;
	bumperWidthMeters: number;
	intakeExtensionMeters: number;
	frontIntakeExtentMeters: number;
}

export interface WaypointEntry {
	index: number;
	segmentId: string;
	segmentIndex: number;
	id: string;
	passType: 'THROUGH' | 'STOP' | 'FINISH_THROUGH';
	x: number;
	y: number;
	withinMeters: number;
	hasHeadingTarget: boolean;
	targetHeadingRadians: number | null;
	targetHeadingDegrees: number | null;
	headingToleranceRadians: number | null;
	headingToleranceDegrees: number | null;
	explicitHeadingTolerance: boolean;
	gatePolicy: string;
	turnDirection: string;
	anchorOffsetMeters: number;
	marker: string | null;
	closestPassMeters: number;
	sourceLine: number | null;
	sourceEndLine?: number | null;
}

export interface MathPieceEntry {
	segmentId: string;
	type: 'LINE' | 'ARC';
	startProgress: number;
	lengthMeters: number;
	entryX: number;
	entryY: number;
	directionX: number;
	directionY: number;
	curvature: number;
}

export interface SampledMathPose {
	segmentId: string;
	s: number;
	x: number;
	y: number;
	headingRadians: number;
	headingDegrees: number;
	curvature: number;
}

export interface MathPathEntry {
	totalLengthMeters: number;
	pieces: MathPieceEntry[];
	anchors: number[];
	sampledPoses: SampledMathPose[];
}

export interface KeepoutPolygon {
	id: string;
	vertices: PointEntry[];
}

export interface KeepoutSet {
	marginMeters: number;
	hardPolygons: KeepoutPolygon[];
	marginPolygons: KeepoutPolygon[];
}

export type DisturbanceKind = 'PUSH' | 'VISION_JUMP' | 'START_OFFSET';
export type DisturbanceFrame = 'FIELD' | 'ROBOT';

export interface DisturbanceEvent {
	id: string;
	enabled: boolean;
	kind: DisturbanceKind;
	frame: DisturbanceFrame;
	timestampSeconds: number;
	durationSeconds: number;
	dxMeters: number;
	dyMeters: number;
	dThetaDegrees: number;
	dvxMetersPerSec: number;
	dvyMetersPerSec: number;
	dOmegaDegreesPerSec: number;
	triggerRelatch: boolean;
}

export interface AppliedDisturbanceEntry {
	id: string;
	kind: DisturbanceKind;
	startTimeSeconds: number;
	endTimeSeconds: number;
	beforePose: PoseEntry;
	afterPose: PoseEntry;
	fieldDxMeters: number;
	fieldDyMeters: number;
	fieldDThetaDegrees: number;
	fieldDvxMetersPerSec: number;
	fieldDvyMetersPerSec: number;
	fieldDOmegaDegreesPerSec: number;
}

export interface ReplannedPathEntry {
	segmentId: string;
	geometryGeneration: number;
	timestampSeconds: number;
	poses: PoseEntry[];
}

export interface TrajectorySample {
	timestampSeconds: number;
	segmentId: string;
	segmentIndex: number;
	x: number;
	y: number;
	headingRadians: number;
	headingDegrees: number;
	vxMetersPerSec: number;
	vyMetersPerSec: number;
	omegaRadiansPerSec: number;
	commandedSpeedMetersPerSec: number;
	achievedSpeedMetersPerSec: number;
	maxSpeedMetersPerSec: number;
	slowdownRatio: number;
	paceSecondsPerMeter: number;
	heatmapNormalized: number;
	progressMeters: number;
	limitingFactor: string;
	status: string;
	diagnostic: string;
	geometryGeneration?: number;
	clearanceMeters?: number;
}

export interface PassedMarkerEntry {
	marker: string;
	segmentId: string;
	timestampSeconds: number;
	progressMeters: number;
	x: number;
	y: number;
}

export interface HeadingArrivalEntry {
	segmentId: string;
	goalIndex: number;
	errorRadians: number;
	errorDegrees: number;
	withinTolerance: boolean;
}

export interface DiagnosticEntry {
	severity: 'ERROR' | 'WARNING';
	code: string;
	message: string;
	segmentId: string | null;
	targetGoal: string | null;
	timestampSeconds: number | null;
	line: number | null;
}

export interface PreviewResult {
	schemaVersion: number;
	autoClass: string;
	scope: 'SEGMENT' | 'WHOLE_AUTO';
	activeSegmentId: string;
	segments: SegmentDescriptor[];
	robotConfigId: '581' | '9991' | '9981';
	alliance: 'Blue' | 'Red';
	seed: number;
	totalDurationSeconds: number;
	phaseTimings: PhaseTimings;
	buildIdentity: BuildIdentity;
	robotFootprint: RobotFootprint;
	startPose: PoseEntry;
	waypoints: WaypointEntry[];
	mathPath: MathPathEntry;
	keepouts: KeepoutSet;
	trajectory: TrajectorySample[];
	passedMarkers: PassedMarkerEntry[];
	headingArrivals: HeadingArrivalEntry[];
	dominantLimitingFactor: string;
	minSlowdownRatio: number;
	diagnostics: DiagnosticEntry[];
	appliedDisturbances?: AppliedDisturbanceEntry[];
	replannedPaths?: ReplannedPathEntry[];
	relatchCount?: number;
	minClearanceMeters?: number;
	maxCrossTrackErrorMeters?: number;
}

export interface WorkerResponse {
	id: string | null;
	type: string;
	ok: boolean;
	stale: boolean;
	errorCode: string | null;
	errorMessage: string | null;
	diagnostics: DiagnosticEntry[] | null;
	segments: SegmentDescriptor[] | null;
	result: PreviewResult | null;
}

export interface FieldCalibration {
	name: string;
	isFTC: boolean;
	coordinateSystem: 'wall-blue';
	imageWidthPx: number;
	imageHeightPx: number;
	topLeft: readonly [number, number];
	bottomRight: readonly [number, number];
	widthInches: number;
	heightInches: number;
	fieldLengthMeters: number;
	fieldWidthMeters: number;
}

export const FIELD_2026_CONFIG: FieldCalibration = {
	name: '2026 Field',
	isFTC: false,
	coordinateSystem: 'wall-blue',
	imageWidthPx: 3902,
	imageHeightPx: 1584,
	topLeft: [524, 95],
	bottomRight: [3378, 1489],
	widthInches: 651.22,
	heightInches: 317.677,
	fieldLengthMeters: 651.22 * 0.0254,
	fieldWidthMeters: 317.677 * 0.0254,
};

export interface BoundingBox {
	minX: number;
	maxX: number;
	minY: number;
	maxY: number;
}

export interface CameraViewport {
	centerX: number;
	centerY: number;
	spanX: number;
	spanY: number;
}

/** Validates and parses a versioned PreviewResult payload, rejecting mismatched schema versions. */
export function parsePreviewResult(input: unknown): PreviewResult {
	const raw = typeof input === 'string' ? (JSON.parse(input) as unknown) : input;
	if (!raw || typeof raw !== 'object') {
		throw new Error('Invalid preview payload: expected an object');
	}
	const record = raw as Record<string, unknown>;
	if (typeof record.schemaVersion !== 'number') {
		throw new Error('Missing schemaVersion in preview payload');
	}
	if (record.schemaVersion !== SCHEMA_VERSION) {
		throw new Error(`Unsupported schemaVersion ${String(record.schemaVersion)} (expected ${SCHEMA_VERSION})`);
	}
	if (typeof record.autoClass !== 'string' || typeof record.totalDurationSeconds !== 'number') {
		throw new Error('Malformed preview payload: missing required envelope fields');
	}
	return raw as PreviewResult;
}

/**
 * Converts WPILib Blue-origin field coordinates (x, y in meters) into exact pixel coordinates
 * on AdvantageScope's 2026 field image (3902 x 1584 px, coordinateSystem: "wall-blue").
 */
export function fieldMetersToImagePixels(
	xMeters: number,
	yMeters: number,
	config: FieldCalibration = FIELD_2026_CONFIG,
): { uPx: number; vPx: number } {
	const [leftPx, topPx] = config.topLeft;
	const [rightPx, bottomPx] = config.bottomRight;
	const uPx = rightPx - (xMeters / config.fieldLengthMeters) * (rightPx - leftPx);
	const vPx = topPx + (yMeters / config.fieldWidthMeters) * (bottomPx - topPx);
	return { uPx, vPx };
}

/** Inverse mapping from AdvantageScope 2026 field image pixel coordinates to Blue-origin meters. */
export function imagePixelsToFieldMeters(
	uPx: number,
	vPx: number,
	config: FieldCalibration = FIELD_2026_CONFIG,
): { xMeters: number; yMeters: number } {
	const [leftPx, topPx] = config.topLeft;
	const [rightPx, bottomPx] = config.bottomRight;
	const xMeters = ((rightPx - uPx) / (rightPx - leftPx)) * config.fieldLengthMeters;
	const yMeters = ((vPx - topPx) / (bottomPx - topPx)) * config.fieldWidthMeters;
	return { xMeters, yMeters };
}

/**
 * Computes the axis-aligned bounding box of the active path (startPose, waypoints + withinMeters,
 * mathPath, trajectory, and swept bumper/intake footprint) expanded by `paddingMeters` (~1.0 m) on all sides.
 */
export function computeAutoCropBounds(result: PreviewResult, paddingMeters = 1.0): BoundingBox {
	let minX = Number.POSITIVE_INFINITY;
	let maxX = Number.NEGATIVE_INFINITY;
	let minY = Number.POSITIVE_INFINITY;
	let maxY = Number.NEGATIVE_INFINITY;

	const includePoint = (x: number, y: number, radius = 0) => {
		if (!Number.isFinite(x) || !Number.isFinite(y)) return;
		const r = Number.isFinite(radius) && radius > 0 ? radius : 0;
		if (x - r < minX) minX = x - r;
		if (x + r > maxX) maxX = x + r;
		if (y - r < minY) minY = y - r;
		if (y + r > maxY) maxY = y + r;
	};

	const rearL = result.robotFootprint?.halfLengthMeters ?? 0.3556;
	const frontL = result.robotFootprint?.frontIntakeExtentMeters ?? result.robotFootprint?.halfLengthMeters ?? 0.3556;
	const maxLongitudinal = Math.max(rearL, frontL);
	const halfW = result.robotFootprint?.halfWidthMeters ?? 0.37465;

	const includePoseFootprint = (x: number, y: number, headingRadians: number) => {
		const absCos = Math.abs(Math.cos(headingRadians));
		const absSin = Math.abs(Math.sin(headingRadians));
		const extX = maxLongitudinal * absCos + halfW * absSin;
		const extY = maxLongitudinal * absSin + halfW * absCos;
		if (x - extX < minX) minX = x - extX;
		if (x + extX > maxX) maxX = x + extX;
		if (y - extY < minY) minY = y - extY;
		if (y + extY > maxY) maxY = y + extY;
	};

	if (result.startPose) {
		includePoseFootprint(result.startPose.x, result.startPose.y, result.startPose.headingRadians);
	}
	for (const wp of result.waypoints) {
		includePoint(wp.x, wp.y, wp.withinMeters);
	}
	for (const mp of result.mathPath.sampledPoses) {
		includePoseFootprint(mp.x, mp.y, mp.headingRadians);
	}
	for (const sample of result.trajectory) {
		includePoseFootprint(sample.x, sample.y, sample.headingRadians);
	}

	if (!Number.isFinite(minX) || !Number.isFinite(maxX)) {
		return {
			minX: 0,
			maxX: FIELD_2026_CONFIG.fieldLengthMeters,
			minY: 0,
			maxY: FIELD_2026_CONFIG.fieldWidthMeters,
		};
	}

	return {
		minX: minX - paddingMeters,
		maxX: maxX + paddingMeters,
		minY: minY - paddingMeters,
		maxY: maxY + paddingMeters,
	};
}

/** Computes a tight local camera viewport zoomed in on a single waypoint. */
export function computeWaypointViewport(
	wp: { x: number; y: number; withinMeters?: number },
	minSpanMeters = 2.7,
): CameraViewport {
	const span = Math.max(minSpanMeters, (wp.withinMeters ?? 0.3) * 5);
	return {
		centerX: wp.x,
		centerY: wp.y,
		spanX: span,
		spanY: span,
	};
}

/**
 * Zooms `viewport` by `zoomFactor` (< 1 zooms in, > 1 zooms out) while keeping the field
 * coordinate under `(cursorCssX, cursorCssY)` stationary on screen.
 */
export function zoomViewportAtCursor(
	viewport: CameraViewport,
	widthCssPx: number,
	heightCssPx: number,
	cursorCssX: number,
	cursorCssY: number,
	zoomFactor: number,
): CameraViewport {
	const w = Math.max(1, widthCssPx);
	const h = Math.max(1, heightCssPx);
	const oldScale = Math.min(w / Math.max(0.5, viewport.spanX), h / Math.max(0.5, viewport.spanY));
	const cx = w * 0.5;
	const cy = h * 0.5;

	const anchorFieldX = viewport.centerX + (cursorCssX - cx) / oldScale;
	const anchorFieldY = viewport.centerY - (cursorCssY - cy) / oldScale;

	const minSpan = 1.1;
	const maxSpan = FIELD_2026_CONFIG.fieldLengthMeters + 4.0;
	const factor = Math.max(0.2, Math.min(5.0, zoomFactor));
	const newSpanX = Math.max(minSpan, Math.min(maxSpan, viewport.spanX * factor));
	const newSpanY = Math.max(minSpan, Math.min(maxSpan, viewport.spanY * factor));
	const newScale = Math.min(w / newSpanX, h / newSpanY);

	return {
		centerX: anchorFieldX - (cursorCssX - cx) / newScale,
		centerY: anchorFieldY + (cursorCssY - cy) / newScale,
		spanX: newSpanX,
		spanY: newSpanY,
	};
}

/** Converts a BoundingBox into a CameraViewport. */
export function boundsToViewport(bounds: BoundingBox): CameraViewport {
	const spanX = Math.max(1.5, bounds.maxX - bounds.minX);
	const spanY = Math.max(1.5, bounds.maxY - bounds.minY);
	return {
		centerX: 0.5 * (bounds.minX + bounds.maxX),
		centerY: 0.5 * (bounds.minY + bounds.maxY),
		spanX,
		spanY,
	};
}

/** Strong ease-out curve matching cubic-bezier(0.23, 1, 0.32, 1) for camera and path reveal animations. */
export function easeOutQuint(t: number): number {
	const clamped = Math.max(0, Math.min(1, t));
	return 1 - Math.pow(1 - clamped, 4);
}

/** Smoothly interpolates between two camera viewports for segment/whole-auto transitions. */
export function interpolateViewport(from: CameraViewport, to: CameraViewport, progress: number): CameraViewport {
	const e = easeOutQuint(progress);
	return {
		centerX: from.centerX + (to.centerX - from.centerX) * e,
		centerY: from.centerY + (to.centerY - from.centerY) * e,
		spanX: from.spanX + (to.spanX - from.spanX) * e,
		spanY: from.spanY + (to.spanY - from.spanY) * e,
	};
}

/**
 * Maps a normalized speed ratio in [0, 1] (0 = slowest/highest pace, 1 = fastest near v_max)
 * to a continuous heatmap CSS color string (hot red/amber -> bright emerald/cyan).
 */
export function heatmapColor(normalizedSpeed: number): string {
	const t = Math.max(0, Math.min(1, Number.isFinite(normalizedSpeed) ? normalizedSpeed : 0));
	let r: number;
	let g: number;
	let b: number;
	if (t < 0.45) {
		const u = t / 0.45;
		r = Math.round(244 + (245 - 244) * u);
		g = Math.round(63 + (158 - 63) * u);
		b = Math.round(94 + (11 - 94) * u);
	} else {
		const u = (t - 0.45) / 0.55;
		r = Math.round(245 + (16 - 245) * u);
		g = Math.round(158 + (185 - 158) * u);
		b = Math.round(11 + (129 - 11) * u);
	}
	return `rgb(${r}, ${g}, ${b})`;
}

/** Interpolates or selects the trajectory sample at `timeSeconds` for timeline hover/pin scrubbing. */
export function sampleTrajectoryAtTime(
	trajectory: readonly TrajectorySample[],
	timeSeconds: number,
): TrajectorySample | null {
	if (!trajectory.length) return null;
	if (timeSeconds <= trajectory[0].timestampSeconds) return trajectory[0];
	const last = trajectory[trajectory.length - 1];
	if (timeSeconds >= last.timestampSeconds) return last;

	let lo = 0;
	let hi = trajectory.length - 1;
	while (lo <= hi) {
		const mid = (lo + hi) >> 1;
		const tMid = trajectory[mid].timestampSeconds;
		if (Math.abs(tMid - timeSeconds) < 1e-6) return trajectory[mid];
		if (tMid < timeSeconds) lo = mid + 1;
		else hi = mid - 1;
	}
	const prev = trajectory[Math.max(0, lo - 1)];
	const next = trajectory[Math.min(trajectory.length - 1, lo)];
	return Math.abs(timeSeconds - prev.timestampSeconds) <= Math.abs(next.timestampSeconds - timeSeconds) ? prev : next;
}

/**
 * Smoothly interpolates pose and telemetry along `trajectory` at `timeSeconds` for high-fps video export,
 * snapping across discontinuous vision-jump steps rather than sliding through them.
 */
export function interpolateTrajectoryAtTime(
	trajectory: readonly TrajectorySample[],
	timeSeconds: number,
): TrajectorySample | null {
	if (!trajectory.length) return null;
	if (timeSeconds <= trajectory[0].timestampSeconds) return trajectory[0];
	const last = trajectory[trajectory.length - 1];
	if (timeSeconds >= last.timestampSeconds) return last;

	let lo = 0;
	let hi = trajectory.length - 1;
	while (lo <= hi) {
		const mid = (lo + hi) >> 1;
		const tMid = trajectory[mid].timestampSeconds;
		if (Math.abs(tMid - timeSeconds) < 1e-6) return trajectory[mid];
		if (tMid < timeSeconds) lo = mid + 1;
		else hi = mid - 1;
	}
	const prev = trajectory[Math.max(0, lo - 1)];
	const next = trajectory[Math.min(trajectory.length - 1, lo)];
	const dt = next.timestampSeconds - prev.timestampSeconds;
	if (dt <= 1e-9) return prev;

	const alpha = Math.max(0, Math.min(1, (timeSeconds - prev.timestampSeconds) / dt));
	if (dt <= 0.05 && Math.hypot(next.x - prev.x, next.y - prev.y) > 0.25) {
		return alpha < 0.5 ? prev : next;
	}

	const lerp = (a: number, b: number) => a + (b - a) * alpha;
	let dTheta = next.headingRadians - prev.headingRadians;
	while (dTheta > Math.PI) dTheta -= Math.PI * 2;
	while (dTheta < -Math.PI) dTheta += Math.PI * 2;
	const headingRadians = prev.headingRadians + dTheta * alpha;
	const discrete = alpha < 0.5 ? prev : next;
	const clearanceMeters =
		typeof prev.clearanceMeters === 'number' && typeof next.clearanceMeters === 'number'
			? lerp(prev.clearanceMeters, next.clearanceMeters)
			: discrete.clearanceMeters;

	return {
		...discrete,
		timestampSeconds: timeSeconds,
		x: lerp(prev.x, next.x),
		y: lerp(prev.y, next.y),
		headingRadians,
		headingDegrees: (headingRadians * 180) / Math.PI,
		vxMetersPerSec: lerp(prev.vxMetersPerSec, next.vxMetersPerSec),
		vyMetersPerSec: lerp(prev.vyMetersPerSec, next.vyMetersPerSec),
		omegaRadiansPerSec: lerp(prev.omegaRadiansPerSec, next.omegaRadiansPerSec),
		commandedSpeedMetersPerSec: lerp(prev.commandedSpeedMetersPerSec, next.commandedSpeedMetersPerSec),
		achievedSpeedMetersPerSec: lerp(prev.achievedSpeedMetersPerSec, next.achievedSpeedMetersPerSec),
		maxSpeedMetersPerSec: lerp(prev.maxSpeedMetersPerSec, next.maxSpeedMetersPerSec),
		slowdownRatio: lerp(prev.slowdownRatio, next.slowdownRatio),
		paceSecondsPerMeter: lerp(prev.paceSecondsPerMeter, next.paceSecondsPerMeter),
		heatmapNormalized: lerp(prev.heatmapNormalized, next.heatmapNormalized),
		progressMeters: lerp(prev.progressMeters, next.progressMeters),
		clearanceMeters,
	};
}
