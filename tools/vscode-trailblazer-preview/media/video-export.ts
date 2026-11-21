import { ArrayBufferTarget as Mp4ArrayBufferTarget, Muxer as Mp4Muxer } from 'mp4-muxer';
import {
	type CameraViewport,
	type DisturbanceEvent,
	type PassedMarkerEntry,
	type PreviewResult,
	type TrajectorySample,
	boundsToViewport,
	computeAutoCropBounds,
	heatmapColor,
	interpolateTrajectoryAtTime,
} from '../schema.ts';
import {
	type ScreenTransform,
	createScreenTransform,
	drawBaselineDiffOverlay,
	drawCollisionRingIfNegative,
	drawDisturbanceOverlays,
	drawFieldBackdrop,
	drawHeatmapTrajectory,
	drawKeepoutPolygons,
	drawMathPath,
	drawReplannedPaths,
	drawRobotFootprintWithIntake,
	drawSweptFootprints,
	drawWaypointGlyph,
	drawWaypointPolyline,
} from './renderer.ts';

export const VIDEO_WIDTH = 1280;
export const VIDEO_HEIGHT = 720;
export const VIDEO_TIMELINE_HEIGHT = 48;
export const VIDEO_END_HOLD_SECONDS = 0.45;

export interface VideoExportPlan {
	fps: 30 | 60;
	durationSeconds: number;
	holdSeconds: number;
	motionFrames: number;
	holdFrames: number;
	totalFrames: number;
	width: number;
	height: number;
	stageHeight: number;
}

export interface VideoRenderContext {
	result: PreviewResult;
	baselineResult: PreviewResult | null;
	diffMode: boolean;
	disturbances: readonly DisturbanceEvent[];
	fieldImage: HTMLImageElement | null;
	includeTelemetry: boolean;
}

export interface VideoExportOutcome {
	format: 'mp4' | 'webm';
	bytesBase64: string;
	fps: number;
	frameCount: number;
	elapsedMs: number;
	codec: string;
}

/** Computes frame rate and frame counts so even 20 s autos stay under ~600 frames for < 3 s export. */
export function computeVideoExportPlan(totalDurationSeconds: number, includeTelemetry = true): VideoExportPlan {
	const durationSeconds = Math.max(0.1, Number.isFinite(totalDurationSeconds) ? totalDurationSeconds : 0.1);
	const fps: 30 | 60 = durationSeconds <= 10 ? 60 : 30;
	const holdSeconds = VIDEO_END_HOLD_SECONDS;
	const motionFrames = Math.max(2, Math.round(durationSeconds * fps));
	const holdFrames = Math.round(holdSeconds * fps);
	return {
		fps,
		durationSeconds,
		holdSeconds,
		motionFrames,
		holdFrames,
		totalFrames: motionFrames + holdFrames,
		width: VIDEO_WIDTH,
		height: VIDEO_HEIGHT,
		stageHeight: includeTelemetry ? VIDEO_HEIGHT - VIDEO_TIMELINE_HEIGHT : VIDEO_HEIGHT,
	};
}

/** Computes an auto-cropped viewport with subtle top headroom so corner HUD pills never crowd the path. */
export function computeVideoViewport(result: PreviewResult, includeTelemetry = true): CameraViewport {
	const vp = boundsToViewport(computeAutoCropBounds(result, includeTelemetry ? 1.2 : 1.0));
	if (!includeTelemetry) return vp;
	const paddedSpanY = vp.spanY * 1.12;
	return {
		centerX: vp.centerX,
		centerY: vp.centerY + paddedSpanY * 0.03,
		spanX: vp.spanX * 1.04,
		spanY: paddedSpanY,
	};
}

export function formatVideoScopeLabel(result: PreviewResult): string {
	return result.scope === 'SEGMENT' && result.activeSegmentId ? result.activeSegmentId : 'Whole Auto';
}

export function formatVideoTime(seconds: number): string {
	if (!Number.isFinite(seconds) || seconds < 0) return '0:00.00';
	const hundredths = Math.round(seconds * 100);
	const minutes = Math.floor(hundredths / 6000);
	const secs = ((hundredths % 6000) / 100).toFixed(2).padStart(5, '0');
	return `${minutes}:${secs}`;
}

export function buildSuggestedVideoFilename(result: PreviewResult | null): string {
	if (!result) return 'trailblazer-path.mp4';
	const cleanAuto = result.autoClass.replace(/[^A-Za-z0-9_-]+/g, '-');
	const scopePart =
		result.scope === 'SEGMENT' && result.activeSegmentId
			? result.activeSegmentId.replace(/[^A-Za-z0-9_-]+/g, '-')
			: 'WholeAuto';
	return `${cleanAuto}-${scopePart}-${result.alliance}-${result.robotConfigId}.mp4`;
}

export function uint8ArrayToBase64(bytes: Uint8Array): string {
	const chunkSize = 0x2000;
	const chunks: string[] = [];
	for (let i = 0; i < bytes.length; i += chunkSize) {
		const slice = bytes.subarray(i, Math.min(i + chunkSize, bytes.length));
		chunks.push(String.fromCharCode(...slice));
	}
	return btoa(chunks.join(''));
}

type ExportCanvas = HTMLCanvasElement | OffscreenCanvas;

function createExportCanvas(width: number, height: number): ExportCanvas {
	if (typeof OffscreenCanvas !== 'undefined') {
		return new OffscreenCanvas(width, height);
	}
	const canvas = document.createElement('canvas');
	canvas.width = width;
	canvas.height = height;
	return canvas;
}

function get2dContext(canvas: ExportCanvas): CanvasRenderingContext2D {
	const ctx = canvas.getContext('2d', { alpha: false }) as CanvasRenderingContext2D | null;
	if (!ctx) {
		throw new Error('Failed to acquire 2D canvas context for video export');
	}
	return ctx;
}

/** Pre-renders the static field, keepouts, baseline diff, planned centerline, ghost path, and waypoints once. */
export function renderVideoStaticLayer(
	ctx: CanvasRenderingContext2D,
	context: VideoRenderContext,
	width: number,
	stageHeight: number,
): ScreenTransform {
	const viewport = computeVideoViewport(context.result, context.includeTelemetry);
	const transform = createScreenTransform(width, stageHeight, 1, viewport);

	ctx.save();
	ctx.imageSmoothingEnabled = true;
	ctx.imageSmoothingQuality = 'high';

	drawFieldBackdrop(ctx, transform, width, stageHeight, context.fieldImage, false);
	drawKeepoutPolygons(ctx, transform, context.result.keepouts);

	if (context.baselineResult && (context.diffMode || (context.result.appliedDisturbances?.length ?? 0) > 0)) {
		drawBaselineDiffOverlay(ctx, transform, context.baselineResult);
	}

	drawWaypointPolyline(ctx, transform, context.result);
	drawMathPath(ctx, transform, context.result.mathPath, context.result.mathPath.totalLengthMeters, 0.55);
	drawHeatmapTrajectory(ctx, transform, context.result.trajectory, context.result.totalDurationSeconds, 3.0, 0.22);

	for (const wp of context.result.waypoints) {
		drawWaypointGlyph(ctx, transform, wp, 1, false, 0);
	}

	ctx.restore();
	return transform;
}

/** Renders a single video frame at `timeSeconds` using the cached static background and clean telemetry HUD. */
export function renderVideoFrame(
	ctx: CanvasRenderingContext2D,
	staticLayer: CanvasImageSource,
	transform: ScreenTransform,
	context: VideoRenderContext,
	plan: VideoExportPlan,
	timeSeconds: number,
): void {
	const { result, baselineResult, diffMode, disturbances, includeTelemetry } = context;
	const t = Math.max(0, Math.min(plan.durationSeconds, timeSeconds));
	const sample = interpolateTrajectoryAtTime(result.trajectory, t);

	ctx.save();
	ctx.drawImage(staticLayer, 0, 0);

	// Clip dynamic field drawings to the field stage above the telemetry footer
	ctx.save();
	ctx.beginPath();
	ctx.rect(0, 0, plan.width, plan.stageHeight);
	ctx.clip();

	const footprint = result.robotFootprint;
	drawSweptFootprints(ctx, transform, result.trajectory, footprint, t);
	drawHeatmapTrajectory(
		ctx,
		transform,
		result.trajectory,
		t,
		4.2,
		1,
		sample ? { x: sample.x, y: sample.y, heatmapNormalized: sample.heatmapNormalized } : null,
	);

	if (result.replannedPaths && result.replannedPaths.length > 0) {
		drawReplannedPaths(ctx, transform, result.replannedPaths, t);
	}
	if (result.appliedDisturbances && result.appliedDisturbances.length > 0) {
		drawDisturbanceOverlays(ctx, transform, result.appliedDisturbances, footprint, null, t);
	}

	// Marker pulse rings when passing autonomous event markers
	const recentMarkers = findRecentMarkers(result.passedMarkers, t);
	for (const { marker, ageSeconds } of recentMarkers) {
		const progress = ageSeconds / 0.85;
		const mx = transform.toScreenX(marker.x);
		const my = transform.toScreenY(marker.y);
		ctx.save();
		ctx.strokeStyle = '#e879f9';
		ctx.globalAlpha = Math.max(0, 1 - progress);
		ctx.lineWidth = 2.2;
		ctx.beginPath();
		ctx.arc(mx, my, 8 + progress * 24, 0, Math.PI * 2);
		ctx.stroke();
		ctx.restore();
	}

	const pose = sample
		? { x: sample.x, y: sample.y, headingRadians: sample.headingRadians }
		: {
				x: result.startPose.x,
				y: result.startPose.y,
				headingRadians: result.startPose.headingRadians,
			};

	drawRobotFootprintWithIntake(ctx, transform, pose.x, pose.y, pose.headingRadians, footprint, true);
	drawCollisionRingIfNegative(ctx, transform, sample, pose, footprint);

	ctx.restore(); // end field clip

	if (includeTelemetry) {
		drawCornerTelemetryHud(ctx, result, baselineResult, diffMode, sample, recentMarkers[0] ?? null, plan.width);
		drawVideoTimelineFooter(ctx, result, disturbances, t, plan);
	}

	ctx.restore();
}

function findRecentMarkers(
	markers: readonly PassedMarkerEntry[],
	t: number,
): { marker: PassedMarkerEntry; ageSeconds: number }[] {
	const recent: { marker: PassedMarkerEntry; ageSeconds: number }[] = [];
	for (let i = markers.length - 1; i >= 0; i--) {
		const m = markers[i];
		const age = t - m.timestampSeconds;
		if (age >= -1e-6 && age <= 0.85) {
			recent.push({ marker: m, ageSeconds: Math.max(0, age) });
		}
	}
	return recent;
}

function drawRoundedRectPath(
	ctx: CanvasRenderingContext2D,
	x: number,
	y: number,
	w: number,
	h: number,
	r: number,
): void {
	const radius = Math.min(r, w * 0.5, h * 0.5);
	ctx.beginPath();
	ctx.moveTo(x + radius, y);
	ctx.lineTo(x + w - radius, y);
	ctx.quadraticCurveTo(x + w, y, x + w, y + radius);
	ctx.lineTo(x + w, y + h - radius);
	ctx.quadraticCurveTo(x + w, y + h, x + w - radius, y + h);
	ctx.lineTo(x + radius, y + h);
	ctx.quadraticCurveTo(x, y + h, x, y + h - radius);
	ctx.lineTo(x, y + radius);
	ctx.quadraticCurveTo(x, y, x + radius, y);
	ctx.closePath();
}

function drawPillSurface(ctx: CanvasRenderingContext2D, x: number, y: number, w: number, h: number): void {
	ctx.save();
	drawRoundedRectPath(ctx, x, y, w, h, 9);
	ctx.fillStyle = 'rgba(13, 17, 24, 0.84)';
	ctx.fill();
	ctx.strokeStyle = 'rgba(148, 163, 184, 0.24)';
	ctx.lineWidth = 1;
	ctx.stroke();
	ctx.restore();
}

function drawCornerTelemetryHud(
	ctx: CanvasRenderingContext2D,
	result: PreviewResult,
	baselineResult: PreviewResult | null,
	diffMode: boolean,
	sample: TrajectorySample | null,
	activeMarker: { marker: PassedMarkerEntry; ageSeconds: number } | null,
	width: number,
): void {
	const uiFont = '-apple-system, BlinkMacSystemFont, "Segoe UI", system-ui, sans-serif';
	const monoFont = 'ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace';

	// Top-left identity pill
	const scopeLabel = formatVideoScopeLabel(result);
	const titleText = `${result.autoClass} · ${scopeLabel}`;
	const robotText = result.robotConfigId;
	const hasBaselineDelta = baselineResult !== null && (diffMode || (result.appliedDisturbances?.length ?? 0) > 0);
	const deltaSec = hasBaselineDelta ? result.totalDurationSeconds - baselineResult.totalDurationSeconds : 0;
	const deltaText = hasBaselineDelta ? `Δ ${deltaSec > 1e-4 ? '+' : ''}${deltaSec.toFixed(2)}s` : '';

	ctx.save();
	ctx.font = `600 13px ${uiFont}`;
	const titleW = ctx.measureText(titleText).width;
	ctx.font = `600 12px ${monoFont}`;
	const robotW = ctx.measureText(robotText).width;
	const deltaW = deltaText ? ctx.measureText(deltaText).width + 16 : 0;

	const leftPillX = 16;
	const topPillY = 14;
	const pillH = 32;
	const leftPillW = 14 + 8 + 8 + titleW + 10 + 1 + 10 + robotW + 14;
	drawPillSurface(ctx, leftPillX, topPillY, leftPillW, pillH);

	// Alliance dot
	ctx.fillStyle = result.alliance === 'Red' ? '#f85149' : '#388bfd';
	ctx.beginPath();
	ctx.arc(leftPillX + 18, topPillY + pillH * 0.5, 4, 0, Math.PI * 2);
	ctx.fill();

	// Title text
	ctx.fillStyle = '#f1f5f9';
	ctx.font = `600 13px ${uiFont}`;
	ctx.textBaseline = 'middle';
	ctx.fillText(titleText, leftPillX + 30, topPillY + pillH * 0.5 + 0.5);

	// Divider + robot ID
	const divX = leftPillX + 30 + titleW + 10;
	ctx.fillStyle = 'rgba(148, 163, 184, 0.25)';
	ctx.fillRect(divX, topPillY + 8, 1, pillH - 16);

	ctx.fillStyle = '#94a3b8';
	ctx.font = `600 12px ${monoFont}`;
	ctx.fillText(robotText, divX + 11, topPillY + pillH * 0.5 + 0.5);

	if (deltaText) {
		const dx = leftPillX + leftPillW + 8;
		drawPillSurface(ctx, dx, topPillY, deltaW, pillH);
		ctx.fillStyle = deltaSec < -0.005 ? '#5cbf6d' : deltaSec > 0.005 ? '#fea619' : '#e879f9';
		ctx.font = `600 12px ${monoFont}`;
		ctx.fillText(deltaText, dx + 8, topPillY + pillH * 0.5 + 0.5);
	}

	// Marker toast badge below top-left pill
	if (activeMarker) {
		const fade = activeMarker.ageSeconds > 0.55 ? 1 - (activeMarker.ageSeconds - 0.55) / 0.3 : 1;
		const markerLabel = `◆ ${activeMarker.marker.marker}`;
		ctx.font = `600 11.5px ${monoFont}`;
		const mw = ctx.measureText(markerLabel).width + 18;
		const my = topPillY + pillH + 8;
		ctx.save();
		ctx.globalAlpha = Math.max(0, Math.min(1, fade));
		drawRoundedRectPath(ctx, leftPillX, my, mw, 24, 7);
		ctx.fillStyle = 'rgba(232, 121, 249, 0.16)';
		ctx.fill();
		ctx.strokeStyle = 'rgba(232, 121, 249, 0.55)';
		ctx.lineWidth = 1;
		ctx.stroke();
		ctx.fillStyle = '#f0abfc';
		ctx.fillText(markerLabel, leftPillX + 9, my + 12.5);
		ctx.restore();
	}

	// Top-right telemetry pill
	const speed = sample ? sample.achievedSpeedMetersPerSec : 0;
	const pct = sample ? Math.round(sample.slowdownRatio * 100) : 100;
	const limiter = sample ? sample.limitingFactor : result.dominantLimitingFactor;
	const speedText = `${speed.toFixed(2)} m/s`;
	const limiterText = `${limiter} · ${pct}%`;
	const totalDist = result.mathPath.totalLengthMeters;
	const distText =
		totalDist > 0.01 && sample
			? `${Math.min(totalDist, sample.progressMeters).toFixed(2)} / ${totalDist.toFixed(2)} m`
			: '';

	ctx.font = `600 13px ${monoFont}`;
	const speedW = ctx.measureText(speedText).width;
	ctx.font = `500 12px ${monoFont}`;
	const limiterW = ctx.measureText(limiterText).width;
	const distW = distText ? ctx.measureText(distText).width : 0;

	const rightPillW = 14 + 8 + 8 + speedW + 10 + 1 + 10 + limiterW + (distText ? 10 + 1 + 10 + distW : 0) + 14;
	const rightPillX = width - 16 - rightPillW;
	drawPillSurface(ctx, rightPillX, topPillY, rightPillW, pillH);

	// Heatmap speed dot
	ctx.fillStyle = sample ? heatmapColor(sample.heatmapNormalized) : '#94a3b8';
	ctx.beginPath();
	ctx.arc(rightPillX + 18, topPillY + pillH * 0.5, 4, 0, Math.PI * 2);
	ctx.fill();

	ctx.fillStyle = '#f8fafc';
	ctx.font = `600 13px ${monoFont}`;
	ctx.fillText(speedText, rightPillX + 30, topPillY + pillH * 0.5 + 0.5);

	const rDiv1 = rightPillX + 30 + speedW + 10;
	ctx.fillStyle = 'rgba(148, 163, 184, 0.25)';
	ctx.fillRect(rDiv1, topPillY + 8, 1, pillH - 16);

	ctx.fillStyle = '#cbd5e1';
	ctx.font = `500 12px ${monoFont}`;
	ctx.fillText(limiterText, rDiv1 + 11, topPillY + pillH * 0.5 + 0.5);

	if (distText) {
		const rDiv2 = rDiv1 + 11 + limiterW + 10;
		ctx.fillStyle = 'rgba(148, 163, 184, 0.25)';
		ctx.fillRect(rDiv2, topPillY + 8, 1, pillH - 16);
		ctx.fillStyle = '#94a3b8';
		ctx.fillText(distText, rDiv2 + 11, topPillY + pillH * 0.5 + 0.5);
	}

	ctx.restore();
}

function drawVideoTimelineFooter(
	ctx: CanvasRenderingContext2D,
	result: PreviewResult,
	disturbances: readonly DisturbanceEvent[],
	timeSeconds: number,
	plan: VideoExportPlan,
): void {
	const y0 = plan.stageHeight;
	const h = VIDEO_TIMELINE_HEIGHT;
	const monoFont = 'ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace';
	const totalT = Math.max(1e-6, result.totalDurationSeconds);
	const progressFrac = Math.max(0, Math.min(1, timeSeconds / totalT));

	ctx.save();
	ctx.fillStyle = '#0d1118';
	ctx.fillRect(0, y0, plan.width, h);

	ctx.fillStyle = 'rgba(148, 163, 184, 0.16)';
	ctx.fillRect(0, y0, plan.width, 1);

	const activeStr = formatVideoTime(timeSeconds);
	const totalStr = formatVideoTime(result.totalDurationSeconds);

	ctx.font = `600 13px ${monoFont}`;
	ctx.textBaseline = 'middle';
	ctx.fillStyle = '#f1f5f9';
	ctx.textAlign = 'left';
	ctx.fillText(activeStr, 18, y0 + h * 0.5 + 0.5);

	ctx.fillStyle = '#94a3b8';
	ctx.textAlign = 'right';
	ctx.fillText(totalStr, plan.width - 18, y0 + h * 0.5 + 0.5);

	// Track geometry
	const trackX = 94;
	const trackW = plan.width - 188;
	const trackH = 8;
	const trackY = y0 + (h - trackH) * 0.5;
	const playheadX = trackX + progressFrac * trackW;

	// Rounded clip for track heatmap bands
	ctx.save();
	drawRoundedRectPath(ctx, trackX, trackY, trackW, trackH, 4);
	ctx.clip();

	ctx.fillStyle = 'rgba(148, 163, 184, 0.14)';
	ctx.fillRect(trackX, trackY, trackW, trackH);

	const traj = result.trajectory;
	for (let i = 0; i < traj.length; i++) {
		const s = traj[i];
		const nextT = i + 1 < traj.length ? traj[i + 1].timestampSeconds : totalT;
		const sx0 = trackX + (s.timestampSeconds / totalT) * trackW;
		const sx1 = Math.max(sx0 + 1, trackX + (nextT / totalT) * trackW);
		ctx.fillStyle = heatmapColor(s.heatmapNormalized);
		ctx.globalAlpha = s.timestampSeconds <= timeSeconds + 1e-6 ? 0.95 : 0.28;
		ctx.fillRect(sx0, trackY, sx1 - sx0, trackH);
	}
	ctx.globalAlpha = 1;

	// Passed marker ticks
	ctx.strokeStyle = '#e879f9';
	ctx.lineWidth = 1.6;
	for (const marker of result.passedMarkers) {
		const mx = trackX + Math.max(0, Math.min(1, marker.timestampSeconds / totalT)) * trackW;
		ctx.beginPath();
		ctx.moveTo(mx, trackY);
		ctx.lineTo(mx, trackY + trackH);
		ctx.stroke();
	}
	ctx.restore();

	// Disturbance pins/spans below track
	for (const ev of disturbances) {
		if (!ev.enabled) continue;
		const sx = trackX + Math.max(0, Math.min(1, ev.timestampSeconds / totalT)) * trackW;
		if (ev.kind === 'PUSH') {
			const ex =
				trackX + Math.max(0, Math.min(1, (ev.timestampSeconds + Math.max(0.02, ev.durationSeconds)) / totalT)) * trackW;
			ctx.fillStyle = 'rgba(245, 158, 11, 0.85)';
			ctx.fillRect(sx, trackY + trackH + 3, Math.max(3, ex - sx), 3);
		} else {
			ctx.fillStyle = ev.kind === 'VISION_JUMP' ? '#38bdf8' : '#e879f9';
			ctx.beginPath();
			ctx.arc(sx, trackY + trackH + 4.5, 2.5, 0, Math.PI * 2);
			ctx.fill();
		}
	}

	// Diagnostic alert dots above track
	for (const d of result.diagnostics) {
		if (d.timestampSeconds === null || !Number.isFinite(d.timestampSeconds)) continue;
		const ax = trackX + Math.max(0, Math.min(1, d.timestampSeconds / totalT)) * trackW;
		ctx.fillStyle = d.severity === 'ERROR' ? '#f97758' : '#fea619';
		ctx.beginPath();
		ctx.arc(ax, trackY - 5, 2.8, 0, Math.PI * 2);
		ctx.fill();
	}

	// Crisp playhead dot
	ctx.fillStyle = '#0d1118';
	ctx.strokeStyle = '#f8fafc';
	ctx.lineWidth = 2.2;
	ctx.beginPath();
	ctx.arc(playheadX, trackY + trackH * 0.5, 6, 0, Math.PI * 2);
	ctx.fill();
	ctx.stroke();

	ctx.restore();
}

interface SupportedCodecChoice {
	webCodecsCodec: string;
	muxerCodec: 'avc' | 'vp9' | 'av1';
	config: VideoEncoderConfig;
}

async function collectCodecCandidates(width: number, height: number, fps: number): Promise<SupportedCodecChoice[]> {
	if (typeof VideoEncoder !== 'function' || typeof VideoEncoder.isConfigSupported !== 'function') {
		return [];
	}

	const choices: SupportedCodecChoice[] = [];
	const h264Candidates = ['avc1.640028', 'avc1.4d002a', 'avc1.42001f', 'avc1.42E01F'];
	const modernMp4Candidates: { codec: string; muxerCodec: 'vp9' | 'av1' }[] = [
		{ codec: 'vp09.00.10.08', muxerCodec: 'vp9' },
		{ codec: 'av01.0.04M.08', muxerCodec: 'av1' },
	];

	for (const codec of h264Candidates) {
		const config: VideoEncoderConfig = {
			codec,
			width,
			height,
			bitrate: 6_000_000,
			framerate: fps,
			latencyMode: 'realtime',
			avc: { format: 'avc' },
		};
		try {
			const support = await VideoEncoder.isConfigSupported(config);
			if (support.supported) {
				choices.push({ webCodecsCodec: codec, muxerCodec: 'avc', config: support.config ?? config });
				break;
			}
		} catch {
			// Try next candidate
		}
	}

	for (const candidate of modernMp4Candidates) {
		const config: VideoEncoderConfig = {
			codec: candidate.codec,
			width,
			height,
			bitrate: 5_000_000,
			framerate: fps,
			latencyMode: 'realtime',
		};
		try {
			const support = await VideoEncoder.isConfigSupported(config);
			if (support.supported) {
				choices.push({
					webCodecsCodec: candidate.codec,
					muxerCodec: candidate.muxerCodec,
					config: support.config ?? config,
				});
				break;
			}
		} catch {
			// Try next candidate
		}
	}

	return choices;
}

async function encodeFramesWithWebCodecs(
	codecChoice: SupportedCodecChoice,
	frameCanvas: ExportCanvas,
	frameCtx: CanvasRenderingContext2D,
	staticCanvas: ExportCanvas,
	transform: ScreenTransform,
	context: VideoRenderContext,
	plan: VideoExportPlan,
	startedAt: number,
): Promise<VideoExportOutcome> {
	let encoderError: Error | null = null;
	const mp4Target = new Mp4ArrayBufferTarget();
	const mp4Muxer = new Mp4Muxer({
		target: mp4Target,
		video: { codec: codecChoice.muxerCodec, width: plan.width, height: plan.height, frameRate: plan.fps },
		fastStart: 'in-memory',
	});

	const encoder = new VideoEncoder({
		output: (chunk, meta) => {
			mp4Muxer.addVideoChunk(chunk, meta);
		},
		error: (err) => {
			encoderError = err instanceof Error ? err : new Error(String(err));
		},
	});

	try {
		encoder.configure(codecChoice.config);

		const frameDurationUs = Math.round(1_000_000 / plan.fps);
		const keyFrameInterval = plan.fps * 2;

		for (let i = 0; i < plan.totalFrames; i++) {
			if (encoderError !== null) throw encoderError;
			const t = i < plan.motionFrames ? (i / (plan.motionFrames - 1)) * plan.durationSeconds : plan.durationSeconds;
			renderVideoFrame(frameCtx, staticCanvas, transform, context, plan, t);

			const frame = new VideoFrame(frameCanvas, {
				timestamp: i * frameDurationUs,
				duration: frameDurationUs,
			});
			if (encoder.encodeQueueSize > 8) {
				await waitForEncoderDrain(encoder);
			}
			encoder.encode(frame, { keyFrame: i % keyFrameInterval === 0 });
			frame.close();

			if (i > 0 && i % 30 === 0) {
				await new Promise((resolve) => setTimeout(resolve, 0));
			}
		}

		await encoder.flush();
		if (encoderError !== null) throw encoderError;
	} finally {
		if (encoder.state !== 'closed') {
			encoder.close();
		}
	}

	mp4Muxer.finalize();
	return {
		format: 'mp4',
		bytesBase64: uint8ArrayToBase64(new Uint8Array(mp4Target.buffer)),
		fps: plan.fps,
		frameCount: plan.totalFrames,
		elapsedMs: Math.round(performance.now() - startedAt),
		codec: codecChoice.webCodecsCodec,
	};
}

/**
 * Encodes the selected path using WebCodecs (`VideoEncoder`) + `mp4-muxer`.
 * Falls back to streaming JPEG frames when WebCodecs is unavailable and `onFallbackJpegFrames` is provided.
 */
export async function exportPathVideo(
	context: VideoRenderContext,
	_preferredFormat: 'mp4' | 'webm',
	onFallbackJpegFrames?: (framesBase64: string[], fps: number) => Promise<void>,
): Promise<VideoExportOutcome | null> {
	const startedAt = performance.now();
	const plan = computeVideoExportPlan(context.result.totalDurationSeconds, context.includeTelemetry);

	const staticCanvas = createExportCanvas(plan.width, plan.stageHeight);
	const staticCtx = get2dContext(staticCanvas);
	const transform = renderVideoStaticLayer(staticCtx, context, plan.width, plan.stageHeight);

	const frameCanvas = createExportCanvas(plan.width, plan.height);
	const frameCtx = get2dContext(frameCanvas);

	const candidates = await collectCodecCandidates(plan.width, plan.height, plan.fps);
	let lastCodecErr: unknown = null;
	for (const candidate of candidates) {
		try {
			return await encodeFramesWithWebCodecs(
				candidate,
				frameCanvas,
				frameCtx,
				staticCanvas,
				transform,
				context,
				plan,
				startedAt,
			);
		} catch (err) {
			lastCodecErr = err;
		}
	}

	if (!onFallbackJpegFrames) {
		throw lastCodecErr instanceof Error
			? lastCodecErr
			: new Error('WebCodecs VideoEncoder is not available in this webview');
	}

	const jpegFrames: string[] = [];
	for (let i = 0; i < plan.totalFrames; i++) {
		const t = i < plan.motionFrames ? (i / (plan.motionFrames - 1)) * plan.durationSeconds : plan.durationSeconds;
		renderVideoFrame(frameCtx, staticCanvas, transform, context, plan, t);
		const blob = await canvasToJpegBlob(frameCanvas, 0.92);
		const buf = new Uint8Array(await blob.arrayBuffer());
		jpegFrames.push(uint8ArrayToBase64(buf));
	}
	await onFallbackJpegFrames(jpegFrames, plan.fps);
	return null;
}

function waitForEncoderDrain(encoder: VideoEncoder): Promise<void> {
	return new Promise((resolve) => {
		if (encoder.encodeQueueSize <= 4) {
			resolve();
			return;
		}
		const onDequeue = () => {
			if (encoder.encodeQueueSize <= 4) {
				encoder.removeEventListener('dequeue', onDequeue);
				resolve();
			}
		};
		encoder.addEventListener('dequeue', onDequeue);
	});
}

function canvasToJpegBlob(canvas: ExportCanvas, quality: number): Promise<Blob> {
	if ('convertToBlob' in canvas && typeof canvas.convertToBlob === 'function') {
		return canvas.convertToBlob({ type: 'image/jpeg', quality });
	}
	return new Promise((resolve, reject) => {
		(canvas as HTMLCanvasElement).toBlob(
			(blob) => {
				if (blob) resolve(blob);
				else reject(new Error('Failed to encode canvas frame as JPEG'));
			},
			'image/jpeg',
			quality,
		);
	});
}
