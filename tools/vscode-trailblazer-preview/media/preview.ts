import {
	FIELD_2026_CONFIG,
	type CameraViewport,
	type DiagnosticEntry,
	type PreviewResult,
	type SegmentDescriptor,
	type WaypointEntry,
	boundsToViewport,
	computeAutoCropBounds,
	computeWaypointViewport,
	interpolateViewport,
	parsePreviewResult,
	sampleTrajectoryAtTime,
	zoomViewportAtCursor,
} from '../schema.ts';
import {
	CAMERA_TRANSITION_MS,
	EDIT_ZOOM_HOLD_MS,
	PATH_REVEAL_MS,
	type ScreenTransform,
	hitTestWaypoint,
	renderFieldCanvas,
	renderTimelineCanvas,
} from './renderer.ts';

export class TimelineController {
	public totalDurationSeconds = 0;
	public hoveredTimeSeconds: number | null = null;
	public pinnedTimeSeconds: number | null = null;
	private playing = false;
	private lastPlayTickMs = 0;

	public setTotalDuration(durationSeconds: number): void {
		this.totalDurationSeconds = Math.max(0, Number.isFinite(durationSeconds) ? durationSeconds : 0);
		if (this.pinnedTimeSeconds !== null && this.pinnedTimeSeconds > this.totalDurationSeconds) {
			this.pinnedTimeSeconds = this.totalDurationSeconds;
		}
		if (this.hoveredTimeSeconds !== null && this.hoveredTimeSeconds > this.totalDurationSeconds) {
			this.hoveredTimeSeconds = this.totalDurationSeconds;
		}
	}

	public hoverAtTime(timeSeconds: number): number {
		const clamped = this.clampTime(timeSeconds);
		this.hoveredTimeSeconds = clamped;
		return this.activeTimeSeconds();
	}

	public hoverAtFraction(fraction: number): number {
		return this.hoverAtTime(fraction * this.totalDurationSeconds);
	}

	public clearHover(): void {
		this.hoveredTimeSeconds = null;
	}

	public clickAtTime(timeSeconds: number): number | null {
		this.playing = false;
		const clamped = this.clampTime(timeSeconds);
		if (
			this.pinnedTimeSeconds !== null &&
			Math.abs(this.pinnedTimeSeconds - clamped) <= Math.max(0.03, this.totalDurationSeconds * 0.015)
		) {
			this.pinnedTimeSeconds = null;
		} else {
			this.pinnedTimeSeconds = clamped;
		}
		return this.pinnedTimeSeconds;
	}

	public clickAtFraction(fraction: number): number | null {
		return this.clickAtTime(fraction * this.totalDurationSeconds);
	}

	public unpin(): void {
		this.playing = false;
		this.pinnedTimeSeconds = null;
	}

	public isPinned(): boolean {
		return this.pinnedTimeSeconds !== null;
	}

	public isPlaying(): boolean {
		return this.playing;
	}

	/** Toggles 1.0x real-time playback, restarting from 0.00 s if currently at the end. */
	public togglePlay(nowMs: number): boolean {
		if (this.playing) {
			this.playing = false;
			return false;
		}
		if (this.totalDurationSeconds <= 0) return false;
		const startFrom =
			this.pinnedTimeSeconds !== null && this.pinnedTimeSeconds < this.totalDurationSeconds - 1e-3
				? this.pinnedTimeSeconds
				: 0;
		this.pinnedTimeSeconds = startFrom;
		this.lastPlayTickMs = nowMs;
		this.playing = true;
		return true;
	}

	/** Steps 1.0x real-time playback forward by `(nowMs - lastPlayTickMs) / 1000`. */
	public stepPlayback(nowMs: number): number {
		if (!this.playing) return this.activeTimeSeconds();
		const dtSec = Math.max(0, (nowMs - this.lastPlayTickMs) / 1000);
		this.lastPlayTickMs = nowMs;
		const nextT = (this.pinnedTimeSeconds ?? 0) + dtSec;
		if (nextT >= this.totalDurationSeconds) {
			this.pinnedTimeSeconds = this.totalDurationSeconds;
			this.playing = false;
		} else {
			this.pinnedTimeSeconds = nextT;
		}
		return this.activeTimeSeconds();
	}

	public activeTimeSeconds(): number {
		return this.pinnedTimeSeconds ?? this.hoveredTimeSeconds ?? this.totalDurationSeconds;
	}

	private clampTime(t: number): number {
		if (!Number.isFinite(t)) return 0;
		return Math.max(0, Math.min(this.totalDurationSeconds, t));
	}
}

export class CameraAnimator {
	private current: CameraViewport;
	private startViewport: CameraViewport;
	private targetViewport: CameraViewport;
	private transitionStartMs = 0;
	private transitioning = false;

	public constructor(initial?: CameraViewport) {
		const fallback: CameraViewport = initial ?? {
			centerX: FIELD_2026_CONFIG.fieldLengthMeters * 0.5,
			centerY: FIELD_2026_CONFIG.fieldWidthMeters * 0.5,
			spanX: FIELD_2026_CONFIG.fieldLengthMeters + 2.0,
			spanY: FIELD_2026_CONFIG.fieldWidthMeters + 2.0,
		};
		this.current = { ...fallback };
		this.startViewport = { ...fallback };
		this.targetViewport = { ...fallback };
	}

	public snapTo(viewport: CameraViewport): void {
		this.current = { ...viewport };
		this.startViewport = { ...viewport };
		this.targetViewport = { ...viewport };
		this.transitioning = false;
	}

	public animateTo(viewport: CameraViewport, nowMs: number): void {
		const cur = this.sample(nowMs);
		this.startViewport = { ...cur };
		this.targetViewport = { ...viewport };
		this.transitionStartMs = nowMs;
		this.transitioning = true;
	}

	public zoomAtCursor(
		widthCssPx: number,
		heightCssPx: number,
		cursorCssX: number,
		cursorCssY: number,
		zoomFactor: number,
		nowMs: number,
	): CameraViewport {
		const cur = this.sample(nowMs);
		const zoomed = zoomViewportAtCursor(cur, widthCssPx, heightCssPx, cursorCssX, cursorCssY, zoomFactor);
		this.snapTo(zoomed);
		return zoomed;
	}

	public sample(nowMs: number): CameraViewport {
		if (!this.transitioning) return this.current;
		const elapsed = Math.max(0, nowMs - this.transitionStartMs);
		if (elapsed >= CAMERA_TRANSITION_MS) {
			this.current = { ...this.targetViewport };
			this.transitioning = false;
			return this.current;
		}
		this.current = interpolateViewport(this.startViewport, this.targetViewport, elapsed / CAMERA_TRANSITION_MS);
		return this.current;
	}

	public isTransitioning(): boolean {
		return this.transitioning;
	}

	public getTargetViewport(): CameraViewport {
		return this.transitioning ? this.targetViewport : this.current;
	}
}

/** Resolves a unique source match in the active segment or across the whole auto. */
export function findWaypointForSourceLine(
	waypoints: readonly WaypointEntry[],
	line: number,
	segmentId?: string | null,
): WaypointEntry | null {
	let match: WaypointEntry | null = null;
	for (const waypoint of waypoints) {
		if (segmentId && waypoint.segmentId !== segmentId) continue;
		const start = waypoint.sourceLine;
		if (start === null || line < start || line > (waypoint.sourceEndLine ?? start)) continue;
		if (match) return null;
		match = waypoint;
	}
	return match;
}

interface SavedWebviewState {
	pinnedWholeAuto?: boolean;
	diffMode?: boolean;
	alliance?: 'Blue' | 'Red';
	robotConfigId?: '581' | '9991' | '9981';
}

declare function acquireVsCodeApi(): {
	getState: () => SavedWebviewState | undefined;
	setState: (state: SavedWebviewState) => void;
	postMessage: (message: Record<string, unknown>) => void;
};

function bootWebview() {
	if (typeof document === 'undefined' || typeof window === 'undefined') return;
	const fieldCanvas = document.querySelector<HTMLCanvasElement>('#field-canvas');
	const timelineCanvas = document.querySelector<HTMLCanvasElement>('#timeline-canvas');
	if (!fieldCanvas || !timelineCanvas) return;

	const vscode = typeof acquireVsCodeApi === 'function' ? acquireVsCodeApi() : null;
	const saved = vscode?.getState();

	let currentResult: PreviewResult | null = null;
	let baselineResult: PreviewResult | null = null;
	let segments: SegmentDescriptor[] = [];
	let diagnostics: DiagnosticEntry[] = [];
	let stale = false;
	let pinnedWholeAuto = Boolean(saved?.pinnedWholeAuto);
	let diffMode = Boolean(saved?.diffMode);
	let selectedSegmentId: string | null = null;
	let activeWaypointIndex: number | null = null;
	let alliance: 'Blue' | 'Red' = saved?.alliance ?? 'Blue';
	let robotConfigId: '581' | '9991' | '9981' = saved?.robotConfigId ?? '581';

	const timeline = new TimelineController();
	const camera = new CameraAnimator();
	let lastTransform: ScreenTransform | null = null;
	let hasInitializedCamera = false;
	let lastScopeKey = '';
	let revealStartMs = 0;
	let revealActive = false;
	let editZoomBackTimer: ReturnType<typeof setTimeout> | null = null;
	let pointFocus: WaypointEntry | null = null;
	let editedWaypointLine: number | null = null;
	let refitWhenRefreshed = false;
	let sourceRevision = 0;
	let resultRevision = -1;
	let rafId = 0;

	const fieldImage = new Image();
	const fieldImageUrl = document.body.dataset.fieldImage;
	if (fieldImageUrl) {
		fieldImage.onload = () => scheduleFrame();
		fieldImage.src = fieldImageUrl;
	}

	const el = <T extends Element>(sel: string) => document.querySelector<T>(sel)!;
	const diagStrip = el<HTMLDivElement>('#diagnostics-strip');
	const timelineAlerts = el<HTMLDivElement>('#timeline-alerts');
	const segGroup = el<HTMLDivElement>('#segment-pills');
	const modeCursorBtn = el<HTMLButtonElement>('#mode-cursor');
	const modeWholeBtn = el<HTMLButtonElement>('#mode-whole');
	const diffBtn = el<HTMLButtonElement>('#btn-diff');
	const fitBtn = el<HTMLButtonElement>('#btn-fit');
	const playBtn = el<HTMLButtonElement>('#btn-play');
	const iconPlay = el<SVGElement>('#icon-play');
	const iconPause = el<SVGElement>('#icon-pause');
	const optionsBtn = el<HTMLButtonElement>('#btn-options');
	const optionsPopover = el<HTMLDivElement>('#options-popover');
	const staleBadge = el<HTMLSpanElement>('#stale-badge');
	const diffDeltaBadge = el<HTMLSpanElement>('#diff-delta-badge');
	const slowdownBadge = el<HTMLSpanElement>('#slowdown-badge');
	const speedReadout = el<HTMLSpanElement>('#speed-readout');
	const durationReadout = el<HTMLSpanElement>('#duration-readout');
	const timelineTrack = el<HTMLElement>('#timeline-track');

	function setPlayButtonState(playing: boolean) {
		playBtn.setAttribute('aria-pressed', String(playing));
		playBtn.setAttribute('aria-label', playing ? 'Pause' : 'Play');
		iconPlay.toggleAttribute('hidden', playing);
		iconPause.toggleAttribute('hidden', !playing);
	}

	function setOptionsPopoverOpen(open: boolean) {
		optionsPopover.hidden = !open;
		optionsBtn.setAttribute('aria-expanded', String(open));
	}

	function updateFitButtonState() {
		if (!currentResult) {
			fitBtn.disabled = true;
			return;
		}
		const fitVp = boundsToViewport(computeAutoCropBounds(currentResult, 1.0));
		const targetVp = camera.getTargetViewport();
		const atFit =
			Math.abs(targetVp.centerX - fitVp.centerX) < 1e-3 &&
			Math.abs(targetVp.centerY - fitVp.centerY) < 1e-3 &&
			Math.abs(targetVp.spanX - fitVp.spanX) < 1e-3 &&
			Math.abs(targetVp.spanY - fitVp.spanY) < 1e-3;
		fitBtn.disabled = atFit;
	}

	function persistState() {
		vscode?.setState({ pinnedWholeAuto, diffMode, alliance, robotConfigId });
	}

	function updateControlsUI() {
		modeCursorBtn.setAttribute('aria-pressed', String(!pinnedWholeAuto && selectedSegmentId === null));
		modeWholeBtn.setAttribute('aria-pressed', String(pinnedWholeAuto));
		diffBtn.setAttribute('aria-pressed', String(diffMode));
		setPlayButtonState(timeline.isPlaying());
		updateFitButtonState();

		for (const btn of document.querySelectorAll<HTMLButtonElement>('[data-alliance]')) {
			btn.setAttribute('aria-pressed', String(btn.dataset.alliance === alliance));
		}
		for (const btn of document.querySelectorAll<HTMLButtonElement>('[data-robot]')) {
			btn.setAttribute('aria-pressed', String(btn.dataset.robot === robotConfigId));
		}

		staleBadge.hidden = !stale;

		if (diffMode && currentResult && baselineResult) {
			const delta = currentResult.totalDurationSeconds - baselineResult.totalDurationSeconds;
			const sign = delta > 1e-4 ? '+' : '';
			diffDeltaBadge.textContent = `Δ ${sign}${delta.toFixed(2)} s`;
			diffDeltaBadge.dataset.faster = String(delta < -0.005);
			diffDeltaBadge.dataset.slower = String(delta > 0.005);
			diffDeltaBadge.hidden = false;
		} else {
			diffDeltaBadge.hidden = true;
		}

		segGroup.replaceChildren();
		const activeSegId = currentResult?.scope === 'SEGMENT' ? currentResult.activeSegmentId : selectedSegmentId;
		for (const seg of segments) {
			const btn = document.createElement('button');
			btn.type = 'button';
			btn.textContent = seg.id;
			btn.setAttribute('aria-pressed', String(!pinnedWholeAuto && activeSegId === seg.id));
			btn.onclick = () => {
				pinnedWholeAuto = false;
				selectedSegmentId = seg.id;
				updateControlsUI();
				persistState();
				vscode?.postMessage({ type: 'setMode', pinnedWholeAuto: false, segmentId: seg.id });
			};
			segGroup.append(btn);
		}

		diagStrip.replaceChildren();
		timelineAlerts.replaceChildren();
		const totalDuration = currentResult?.totalDurationSeconds ?? 0;
		for (const d of diagnostics) {
			const targetSuffix = d.targetGoal
				? ` · ${d.targetGoal}`
				: d.line
					? ` · L${d.line}`
					: d.timestampSeconds !== null
						? ` · ${d.timestampSeconds.toFixed(2)}s`
						: '';
			const onSelectDiag = () => {
				if (d.timestampSeconds !== null && Number.isFinite(d.timestampSeconds)) {
					timeline.clickAtTime(d.timestampSeconds);
					updateTelemetryUI();
					scheduleFrame();
				}
				if (d.line && d.line > 0) {
					vscode?.postMessage({ type: 'revealLine', line: d.line, sourceRevision });
				}
			};

			if (d.timestampSeconds !== null && Number.isFinite(d.timestampSeconds) && totalDuration > 0) {
				const dot = document.createElement('button');
				dot.type = 'button';
				dot.className = 'timeline-alert-dot';
				dot.dataset.severity = d.severity;
				const pct = Math.max(0, Math.min(1, d.timestampSeconds / totalDuration)) * 100;
				dot.style.left = `${pct.toFixed(3)}%`;
				const label = `${d.code}${targetSuffix}${d.message ? ` — ${d.message}` : ''}`;
				dot.title = label;
				dot.setAttribute('aria-label', label);
				dot.onclick = onSelectDiag;
				timelineAlerts.append(dot);
			} else {
				const pill = document.createElement('button');
				pill.type = 'button';
				pill.className = 'diag-pill';
				pill.dataset.severity = d.severity;
				pill.textContent = `${d.code}${targetSuffix}`;
				if (d.message) pill.title = d.message;
				pill.onclick = onSelectDiag;
				diagStrip.append(pill);
			}
		}
	}

	function updateTelemetryUI() {
		setPlayButtonState(timeline.isPlaying());

		if (!currentResult) {
			durationReadout.textContent = '0.00/0.00s';
			speedReadout.textContent = '0.00 m/s';
			slowdownBadge.textContent = 'NONE · 100%';
			return;
		}
		const total = currentResult.totalDurationSeconds;
		const activeT = timeline.activeTimeSeconds();
		const sample = sampleTrajectoryAtTime(currentResult.trajectory, activeT);

		const isScrubbing = timeline.isPlaying() || timeline.isPinned() || timeline.hoveredTimeSeconds !== null;
		durationReadout.textContent = `${activeT.toFixed(2)}/${total.toFixed(2)}s`;
		timelineTrack.setAttribute('aria-valuemax', total.toFixed(2));
		timelineTrack.setAttribute('aria-valuenow', activeT.toFixed(2));

		if (sample && isScrubbing) {
			speedReadout.textContent = `${sample.achievedSpeedMetersPerSec.toFixed(2)} m/s`;
			const pct = Math.round(sample.slowdownRatio * 100);
			slowdownBadge.textContent = `${sample.limitingFactor} · ${pct}%`;
		} else {
			const peakSpeed = currentResult.trajectory.reduce((acc, s) => Math.max(acc, s.achievedSpeedMetersPerSec), 0);
			speedReadout.textContent = `${peakSpeed.toFixed(2)} m/s`;
			const minPct = Math.round(currentResult.minSlowdownRatio * 100);
			slowdownBadge.textContent = `${currentResult.dominantLimitingFactor} · ${minPct}%`;
		}
	}

	function renderNow(nowMs: number) {
		rafId = 0;
		if (timeline.isPlaying()) {
			timeline.stepPlayback(nowMs);
			updateTelemetryUI();
		}

		const vp = camera.sample(nowMs);
		let revealProgress = 1;
		if (revealActive) {
			const elapsed = Math.max(0, nowMs - revealStartMs);
			if (elapsed >= PATH_REVEAL_MS) {
				revealProgress = 1;
				revealActive = false;
			} else {
				revealProgress = elapsed / PATH_REVEAL_MS;
			}
		}

		lastTransform = renderFieldCanvas(fieldCanvas!, {
			result: currentResult,
			baselineResult,
			diffMode,
			viewport: vp,
			revealProgress,
			activeTimeSeconds: timeline.activeTimeSeconds(),
			activeWaypointIndex,
			animationClockMs: nowMs,
			fieldImage,
			stale,
		});
		renderTimelineCanvas(timelineCanvas!, {
			result: currentResult,
			hoveredTimeSeconds: timeline.hoveredTimeSeconds,
			pinnedTimeSeconds: timeline.pinnedTimeSeconds,
		});

		if (camera.isTransitioning() || revealActive || timeline.isPlaying() || activeWaypointIndex !== null) {
			scheduleFrame();
		}
	}

	function scheduleFrame() {
		if (rafId !== 0) return;
		rafId = window.requestAnimationFrame(renderNow);
	}

	function clearPointFocus() {
		if (editZoomBackTimer !== null) clearTimeout(editZoomBackTimer);
		editZoomBackTimer = null;
		pointFocus = null;
		editedWaypointLine = null;
		refitWhenRefreshed = false;
	}

	function focusWaypoint(waypoint: WaypointEntry) {
		pointFocus = waypoint;
		camera.animateTo(computeWaypointViewport(waypoint), performance.now());
		updateFitButtonState();
	}

	function scheduleEditZoomBack() {
		if (editZoomBackTimer !== null) clearTimeout(editZoomBackTimer);
		editZoomBackTimer = setTimeout(() => {
			const wasFocused = pointFocus !== null;
			clearPointFocus();
			if (!wasFocused || !currentResult) return;
			refitWhenRefreshed = resultRevision !== sourceRevision;
			const fullVp = boundsToViewport(computeAutoCropBounds(currentResult, 1.0));
			camera.animateTo(fullVp, performance.now());
			updateFitButtonState();
			scheduleFrame();
		}, EDIT_ZOOM_HOLD_MS);
	}

	// Control event listeners
	modeCursorBtn.onclick = () => {
		const isCursorActive = !pinnedWholeAuto && selectedSegmentId === null;
		if (isCursorActive) {
			if (currentResult?.scope === 'SEGMENT' && currentResult.activeSegmentId) {
				pinnedWholeAuto = false;
				selectedSegmentId = currentResult.activeSegmentId;
			} else {
				pinnedWholeAuto = true;
				selectedSegmentId = null;
			}
		} else {
			pinnedWholeAuto = false;
			selectedSegmentId = null;
		}
		updateControlsUI();
		persistState();
		vscode?.postMessage({ type: 'setMode', pinnedWholeAuto, segmentId: selectedSegmentId });
	};
	modeWholeBtn.onclick = () => {
		pinnedWholeAuto = true;
		selectedSegmentId = null;
		updateControlsUI();
		persistState();
		vscode?.postMessage({ type: 'setMode', pinnedWholeAuto: true, segmentId: null });
	};
	diffBtn.onclick = () => {
		diffMode = !diffMode;
		updateControlsUI();
		persistState();
		vscode?.postMessage({ type: 'setDiffMode', diffMode });
	};
	playBtn.onclick = () => {
		timeline.togglePlay(performance.now());
		updateTelemetryUI();
		scheduleFrame();
	};
	for (const btn of document.querySelectorAll<HTMLButtonElement>('[data-alliance]')) {
		btn.onclick = () => {
			const next = btn.dataset.alliance as 'Blue' | 'Red';
			alliance = next;
			updateControlsUI();
			persistState();
			vscode?.postMessage({ type: 'setAlliance', alliance: next });
		};
	}
	for (const btn of document.querySelectorAll<HTMLButtonElement>('[data-robot]')) {
		btn.onclick = () => {
			const next = btn.dataset.robot as '581' | '9991' | '9981';
			robotConfigId = next;
			updateControlsUI();
			persistState();
			vscode?.postMessage({ type: 'setRobot', robotConfigId: next });
		};
	}
	fitBtn.onclick = () => {
		if (!currentResult) return;
		clearPointFocus();
		const targetVp = boundsToViewport(computeAutoCropBounds(currentResult, 1.0));
		camera.animateTo(targetVp, performance.now());
		updateFitButtonState();
		scheduleFrame();
	};
	el<HTMLButtonElement>('#btn-refresh').onclick = () => {
		vscode?.postMessage({ type: 'refresh' });
	};
	optionsBtn.onclick = (ev: MouseEvent) => {
		ev.stopPropagation();
		setOptionsPopoverOpen(Boolean(optionsPopover.hidden));
	};
	document.addEventListener('click', (ev: MouseEvent) => {
		if (optionsPopover.hidden) return;
		const target = ev.target as Node | null;
		if (target && !optionsPopover.contains(target) && !optionsBtn.contains(target)) {
			setOptionsPopoverOpen(false);
		}
	});

	// Click on waypoint on field canvas -> bring cursor to it + zoom into that point
	fieldCanvas.addEventListener('click', (ev: MouseEvent) => {
		if (!currentResult || !lastTransform) return;
		const rect = fieldCanvas.getBoundingClientRect();
		const clickX = ev.clientX - rect.left;
		const clickY = ev.clientY - rect.top;
		const hit = hitTestWaypoint(currentResult, lastTransform, clickX, clickY);
		if (hit) {
			activeWaypointIndex = hit.index;
			clearPointFocus();
			focusWaypoint(hit);
			scheduleEditZoomBack();
			if (resultRevision === sourceRevision && hit.sourceLine && hit.sourceLine > 0) {
				vscode?.postMessage({ type: 'revealLine', line: hit.sourceLine, sourceRevision });
			}
			scheduleFrame();
		} else {
			activeWaypointIndex = null;
			scheduleFrame();
		}
	});

	// Scroll wheel on field canvas -> zoom to cursor position
	fieldCanvas.addEventListener(
		'wheel',
		(ev: WheelEvent) => {
			ev.preventDefault();
			clearPointFocus();
			const rect = fieldCanvas.getBoundingClientRect();
			const cursorX = ev.clientX - rect.left;
			const cursorY = ev.clientY - rect.top;
			const zoomFactor = Math.exp(ev.deltaY * 0.0015);
			camera.zoomAtCursor(
				rect.width || fieldCanvas.width,
				rect.height || fieldCanvas.height,
				cursorX,
				cursorY,
				zoomFactor,
				performance.now(),
			);
			updateFitButtonState();
			scheduleFrame();
		},
		{ passive: false },
	);

	// Hover-and-click timeline interaction (with horizontal padding away from panel edges)
	const fractionFromPointer = (ev: MouseEvent | PointerEvent) => {
		const rect = timelineCanvas.getBoundingClientRect();
		if (rect.width <= 0) return 0;
		return Math.max(0, Math.min(1, (ev.clientX - rect.left) / rect.width));
	};

	timelineTrack.addEventListener('pointermove', (ev: PointerEvent) => {
		timeline.hoverAtFraction(fractionFromPointer(ev));
		updateTelemetryUI();
		scheduleFrame();
	});
	timelineTrack.addEventListener('pointerleave', () => {
		timeline.clearHover();
		updateTelemetryUI();
		scheduleFrame();
	});
	timelineTrack.addEventListener('click', (ev: MouseEvent) => {
		timeline.clickAtFraction(fractionFromPointer(ev));
		updateTelemetryUI();
		scheduleFrame();
	});
	window.addEventListener('keydown', (ev: KeyboardEvent) => {
		if (ev.key === 'Escape') {
			if (!optionsPopover.hidden) {
				setOptionsPopoverOpen(false);
				return;
			}
			if (timeline.isPinned()) {
				timeline.unpin();
				updateTelemetryUI();
				scheduleFrame();
			}
		}
	});
	window.addEventListener('resize', () => scheduleFrame());

	window.addEventListener(
		'message',
		(
			event: MessageEvent<{
				type?: string;
				result?: unknown;
				baselineResult?: unknown;
				diffMode?: boolean;
				stale?: boolean;
				pinnedWholeAuto?: boolean;
				cursorLine?: number;
				editedWaypointLine?: number | null;
				sourceRevision?: number;
				segmentId?: string | null;
				diagnostics?: DiagnosticEntry[];
				segments?: SegmentDescriptor[];
				lastResult?: unknown;
			}>,
		) => {
			const msg = event.data;
			if (!msg) return;

			if (msg.type === 'sourceChanged' && typeof msg.sourceRevision === 'number') {
				clearPointFocus();
				sourceRevision = msg.sourceRevision;
				currentResult = baselineResult = null;
				activeWaypointIndex = null;
				hasInitializedCamera = false;
				selectedSegmentId = null;
				segments = [];
				diagnostics = [];
				stale = false;
				updateControlsUI();
				updateTelemetryUI();
				scheduleFrame();
				return;
			}

			if (msg.type === 'sourceEdited' && typeof msg.sourceRevision === 'number') {
				const canUsePreviousGeometry = resultRevision === sourceRevision;
				sourceRevision = msg.sourceRevision;
				editedWaypointLine = msg.editedWaypointLine ?? null;
				const edited =
					canUsePreviousGeometry && currentResult && editedWaypointLine !== null
						? findWaypointForSourceLine(currentResult.waypoints, editedWaypointLine)
						: null;
				if (edited) {
					activeWaypointIndex = edited.index;
					focusWaypoint(edited);
					scheduleFrame();
				}
				scheduleEditZoomBack();
				return;
			}

			if (msg.type === 'cursorLineChanged' && typeof msg.cursorLine === 'number') {
				if (pinnedWholeAuto || selectedSegmentId !== null) return;
				const matchingScope =
					currentResult?.scope === 'WHOLE_AUTO'
						? msg.segmentId == null
						: currentResult?.activeSegmentId === msg.segmentId;
				const matched =
					currentResult && matchingScope && resultRevision === msg.sourceRevision
						? findWaypointForSourceLine(currentResult.waypoints, msg.cursorLine)
						: null;
				activeWaypointIndex = matched?.index ?? null;
				scheduleFrame();
				return;
			}

			if (
				msg.type === 'previewResult' &&
				msg.result &&
				typeof msg.sourceRevision === 'number' &&
				msg.sourceRevision >= sourceRevision
			) {
				sourceRevision = resultRevision = msg.sourceRevision;
				const parsed = parsePreviewResult(msg.result);
				currentResult = parsed;
				baselineResult = msg.baselineResult ? parsePreviewResult(msg.baselineResult) : null;
				if (typeof msg.diffMode === 'boolean') {
					diffMode = msg.diffMode;
				}
				segments = parsed.segments;
				diagnostics = msg.diagnostics ?? parsed.diagnostics;
				stale = Boolean(msg.stale);
				if (typeof msg.pinnedWholeAuto === 'boolean') {
					pinnedWholeAuto = msg.pinnedWholeAuto;
				}
				alliance = parsed.alliance === 'Red' ? 'Red' : 'Blue';
				if (parsed.robotConfigId === '581' || parsed.robotConfigId === '9991' || parsed.robotConfigId === '9981') {
					robotConfigId = parsed.robotConfigId;
				}
				timeline.setTotalDuration(parsed.totalDurationSeconds);
				timeline.unpin();

				const now = performance.now();
				const scopeKey = `${parsed.autoClass}::${parsed.scope}::${parsed.activeSegmentId}::${parsed.alliance}`;
				const scopeChanged = scopeKey !== lastScopeKey;
				lastScopeKey = scopeKey;

				const selected =
					typeof msg.cursorLine === 'number' ? findWaypointForSourceLine(parsed.waypoints, msg.cursorLine) : null;
				activeWaypointIndex = selected?.index ?? null;
				const targetVp = boundsToViewport(computeAutoCropBounds(parsed, 1.0));
				const firstResult = !hasInitializedCamera;
				if (firstResult) {
					camera.snapTo(targetVp);
					hasInitializedCamera = true;
				}
				if ((scopeChanged || refitWhenRefreshed) && !firstResult) {
					pointFocus = null;
					refitWhenRefreshed = false;
					camera.animateTo(targetVp, now);
				}

				if (editZoomBackTimer !== null) {
					const existingFocus = pointFocus;
					const edited =
						editedWaypointLine !== null ? findWaypointForSourceLine(parsed.waypoints, editedWaypointLine) : null;
					const updatedFocus =
						edited ??
						(existingFocus
							? parsed.waypoints.find((wp) => wp.segmentId === existingFocus.segmentId && wp.id === existingFocus.id)
							: null);
					if (updatedFocus) focusWaypoint(updatedFocus);
				}

				if (scopeChanged || firstResult) {
					revealStartMs = now;
					revealActive = true;
				}

				updateControlsUI();
				updateTelemetryUI();
				scheduleFrame();
			} else if (
				msg.type === 'previewStale' &&
				typeof msg.sourceRevision === 'number' &&
				msg.sourceRevision >= sourceRevision
			) {
				sourceRevision = msg.sourceRevision;
				stale = true;
				if (msg.diagnostics) diagnostics = msg.diagnostics;
				if (msg.segments) segments = msg.segments;
				if (!currentResult && msg.lastResult) {
					try {
						currentResult = parsePreviewResult(msg.lastResult);
					} catch {
						// ignore
					}
				}
				updateControlsUI();
				updateTelemetryUI();
				scheduleFrame();
			}
		},
	);

	updateControlsUI();
	updateTelemetryUI();
	scheduleFrame();
	vscode?.postMessage({ type: 'ready' });
}

bootWebview();
