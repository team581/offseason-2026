const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const Module = require('node:module');
const { runInNewContext } = require('node:vm');

const { interpolateTrajectoryAtTime } = require('../schema.ts');
const {
	VIDEO_HEIGHT,
	VIDEO_TIMELINE_HEIGHT,
	VIDEO_WIDTH,
	buildSuggestedVideoFilename,
	computeVideoExportPlan,
	computeVideoViewport,
	exportPathVideo,
	formatVideoScopeLabel,
	formatVideoTime,
	renderVideoFrame,
	renderVideoStaticLayer,
	uint8ArrayToBase64,
} = require('../media/video-export.ts');

function makeSampleResult(overrides = {}) {
	return {
		schemaVersion: 1,
		autoClass: 'RightTrenchAuto',
		scope: 'SEGMENT',
		activeSegmentId: 'CYCLE_1_SEGMENT',
		segments: [
			{
				id: 'CYCLE_1_SEGMENT',
				fieldName: 'CYCLE_1_SEGMENT',
				index: 0,
				declarationStartLine: 10,
				declarationEndLine: 15,
				relatedSpans: [],
			},
		],
		robotConfigId: '581',
		alliance: 'Blue',
		seed: 581,
		totalDurationSeconds: 4.0,
		phaseTimings: { compileMs: 1, planMs: 1, simMs: 1, totalMs: 3 },
		buildIdentity: { sourceHash: 'a', classpathHash: 'b', compiledAtEpochMs: 0 },
		robotFootprint: {
			wheelBaseMeters: 0.5588,
			trackWidthMeters: 0.5969,
			halfLengthMeters: 0.3556,
			halfWidthMeters: 0.37465,
			bumperLengthMeters: 0.7112,
			bumperWidthMeters: 0.7493,
			intakeExtensionMeters: 0.29972,
			frontIntakeExtentMeters: 0.65532,
		},
		startPose: { x: 2.0, y: 2.0, headingRadians: 0, headingDegrees: 0 },
		waypoints: [
			{
				index: 0,
				segmentId: 'CYCLE_1_SEGMENT',
				segmentIndex: 0,
				id: 'goal-0',
				passType: 'STOP',
				x: 4.0,
				y: 3.0,
				withinMeters: 0.2,
				hasHeadingTarget: true,
				targetHeadingRadians: 0.5,
				targetHeadingDegrees: 28.6,
				headingToleranceRadians: 0.2,
				headingToleranceDegrees: 11.4,
				explicitHeadingTolerance: true,
				gatePolicy: 'SOFT',
				turnDirection: 'SHORTEST',
				anchorOffsetMeters: 0,
				marker: 'INTAKE',
				closestPassMeters: 0.05,
				sourceLine: 12,
				sourceEndLine: 12,
			},
		],
		mathPath: {
			totalLengthMeters: 2.5,
			pieces: [],
			anchors: [],
			sampledPoses: [
				{ segmentId: 'CYCLE_1_SEGMENT', s: 0, x: 2, y: 2, headingRadians: 0, headingDegrees: 0, curvature: 0 },
				{ segmentId: 'CYCLE_1_SEGMENT', s: 2.5, x: 4, y: 3, headingRadians: 0.5, headingDegrees: 28.6, curvature: 0 },
			],
		},
		keepouts: { marginMeters: 0.1, hardPolygons: [], marginPolygons: [] },
		trajectory: [
			{
				timestampSeconds: 0,
				segmentId: 'CYCLE_1_SEGMENT',
				segmentIndex: 0,
				x: 2.0,
				y: 2.0,
				headingRadians: Math.PI - 0.1,
				headingDegrees: 174.27,
				vxMetersPerSec: 1.0,
				vyMetersPerSec: 0,
				omegaRadiansPerSec: 0,
				commandedSpeedMetersPerSec: 1.0,
				achievedSpeedMetersPerSec: 1.0,
				maxSpeedMetersPerSec: 4.0,
				slowdownRatio: 0.5,
				paceSecondsPerMeter: 1.0,
				heatmapNormalized: 0.25,
				progressMeters: 0.0,
				limitingFactor: 'CURVATURE',
				status: 'OK',
				diagnostic: '',
			},
			{
				timestampSeconds: 4.0,
				segmentId: 'CYCLE_1_SEGMENT',
				segmentIndex: 0,
				x: 4.0,
				y: 3.0,
				headingRadians: -Math.PI + 0.1,
				headingDegrees: -174.27,
				vxMetersPerSec: 3.0,
				vyMetersPerSec: 0,
				omegaRadiansPerSec: 0,
				commandedSpeedMetersPerSec: 3.0,
				achievedSpeedMetersPerSec: 3.0,
				maxSpeedMetersPerSec: 4.0,
				slowdownRatio: 1.0,
				paceSecondsPerMeter: 0.33,
				heatmapNormalized: 0.75,
				progressMeters: 2.5,
				limitingFactor: 'TOP_SPEED',
				status: 'OK',
				diagnostic: '',
			},
		],
		passedMarkers: [
			{
				marker: 'INTAKE',
				segmentId: 'CYCLE_1_SEGMENT',
				timestampSeconds: 1.5,
				progressMeters: 1.0,
				x: 2.75,
				y: 2.375,
			},
		],
		headingArrivals: [],
		dominantLimitingFactor: 'TOP_SPEED',
		minSlowdownRatio: 0.5,
		diagnostics: [
			{
				severity: 'WARNING',
				code: 'NEAR_REVERSAL_CORNER',
				message: 'Sharp corner',
				segmentId: 'CYCLE_1_SEGMENT',
				targetGoal: null,
				timestampSeconds: 2.0,
				line: 12,
			},
		],
		...overrides,
	};
}

function makeMockCanvasContext() {
	const drawnTexts = [];
	const ctx = {
		fillStyle: '',
		strokeStyle: '',
		lineWidth: 1,
		lineCap: 'butt',
		lineJoin: 'miter',
		globalAlpha: 1,
		font: '',
		textAlign: 'left',
		textBaseline: 'alphabetic',
		imageSmoothingEnabled: true,
		imageSmoothingQuality: 'low',
		save() {},
		restore() {},
		setTransform() {},
		translate() {},
		scale() {},
		beginPath() {},
		closePath() {},
		moveTo() {},
		lineTo() {},
		quadraticCurveTo() {},
		arc() {},
		rect() {},
		clip() {},
		fill() {},
		stroke() {},
		fillRect() {},
		strokeRect() {},
		clearRect() {},
		drawImage() {},
		setLineDash() {},
		measureText: (txt) => ({ width: String(txt).length * 7 }),
		fillText(text, x, y) {
			drawnTexts.push({ text: String(text), x, y });
		},
	};
	return { ctx, drawnTexts };
}

void test('interpolateTrajectoryAtTime interpolates smoothly, wraps angles, and snaps across vision jumps', () => {
	assert.equal(interpolateTrajectoryAtTime([], 1.0), null);

	const result = makeSampleResult();
	const start = interpolateTrajectoryAtTime(result.trajectory, -1);
	assert.equal(start?.x, 2.0);
	const end = interpolateTrajectoryAtTime(result.trajectory, 9);
	assert.equal(end?.x, 4.0);

	// Midpoint between +3.0416 rad and -3.0416 rad wraps through +/- PI rather than 0
	const mid = interpolateTrajectoryAtTime(result.trajectory, 2.0);
	assert.ok(mid);
	assert.ok(Math.abs(mid.x - 3.0) < 1e-6);
	assert.ok(Math.abs(mid.y - 2.5) < 1e-6);
	assert.ok(Math.abs(mid.achievedSpeedMetersPerSec - 2.0) < 1e-6);
	assert.ok(Math.abs(mid.progressMeters - 1.25) < 1e-6);
	assert.ok(Math.abs(Math.abs(mid.headingRadians) - Math.PI) < 1e-4);

	// Discontinuous vision jump (dt <= 0.05 && dist > 0.25) snaps rather than sliding
	const jumpTraj = [
		{ ...result.trajectory[0], timestampSeconds: 1.0, x: 2.0, y: 2.0 },
		{ ...result.trajectory[1], timestampSeconds: 1.02, x: 3.0, y: 2.0 },
	];
	assert.equal(interpolateTrajectoryAtTime(jumpTraj, 1.005)?.x, 2.0);
	assert.equal(interpolateTrajectoryAtTime(jumpTraj, 1.015)?.x, 3.0);
});

void test('video export helpers compute fast plans, viewports, filenames, and render clean telemetry HUD', () => {
	const shortPlan = computeVideoExportPlan(4.0, true);
	assert.equal(shortPlan.fps, 60);
	assert.equal(shortPlan.width, VIDEO_WIDTH);
	assert.equal(shortPlan.height, VIDEO_HEIGHT);
	assert.equal(shortPlan.stageHeight, VIDEO_HEIGHT - VIDEO_TIMELINE_HEIGHT);
	assert.equal(shortPlan.motionFrames, 240);

	const longPlan = computeVideoExportPlan(15.0, true);
	assert.equal(longPlan.fps, 30);
	assert.equal(longPlan.motionFrames, 450);

	const cleanPlan = computeVideoExportPlan(5.0, false);
	assert.equal(cleanPlan.stageHeight, VIDEO_HEIGHT);

	const result = makeSampleResult();
	assert.equal(formatVideoScopeLabel(result), 'CYCLE_1_SEGMENT');
	assert.equal(
		formatVideoScopeLabel(makeSampleResult({ scope: 'WHOLE_AUTO', activeSegmentId: 'WHOLE_AUTO' })),
		'Whole Auto',
	);
	assert.equal(buildSuggestedVideoFilename(result), 'RightTrenchAuto-CYCLE_1_SEGMENT-Blue-581.mp4');
	assert.equal(formatVideoTime(1.42), '0:01.42');

	const vpTelemetry = computeVideoViewport(result, true);
	const vpClean = computeVideoViewport(result, false);
	assert.ok(vpTelemetry.spanY > vpClean.spanY);

	const { ctx: staticCtx } = makeMockCanvasContext();
	const renderCtx = {
		result,
		baselineResult: null,
		diffMode: false,
		disturbances: [],
		fieldImage: null,
		includeTelemetry: true,
	};
	const tf = renderVideoStaticLayer(staticCtx, renderCtx, shortPlan.width, shortPlan.stageHeight);
	assert.ok(tf.scalePxPerMeter > 0);

	// Frame at t = 2.0 s (0.5 s after INTAKE marker at 1.5 s) renders title, live speed, progress, and marker toast
	const { ctx: frameCtx, drawnTexts } = makeMockCanvasContext();
	renderVideoFrame(frameCtx, {}, tf, renderCtx, shortPlan, 2.0);
	const texts = drawnTexts.map((d) => d.text);
	assert.ok(texts.some((t) => t.includes('RightTrenchAuto · CYCLE_1_SEGMENT')));
	assert.ok(texts.some((t) => t === '2.00 m/s'));
	assert.ok(texts.some((t) => t === '1.25 / 2.50 m'));
	assert.ok(texts.some((t) => t === '◆ INTAKE'));
	assert.ok(texts.some((t) => t === '0:02.00'));
	assert.ok(texts.some((t) => t === '0:04.00'));

	// Clean mode renders zero HUD text
	const { ctx: cleanCtx, drawnTexts: cleanTexts } = makeMockCanvasContext();
	renderVideoFrame(cleanCtx, {}, tf, { ...renderCtx, includeTelemetry: false }, cleanPlan, 2.0);
	assert.equal(cleanTexts.length, 0);

	// Base64 roundtrip
	const sampleBytes = new Uint8Array([0, 1, 2, 250, 255]);
	assert.deepEqual(Array.from(Buffer.from(uint8ArrayToBase64(sampleBytes), 'base64')), [0, 1, 2, 250, 255]);
});

void test('exportPathVideo encodes via WebCodecs mp4-muxer when available and falls back to JPEG frames otherwise', async () => {
	const result = makeSampleResult({ totalDurationSeconds: 0.2 });
	const context = {
		result,
		baselineResult: null,
		diffMode: false,
		disturbances: [],
		fieldImage: null,
		includeTelemetry: true,
	};

	const origDoc = globalThis.document;
	const origVideoEncoder = globalThis.VideoEncoder;
	const origVideoFrame = globalThis.VideoFrame;
	const origEncodedVideoChunk = globalThis.EncodedVideoChunk;

	const makeCanvas = (w, h) => ({
		width: w,
		height: h,
		getContext: () => makeMockCanvasContext().ctx,
		toBlob(cb) {
			cb({ arrayBuffer: async () => Uint8Array.from([0xff, 0xd8, 0xff, 0xd9]).buffer });
		},
	});

	globalThis.document = {
		createElement: (tag) => (tag === 'canvas' ? makeCanvas(1280, 720) : {}),
	};

	try {
		// 1. Mock WebCodecs H.264 encoder -> produces valid MP4 container with 'ftyp' box
		class MockVideoFrame {
			constructor(_canvas, init) {
				this.timestamp = init.timestamp;
				this.duration = init.duration;
			}
			close() {}
		}
		class MockEncodedVideoChunk {
			constructor(init) {
				this.type = init.type;
				this.timestamp = init.timestamp;
				this.duration = init.duration;
				this._bytes = init.bytes;
				this.byteLength = init.bytes.byteLength;
			}
			copyTo(dest) {
				new Uint8Array(dest).set(this._bytes);
			}
		}
		class MockVideoEncoder {
			static async isConfigSupported(cfg) {
				return { supported: cfg.codec.startsWith('avc1'), config: cfg };
			}
			constructor({ output }) {
				this._output = output;
				this.encodeQueueSize = 0;
				this.state = 'unconfigured';
			}
			configure() {
				this.state = 'configured';
			}
			encode(frame, opts) {
				const bytes = Uint8Array.from([0, 0, 0, 4, 0x65, 0x88, 0x84, 0x21]);
				this._output(
					new MockEncodedVideoChunk({
						type: opts?.keyFrame ? 'key' : 'delta',
						timestamp: frame.timestamp,
						duration: frame.duration,
						bytes,
					}),
					opts?.keyFrame
						? {
								decoderConfig: {
									codec: 'avc1.640028',
									description: Uint8Array.from([
										1, 0x64, 0, 0x28, 0xff, 0xe1, 0, 4, 0x67, 0x64, 0, 0x28, 1, 0, 4, 0x68, 0xee, 0x3c, 0x80,
									]),
								},
							}
						: undefined,
				);
			}
			async flush() {}
			close() {
				this.state = 'closed';
			}
			addEventListener() {}
			removeEventListener() {}
		}

		globalThis.VideoFrame = MockVideoFrame;
		globalThis.EncodedVideoChunk = MockEncodedVideoChunk;
		globalThis.VideoEncoder = MockVideoEncoder;

		const mp4Outcome = await exportPathVideo(context, 'mp4');
		assert.ok(mp4Outcome);
		assert.equal(mp4Outcome.format, 'mp4');
		assert.equal(mp4Outcome.fps, 60);
		assert.equal(mp4Outcome.frameCount, computeVideoExportPlan(0.2, true).totalFrames);
		const mp4Buf = Buffer.from(mp4Outcome.bytesBase64, 'base64');
		assert.equal(mp4Buf.subarray(4, 8).toString('ascii'), 'ftyp');

		// 2. Without VideoEncoder -> streams JPEG frames to fallback handler
		delete globalThis.VideoEncoder;
		delete globalThis.VideoFrame;
		delete globalThis.EncodedVideoChunk;
		let receivedFrames = null;
		let receivedFps = 0;
		const fallbackOutcome = await exportPathVideo(context, 'mp4', async (frames, fps) => {
			receivedFrames = frames;
			receivedFps = fps;
		});
		assert.equal(fallbackOutcome, null);
		assert.equal(receivedFps, 60);
		assert.equal(receivedFrames.length, computeVideoExportPlan(0.2, true).totalFrames);
	} finally {
		if (origDoc === undefined) delete globalThis.document;
		else globalThis.document = origDoc;
		if (origVideoEncoder === undefined) delete globalThis.VideoEncoder;
		else globalThis.VideoEncoder = origVideoEncoder;
		if (origVideoFrame === undefined) delete globalThis.VideoFrame;
		else globalThis.VideoFrame = origVideoFrame;
		if (origEncodedVideoChunk === undefined) delete globalThis.EncodedVideoChunk;
		else globalThis.EncodedVideoChunk = origEncodedVideoChunk;
	}
});

void test('webview export button and extension host export command wire up end-to-end', async () => {
	const html = fs.readFileSync(path.join(__dirname, '../media/preview.html'), 'utf8');
	assert.match(html, /id="btn-export-video"/);
	assert.match(html, /data-video-telemetry="true"/);

	// 1. Test webview bundle export button & state transitions
	const elements = new Map();
	const listeners = new Map();
	const postedMessages = [];
	const telemetryBtns = ['true', 'false'].map((val) => {
		const b = makeEl(`telemetry-${val}`);
		b.dataset.videoTelemetry = val;
		return b;
	});

	function makeEl(name) {
		const attrs = new Map();
		const children = [];
		return {
			name,
			dataset: {},
			style: {},
			hidden: false,
			disabled: false,
			textContent: '',
			value: '',
			clientWidth: 400,
			clientHeight: 200,
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
				return children.includes(other);
			},
			focus() {},
			select() {},
			setPointerCapture() {},
			hasPointerCapture() {
				return false;
			},
			getBoundingClientRect: () => ({ left: 0, top: 0, width: 400, height: 24 }),
			getContext: () => makeMockCanvasContext().ctx,
			addEventListener(type, cb) {
				listeners.set(`${name}:${type}`, cb);
			},
		};
	}

	function querySelector(sel) {
		if (!elements.has(sel)) elements.set(sel, makeEl(sel));
		return elements.get(sel);
	}

	querySelector('#speed-popover').hidden = true;
	querySelector('#options-popover').hidden = true;
	querySelector('#timeline-hover-line').hidden = true;
	querySelector('#timeline-tooltip').hidden = true;

	runInNewContext(fs.readFileSync(path.join(__dirname, '../media/preview.bundle.js'), 'utf8'), {
		document: {
			body: { dataset: {} },
			addEventListener() {},
			querySelector,
			querySelectorAll(sel) {
				if (sel === '[data-video-telemetry]') return telemetryBtns;
				return [];
			},
			createElement(tag) {
				return makeEl(tag);
			},
		},
		window: {
			addEventListener(type, cb) {
				if (!listeners.has(`win:${type}`)) listeners.set(`win:${type}`, []);
				listeners.get(`win:${type}`).push(cb);
			},
			requestAnimationFrame() {
				return 1;
			},
		},
		Image: class {},
		acquireVsCodeApi: () => ({
			getState: () => undefined,
			setState() {},
			postMessage(msg) {
				postedMessages.push(msg);
			},
		}),
		performance: { now: () => 1000 },
		setTimeout: () => 1,
		clearTimeout() {},
	});

	assert.equal(querySelector('#btn-export-video').disabled, true);
	const sampleResult = makeSampleResult();
	for (const cb of listeners.get('win:message')) {
		cb({ data: { type: 'previewResult', result: sampleResult, sourceRevision: 1 } });
	}
	assert.equal(querySelector('#btn-export-video').disabled, false);

	querySelector('#btn-export-video').onclick();
	assert.equal(querySelector('#btn-export-video').dataset.exporting, 'true');
	const reqMsg = postedMessages.find((m) => m.type === 'requestVideoExport');
	assert.ok(reqMsg);
	assert.equal(reqMsg.suggestedFilename, 'RightTrenchAuto-CYCLE_1_SEGMENT-Blue-581.mp4');

	for (const cb of listeners.get('win:message')) {
		cb({ data: { type: 'videoExportCancelled' } });
	}
	assert.equal(querySelector('#btn-export-video').dataset.exporting, 'false');
	assert.equal(querySelector('#btn-export-video').disabled, false);

	// 2. Test extension host trailblazer.exportVideo + requestVideoExport -> file write
	const tmpDir = fs.mkdtempSync(path.join(os.tmpdir(), 'tb-video-test-'));
	const outFile = path.join(tmpDir, 'exported-path.mp4');
	const commands = new Map();
	const createdPanels = [];
	const infoMessages = [];
	const repoRoot = path.resolve(__dirname, '../../..');
	const rightTrenchPath = path.join(repoRoot, 'comp-bot/src/main/java/frc/robot/autos/RightTrenchAuto.java');

	const mockVscode = {
		ViewColumn: { One: 1, Beside: -2 },
		TextEditorRevealType: { InCenterIfOutsideViewport: 2 },
		Uri: {
			file: (p) => ({ scheme: 'file', fsPath: p, toString: () => `file://${p}` }),
			joinPath: (base, ...segments) => ({
				scheme: 'file',
				fsPath: path.join(base.fsPath, ...segments),
				toString: () => `file://${path.join(base.fsPath, ...segments)}`,
			}),
		},
		Position: class {},
		Selection: class {},
		Range: class {},
		workspace: {
			isTrusted: true,
			workspaceFolders: [{ uri: { fsPath: repoRoot } }],
			getConfiguration: () => ({ get: (_k, fallback) => fallback }),
			onDidGrantWorkspaceTrust: () => ({ dispose() {} }),
			onDidChangeTextDocument: () => ({ dispose() {} }),
			onDidSaveTextDocument: () => ({ dispose() {} }),
			openTextDocument: async (p) => ({ lineCount: 200, uri: { fsPath: p } }),
		},
		window: {
			activeTextEditor: {
				document: {
					uri: { scheme: 'file', fsPath: rightTrenchPath },
					getText: () =>
						'package frc.robot.autos;\npublic final class RightTrenchAuto extends TrailblazerAuto<RightTrenchAuto.State> {\nstatic final AutoSegment CYCLE_1_SEGMENT = Trailblazer.segment("c1").stopAt(1,1,0).build();\n}',
				},
				selection: { active: { line: 2 } },
			},
			onDidChangeActiveTextEditor: () => ({ dispose() {} }),
			onDidChangeTextEditorSelection: () => ({ dispose() {} }),
			showErrorMessage: (m) => infoMessages.push(`ERR:${m}`),
			showInformationMessage: async (m) => {
				infoMessages.push(m);
				return undefined;
			},
			showSaveDialog: async () => ({ scheme: 'file', fsPath: outFile }),
			showTextDocument: async () => ({ selection: null, revealRange() {} }),
			createWebviewPanel: (_vt, _title, _show, _opts) => {
				const sentToWebview = [];
				const panel = {
					webview: {
						cspSource: 'https://vscode-resource.test',
						html: '',
						asWebviewUri: (u) => u,
						postMessage: async (m) => {
							sentToWebview.push(m);
							return true;
						},
						onDidReceiveMessage: (cb) => {
							panel._onMessage = cb;
							return { dispose() {} };
						},
					},
					_sentToWebview: sentToWebview,
					reveal() {},
					onDidDispose: (cb) => {
						panel._onDispose = cb;
						return { dispose() {} };
					},
					dispose() {
						if (this._onDispose) this._onDispose();
					},
				};
				createdPanels.push(panel);
				return panel;
			},
		},
		commands: {
			registerCommand: (name, cb) => {
				commands.set(name, cb);
				return { dispose() {} };
			},
		},
	};

	const origLoad = Module._load;
	Module._load = function (request, parent, isMain) {
		if (request === 'vscode') return mockVscode;
		return origLoad.call(this, request, parent, isMain);
	};

	try {
		const extPath = path.resolve(__dirname, '..');
		delete require.cache[require.resolve('../out/extension.cjs')];
		const ext = require('../out/extension.cjs');
		ext.activate({ extensionPath: extPath, extensionUri: { fsPath: extPath }, subscriptions: [] });

		assert.equal(createdPanels.length, 1);
		const panel = createdPanels[0];

		// Command triggers export in webview
		commands.get('trailblazer.exportVideo')();
		assert.ok(panel._sentToWebview.some((m) => m.type === 'triggerVideoExport'));

		// Webview requests export -> Save dialog -> startVideoExport
		panel._onMessage({ type: 'requestVideoExport', suggestedFilename: 'test.mp4' });
		await new Promise((r) => setTimeout(r, 30));
		const startMsg = panel._sentToWebview.find((m) => m.type === 'startVideoExport');
		assert.ok(startMsg);
		assert.equal(startMsg.targetFormat, 'mp4');

		// Webview posts encoded mp4 bytes -> written to disk cleanly
		const payload = Buffer.from('000000186674797069736f6d', 'hex');
		panel._onMessage({
			type: 'videoExportEncoded',
			exportRequestId: startMsg.exportRequestId,
			encodedFormat: 'mp4',
			base64: payload.toString('base64'),
		});
		await new Promise((r) => setTimeout(r, 40));

		assert.deepEqual(fs.readFileSync(outFile), payload);
		assert.ok(panel._sentToWebview.some((m) => m.type === 'videoExportFinished'));
		assert.ok(infoMessages.some((m) => m.includes('Exported exported-path.mp4')));

		ext.deactivate();
	} finally {
		Module._load = origLoad;
		fs.rmSync(tmpDir, { recursive: true, force: true });
	}
});
