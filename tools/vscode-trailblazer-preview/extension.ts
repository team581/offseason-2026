import * as vscode from 'vscode';
import * as path from 'node:path';
import { readFileSync } from 'node:fs';
import { writeFile } from 'node:fs/promises';
import { execFile, spawn } from 'node:child_process';
import { randomBytes } from 'node:crypto';
import {
	analyzeJavaSourceSpans,
	discoverWorkspaceRoot,
	isTrailblazerAutoFile,
	resolveCursorTarget,
	resolveGitBaselineSource,
} from './project';
import {
	parsePreviewResult,
	type DiagnosticEntry,
	type DisturbanceEvent,
	type PreviewResult,
	type SegmentDescriptor,
} from './schema';
import { renderWebview } from './webview';
import { WarmPreviewWorkerClient } from './worker';

const DISTURBANCE_STORAGE_KEY = 'trailblazer.preview.disturbancesByFile';
const VIDEO_EXPORT_DIR_KEY = 'trailblazer.preview.lastVideoExportDir';

let ffmpegCheckPromise: Promise<boolean> | null = null;

function isFfmpegAvailable(): Promise<boolean> {
	if (!ffmpegCheckPromise) {
		ffmpegCheckPromise = new Promise((resolve) => {
			execFile('ffmpeg', ['-version'], { timeout: 2000 }, (err) => {
				resolve(!err);
			});
		});
	}
	return ffmpegCheckPromise;
}

function runFfmpegStdin(args: string[], writeInput: (stdin: NodeJS.WritableStream) => void): Promise<void> {
	return new Promise((resolve, reject) => {
		const proc = spawn('ffmpeg', args, { stdio: ['pipe', 'ignore', 'pipe'] });
		let stderr = '';
		proc.stderr.on('data', (chunk: Buffer) => {
			stderr += chunk.toString();
		});
		proc.on('error', (err) => reject(err));
		proc.on('close', (code) => {
			if (code === 0) resolve();
			else reject(new Error(`ffmpeg exited with code ${code}: ${stderr.slice(-400)}`));
		});
		proc.stdin.on('error', () => {
			// Ignore EPIPE when ffmpeg exits early; close handler reports the exit status.
		});
		writeInput(proc.stdin);
		proc.stdin.end();
	});
}

async function transcodeVideoViaFfmpeg(
	inputBuffer: Buffer,
	inputFormat: 'webm' | 'mp4',
	outputPath: string,
	targetFormat: 'mp4' | 'webm',
): Promise<void> {
	const codecArgs =
		targetFormat === 'mp4'
			? ['-c:v', 'libx264', '-preset', 'superfast', '-crf', '18', '-pix_fmt', 'yuv420p', '-movflags', '+faststart']
			: ['-c:v', 'libvpx-vp9', '-deadline', 'realtime', '-cpu-used', '8', '-b:v', '4M'];
	await runFfmpegStdin(['-y', '-f', inputFormat, '-i', 'pipe:0', ...codecArgs, outputPath], (stdin) => {
		stdin.write(inputBuffer);
	});
}

async function encodeJpegFramesViaFfmpeg(
	framesBase64: string[],
	fps: number,
	outputPath: string,
	targetFormat: 'mp4' | 'webm',
): Promise<void> {
	const codecArgs =
		targetFormat === 'mp4'
			? ['-c:v', 'libx264', '-preset', 'superfast', '-crf', '18', '-pix_fmt', 'yuv420p', '-movflags', '+faststart']
			: ['-c:v', 'libvpx-vp9', '-deadline', 'realtime', '-cpu-used', '8', '-b:v', '4M'];
	await runFfmpegStdin(
		['-y', '-f', 'image2pipe', '-vcodec', 'mjpeg', '-framerate', String(fps), '-i', 'pipe:0', ...codecArgs, outputPath],
		(stdin) => {
			for (const b64 of framesBase64) {
				stdin.write(Buffer.from(b64, 'base64'));
			}
		},
	);
}

interface PanelSession {
	panel: vscode.WebviewPanel;
	workspaceRoot: string;
	filePath: string;
	sourceText: string;
	cursorLine: number;
	sourceRevision: number;
	requestId: number;
	pinnedWholeAuto: boolean;
	diffMode: boolean;
	selectedSegmentId: string | null;
	lastResolvedScope: 'SEGMENT' | 'WHOLE_AUTO' | null;
	lastResolvedSegmentId: string | null;
	alliance: 'Blue' | 'Red';
	robotConfigId: '581' | '9991' | '9981';
	disturbances: DisturbanceEvent[];
	segments: SegmentDescriptor[];
	latestResult: PreviewResult | null;
	latestResultRevision: number;
	latestBaselineResult: PreviewResult | null;
	baselineCache: Map<string, PreviewResult>;
	nominalCache: Map<string, PreviewResult>;
	stale: boolean;
}

const activeWorkers = new Map<string, WarmPreviewWorkerClient>();
let activeSession: PanelSession | null = null;
let cursorTimer: ReturnType<typeof setTimeout> | null = null;
let editTimer: ReturnType<typeof setTimeout> | null = null;

function getOrCreateWorker(workspaceRoot: string): WarmPreviewWorkerClient {
	let worker = activeWorkers.get(workspaceRoot);
	if (!worker) {
		worker = new WarmPreviewWorkerClient(workspaceRoot, () =>
			vscode.workspace.getConfiguration('trailblazer.preview').get<string>('javaHome', ''),
		);
		activeWorkers.set(workspaceRoot, worker);
	}
	return worker;
}

export function activate(context: vscode.ExtensionContext) {
	const suppressedFiles = new Set<string>();
	const pendingVideoExports = new Map<
		number,
		{ targetPath: string; targetFormat: 'mp4' | 'webm'; startedAtMs: number }
	>();
	let nextExportRequestId = 1;

	function notifyVideoExported(finalPath: string, startedAtMs: number) {
		const secs = ((Date.now() - startedAtMs) / 1000).toFixed(2);
		void context.workspaceState?.update(VIDEO_EXPORT_DIR_KEY, path.dirname(finalPath));
		void vscode.window
			.showInformationMessage?.(`Exported ${path.basename(finalPath)} (${secs} s)`, 'Open Video')
			?.then((choice) => {
				if (choice === 'Open Video') {
					void vscode.env?.openExternal?.(vscode.Uri.file(finalPath));
				}
			});
	}

	async function handleRequestVideoExport(session: PanelSession, suggestedFilename?: string) {
		const lastDir = context.workspaceState?.get<string>(VIDEO_EXPORT_DIR_KEY);
		const defaultDir = lastDir || session.workspaceRoot;
		const cleanFilename = path.basename(suggestedFilename || 'trailblazer-path.mp4');
		const saveUri = await vscode.window.showSaveDialog?.({
			defaultUri: vscode.Uri.file(path.join(defaultDir, cleanFilename)),
			filters: {
				'MP4 Video': ['mp4'],
				'WebM Video': ['webm'],
			},
			saveLabel: 'Export Video',
		});
		if (!saveUri) {
			void session.panel.webview.postMessage({ type: 'videoExportCancelled' });
			return;
		}
		const targetFormat = saveUri.fsPath.toLowerCase().endsWith('.webm') ? 'webm' : 'mp4';
		const exportRequestId = nextExportRequestId++;
		pendingVideoExports.set(exportRequestId, {
			targetPath: saveUri.fsPath,
			targetFormat,
			startedAtMs: Date.now(),
		});
		void session.panel.webview.postMessage({
			type: 'startVideoExport',
			exportRequestId,
			targetFormat,
		});
	}

	async function handleVideoExportEncoded(
		session: PanelSession,
		exportRequestId: number,
		encodedFormat: 'mp4' | 'webm',
		base64: string,
	) {
		const pending = pendingVideoExports.get(exportRequestId);
		pendingVideoExports.delete(exportRequestId);
		if (!pending) return;

		try {
			const buf = Buffer.from(base64, 'base64');
			let finalPath = pending.targetPath;
			if (encodedFormat === pending.targetFormat) {
				await writeFile(finalPath, buf);
			} else if (await isFfmpegAvailable()) {
				await transcodeVideoViaFfmpeg(buf, encodedFormat, finalPath, pending.targetFormat);
			} else {
				const fallbackExt = `.${encodedFormat}`;
				finalPath = finalPath.replace(/\.(mp4|webm)$/i, fallbackExt);
				if (!finalPath.toLowerCase().endsWith(fallbackExt)) {
					finalPath += fallbackExt;
				}
				await writeFile(finalPath, buf);
			}
			notifyVideoExported(finalPath, pending.startedAtMs);
		} catch (err) {
			void vscode.window.showErrorMessage?.(`Video export failed: ${err instanceof Error ? err.message : String(err)}`);
		} finally {
			void session.panel.webview.postMessage({ type: 'videoExportFinished', exportRequestId });
		}
	}

	async function handleVideoExportJpegFrames(
		session: PanelSession,
		exportRequestId: number,
		fps: number,
		framesBase64: string[],
	) {
		const pending = pendingVideoExports.get(exportRequestId);
		pendingVideoExports.delete(exportRequestId);
		if (!pending) return;

		try {
			if (!(await isFfmpegAvailable())) {
				throw new Error('WebCodecs VideoEncoder is unavailable and ffmpeg was not found on PATH.');
			}
			await encodeJpegFramesViaFfmpeg(framesBase64, fps, pending.targetPath, pending.targetFormat);
			notifyVideoExported(pending.targetPath, pending.startedAtMs);
		} catch (err) {
			void vscode.window.showErrorMessage?.(`Video export failed: ${err instanceof Error ? err.message : String(err)}`);
		} finally {
			void session.panel.webview.postMessage({ type: 'videoExportFinished', exportRequestId });
		}
	}

	const storedDisturbances: Record<string, DisturbanceEvent[]> = {
		...context.workspaceState?.get<Record<string, DisturbanceEvent[]>>(DISTURBANCE_STORAGE_KEY),
	};

	function getDisturbancesForFile(filePath: string): DisturbanceEvent[] {
		const list = storedDisturbances[filePath];
		return Array.isArray(list) ? list : [];
	}

	function saveDisturbancesForFile(filePath: string, disturbances: DisturbanceEvent[]): void {
		if (disturbances.length === 0) {
			delete storedDisturbances[filePath];
		} else {
			storedDisturbances[filePath] = disturbances;
		}
		void context.workspaceState?.update(DISTURBANCE_STORAGE_KEY, storedDisturbances);
	}

	function prewarmWorkspaceIfTrusted() {
		if (!vscode.workspace.isTrusted) return;
		const folder = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
		if (!folder) return;
		try {
			const root = discoverWorkspaceRoot(folder);
			const worker = getOrCreateWorker(root);
			const activeDoc = vscode.window.activeTextEditor?.document;
			const overlay =
				activeDoc && isTrailblazerAutoFile(activeDoc.uri.fsPath, activeDoc.getText())
					? activeDoc.uri.fsPath
					: undefined;
			void worker.ensureStarted(overlay).catch(() => {
				// Handled when preview runs
			});
		} catch {
			// Not a Gradle workspace
		}
	}

	async function computeBaselineIfNeeded(
		session: PanelSession,
		worker: WarmPreviewWorkerClient,
		scope: 'SEGMENT' | 'WHOLE_AUTO',
		segmentId: string | null,
		sourceRevision: number,
	): Promise<{ result: PreviewResult | null; kind: 'git' | 'disturbance' | null }> {
		if (session.diffMode) {
			const baseSource = resolveGitBaselineSource(session.workspaceRoot, session.filePath, session.sourceText);
			if (!baseSource) return { result: null, kind: null };

			const cacheKey = `${session.filePath}::${scope}::${segmentId ?? 'ALL'}::${session.alliance}::${session.robotConfigId}::${baseSource.length}`;
			const cached = session.baselineCache.get(cacheKey);
			if (cached) return { result: cached, kind: 'git' };

			try {
				const baseResp = await worker.requestPreview({
					sourceFile: session.filePath,
					sourceContent: baseSource,
					scope,
					segmentId,
					cursorLine: session.cursorLine,
					previousSegmentId: segmentId,
					alliance: session.alliance,
					robotConfigId: session.robotConfigId,
					seed: 581,
				});
				if (baseResp.ok && baseResp.result) {
					const parsed = parsePreviewResult(baseResp.result);
					session.baselineCache.set(cacheKey, parsed);
					return { result: parsed, kind: 'git' };
				}
			} catch {
				// Ignore if baseline fails or is superseded
			}
			return { result: session.latestBaselineResult, kind: session.latestBaselineResult ? 'git' : null };
		}

		const hasEnabledDisturbances = session.disturbances.some((d) => d.enabled);
		if (!hasEnabledDisturbances) return { result: null, kind: null };

		const nominalKey = `${session.filePath}::${sourceRevision}::${scope}::${segmentId ?? 'ALL'}::${session.alliance}::${session.robotConfigId}`;
		const cachedNominal = session.nominalCache.get(nominalKey);
		if (cachedNominal) return { result: cachedNominal, kind: 'disturbance' };

		try {
			const nomResp = await worker.requestPreview({
				sourceFile: session.filePath,
				sourceContent: session.sourceText,
				scope,
				segmentId,
				cursorLine: session.cursorLine,
				previousSegmentId: segmentId,
				alliance: session.alliance,
				robotConfigId: session.robotConfigId,
				seed: 581,
				disturbances: [],
			});
			if (nomResp.ok && nomResp.result) {
				const parsed = parsePreviewResult(nomResp.result);
				session.nominalCache.set(nominalKey, parsed);
				return { result: parsed, kind: 'disturbance' };
			}
		} catch {
			// Ignore if nominal baseline fails or is superseded
		}
		return { result: null, kind: null };
	}

	async function runPreview(session: PanelSession, force = false) {
		if (activeSession !== session || !vscode.workspace.isTrusted) return;
		const parsedSpans = session.segments.length > 0 ? session.segments : analyzeJavaSourceSpans(session.sourceText);
		const target = session.selectedSegmentId
			? { scope: 'SEGMENT' as const, segmentId: session.selectedSegmentId }
			: resolveCursorTarget(parsedSpans, session.cursorLine, session.pinnedWholeAuto, session.lastResolvedSegmentId);

		if (
			!force &&
			session.latestResult &&
			session.latestResultRevision === session.sourceRevision &&
			!session.stale &&
			target.scope === session.latestResult.scope &&
			(target.segmentId ?? 'WHOLE_AUTO') === session.latestResult.activeSegmentId &&
			target.scope === session.lastResolvedScope &&
			target.segmentId === session.lastResolvedSegmentId
		) {
			return;
		}

		const requestId = ++session.requestId;
		const sourceRevision = session.sourceRevision;
		const isCurrent = () =>
			activeSession === session && session.requestId === requestId && session.sourceRevision === sourceRevision;

		session.lastResolvedScope = target.scope;
		session.lastResolvedSegmentId = target.segmentId;

		const worker = getOrCreateWorker(session.workspaceRoot);
		try {
			const response = await worker.requestPreview({
				sourceFile: session.filePath,
				sourceContent: session.sourceText,
				scope: session.pinnedWholeAuto ? 'WHOLE_AUTO' : session.selectedSegmentId ? 'SEGMENT' : 'CURSOR',
				segmentId: target.segmentId,
				cursorLine: session.cursorLine,
				previousSegmentId: session.lastResolvedSegmentId,
				alliance: session.alliance,
				robotConfigId: session.robotConfigId,
				seed: 581,
				disturbances: session.disturbances,
			});

			if (!isCurrent()) return;

			if (response.ok && response.result) {
				const validated = parsePreviewResult(response.result);
				session.latestResult = validated;
				session.latestResultRevision = sourceRevision;
				session.segments = validated.segments;
				session.stale = false;
				session.lastResolvedScope = validated.scope;
				session.lastResolvedSegmentId = validated.scope === 'SEGMENT' ? validated.activeSegmentId : null;

				const baseline = await computeBaselineIfNeeded(
					session,
					worker,
					validated.scope,
					validated.scope === 'SEGMENT' ? validated.activeSegmentId : null,
					sourceRevision,
				);
				if (!isCurrent()) return;
				session.latestBaselineResult = baseline.result;

				void session.panel.webview.postMessage({
					type: 'previewResult',
					result: validated,
					baselineResult: baseline.result,
					baselineKind: baseline.kind,
					disturbances: session.disturbances,
					diffMode: session.diffMode,
					stale: false,
					pinnedWholeAuto: session.pinnedWholeAuto,
					cursorLine: session.cursorLine,
					sourceRevision,
					diagnostics: validated.diagnostics,
				});
			} else {
				session.stale = true;
				if (response.segments && response.segments.length > 0) {
					session.segments = response.segments;
				}
				const diagnostics: DiagnosticEntry[] = response.diagnostics ?? [
					{
						severity: 'ERROR',
						code: response.errorCode ?? 'PREVIEW_ERROR',
						message: response.errorMessage ?? 'Preview failed',
						segmentId: null,
						targetGoal: null,
						timestampSeconds: null,
						line: null,
					},
				];
				void session.panel.webview.postMessage({
					type: 'previewStale',
					sourceRevision,
					stale: true,
					diagnostics,
					segments: session.segments,
					lastResult: session.latestResult,
				});
			}
		} catch (err) {
			const msg = err instanceof Error ? err.message : String(err);
			if (msg.startsWith('Superseded by') || msg === 'Worker disposed') {
				return;
			}
			if (isCurrent()) {
				session.stale = true;
				void session.panel.webview.postMessage({
					type: 'previewStale',
					sourceRevision,
					stale: true,
					diagnostics: [
						{
							severity: 'ERROR',
							code: 'WORKER_ERROR',
							message: msg,
							segmentId: null,
							targetGoal: null,
							timestampSeconds: null,
							line: null,
						},
					],
					segments: session.segments,
					lastResult: session.latestResult,
				});
			}
		}
	}

	async function openPreview(uri?: vscode.Uri, automatic = false) {
		if (!vscode.workspace.isTrusted) return;
		const editor = vscode.window.activeTextEditor;
		const targetUri = uri ?? editor?.document.uri;
		if (!targetUri || targetUri.scheme !== 'file' || !targetUri.fsPath.endsWith('.java')) {
			return;
		}
		const filePath = targetUri.fsPath;
		if (automatic && suppressedFiles.has(filePath)) {
			return;
		}
		const docText =
			editor && editor.document.uri.fsPath === filePath ? editor.document.getText() : readFileSync(filePath, 'utf8');
		if (!isTrailblazerAutoFile(filePath, docText)) {
			return;
		}

		let workspaceRoot: string;
		try {
			workspaceRoot = discoverWorkspaceRoot(filePath);
		} catch (e) {
			if (!automatic) {
				void vscode.window.showErrorMessage(String(e));
			}
			return;
		}

		const cursorLine = editor && editor.document.uri.fsPath === filePath ? editor.selection.active.line + 1 : 1;
		const config = vscode.workspace.getConfiguration('trailblazer.preview');
		const defaultAlliance = config.get<'Blue' | 'Red'>('defaultAlliance', 'Blue');
		const defaultRobot = config.get<'581' | '9991' | '9981'>('defaultRobot', '581');

		if (activeSession) {
			if (filePath !== activeSession.filePath) {
				if (cursorTimer) clearTimeout(cursorTimer);
				if (editTimer) clearTimeout(editTimer);
				cursorTimer = editTimer = null;
				activeSession.sourceRevision++;
				activeSession.latestResult = null;
				activeSession.latestBaselineResult = null;
				activeSession.lastResolvedSegmentId = null;
				activeSession.selectedSegmentId = null;
				activeSession.nominalCache.clear();
				activeSession.disturbances = getDisturbancesForFile(filePath);
				void activeSession.panel.webview.postMessage({
					type: 'sourceChanged',
					sourceRevision: activeSession.sourceRevision,
					disturbances: activeSession.disturbances,
				});
			}
			activeSession.workspaceRoot = workspaceRoot;
			activeSession.filePath = filePath;
			activeSession.sourceText = docText;
			activeSession.cursorLine = cursorLine;
			activeSession.segments = analyzeJavaSourceSpans(docText);
			activeSession.panel.title = `TB3 · ${path.basename(filePath, '.java')}`;
			if (!automatic) {
				activeSession.panel.reveal(vscode.ViewColumn.Beside, true);
			}
			void runPreview(activeSession, true);
			return;
		}

		const panel = vscode.window.createWebviewPanel(
			'trailblazerPreview',
			`TB3 · ${path.basename(filePath, '.java')}`,
			{ viewColumn: vscode.ViewColumn.Beside, preserveFocus: true },
			{
				enableScripts: true,
				retainContextWhenHidden: true,
				localResourceRoots: [vscode.Uri.joinPath(context.extensionUri, 'media')],
			},
		);

		const nonce = randomBytes(16).toString('hex');
		const asset = (relPath: string) =>
			panel.webview.asWebviewUri(vscode.Uri.joinPath(context.extensionUri, 'media', relPath)).toString();
		const template = readFileSync(path.join(context.extensionPath, 'media/preview.html'), 'utf8');
		panel.webview.html = renderWebview(template, {
			cspSource: panel.webview.cspSource,
			nonce,
			stylesheet: asset('preview.css'),
			script: asset('preview.bundle.js'),
			fieldImage: asset('Field2d_2026FRCFieldV2/image.png'),
		});

		const session: PanelSession = {
			panel,
			workspaceRoot,
			filePath,
			sourceText: docText,
			cursorLine,
			sourceRevision: 0,
			requestId: 0,
			pinnedWholeAuto: false,
			diffMode: false,
			selectedSegmentId: null,
			lastResolvedScope: null,
			lastResolvedSegmentId: null,
			alliance: defaultAlliance,
			robotConfigId: defaultRobot,
			disturbances: getDisturbancesForFile(filePath),
			segments: analyzeJavaSourceSpans(docText),
			latestResult: null,
			latestResultRevision: -1,
			latestBaselineResult: null,
			baselineCache: new Map(),
			nominalCache: new Map(),
			stale: false,
		};
		activeSession = session;

		const msgSub = panel.webview.onDidReceiveMessage(
			(msg: {
				type?: string;
				pinnedWholeAuto?: boolean;
				diffMode?: boolean;
				segmentId?: string | null;
				alliance?: 'Blue' | 'Red';
				robotConfigId?: '581' | '9991' | '9981';
				disturbances?: DisturbanceEvent[];
				text?: string;
				line?: number;
				sourceRevision?: number;
				suggestedFilename?: string;
				exportRequestId?: number;
				encodedFormat?: 'mp4' | 'webm';
				base64?: string;
				fps?: number;
				framesBase64?: string[];
				message?: string;
			}) => {
				if (activeSession !== session) return;
				switch (msg.type) {
					case 'ready':
						void session.panel.webview.postMessage({
							type: 'disturbancesState',
							disturbances: session.disturbances,
						});
						session.baselineCache.clear();
						session.nominalCache.clear();
						void runPreview(session, true);
						break;
					case 'refresh':
						session.baselineCache.clear();
						session.nominalCache.clear();
						void runPreview(session, true);
						break;
					case 'setMode':
						session.pinnedWholeAuto = Boolean(msg.pinnedWholeAuto);
						session.selectedSegmentId = msg.segmentId ?? null;
						void runPreview(session, true);
						break;
					case 'setDiffMode':
						session.diffMode = Boolean(msg.diffMode);
						void runPreview(session, true);
						break;
					case 'setAlliance':
						if (msg.alliance === 'Blue' || msg.alliance === 'Red') {
							session.alliance = msg.alliance;
							void runPreview(session, true);
						}
						break;
					case 'setRobot':
						if (msg.robotConfigId === '581' || msg.robotConfigId === '9991' || msg.robotConfigId === '9981') {
							session.robotConfigId = msg.robotConfigId;
							void runPreview(session, true);
						}
						break;
					case 'setDisturbances':
						if (Array.isArray(msg.disturbances)) {
							session.disturbances = msg.disturbances;
							saveDisturbancesForFile(session.filePath, session.disturbances);
							void runPreview(session, true);
						}
						break;
					case 'copyText':
						if (typeof msg.text === 'string' && vscode.env?.clipboard) {
							void vscode.env.clipboard.writeText(msg.text);
						}
						break;
					case 'revealLine':
						if (msg.sourceRevision === session.sourceRevision && typeof msg.line === 'number' && msg.line > 0) {
							session.cursorLine = msg.line;
							void revealSourceLine(session.filePath, msg.line);
						}
						break;
					case 'requestVideoExport':
						void handleRequestVideoExport(session, msg.suggestedFilename);
						break;
					case 'videoExportEncoded':
						if (
							typeof msg.exportRequestId === 'number' &&
							(msg.encodedFormat === 'mp4' || msg.encodedFormat === 'webm') &&
							typeof msg.base64 === 'string'
						) {
							void handleVideoExportEncoded(session, msg.exportRequestId, msg.encodedFormat, msg.base64);
						}
						break;
					case 'videoExportJpegFrames':
						if (
							typeof msg.exportRequestId === 'number' &&
							typeof msg.fps === 'number' &&
							Array.isArray(msg.framesBase64)
						) {
							void handleVideoExportJpegFrames(session, msg.exportRequestId, msg.fps, msg.framesBase64);
						}
						break;
					case 'videoExportError':
						if (typeof msg.exportRequestId === 'number') {
							pendingVideoExports.delete(msg.exportRequestId);
						}
						void session.panel.webview.postMessage({
							type: 'videoExportFinished',
							exportRequestId: msg.exportRequestId,
						});
						void vscode.window.showErrorMessage?.(`Video export failed: ${msg.message ?? 'Unknown error'}`);
						break;
				}
			},
		);

		panel.onDidDispose(() => {
			msgSub.dispose();
			pendingVideoExports.clear();
			if (activeSession === session) {
				activeSession = null;
			}
			suppressedFiles.add(filePath);
		});
	}

	async function revealSourceLine(filePath: string, line1Based: number) {
		try {
			const doc = await vscode.workspace.openTextDocument(filePath);
			const editor = await vscode.window.showTextDocument(doc, {
				viewColumn: vscode.ViewColumn.One,
				preserveFocus: false,
			});
			const lineIdx = Math.max(0, Math.min(doc.lineCount - 1, line1Based - 1));
			const pos = new vscode.Position(lineIdx, 0);
			editor.selection = new vscode.Selection(pos, pos);
			editor.revealRange(new vscode.Range(pos, pos), vscode.TextEditorRevealType.InCenterIfOutsideViewport);
		} catch {
			// ignore
		}
	}

	context.subscriptions.push(
		vscode.commands.registerCommand('trailblazer.previewAuto', (uri?: vscode.Uri) => {
			if (uri) suppressedFiles.delete(uri.fsPath);
			else if (vscode.window.activeTextEditor) {
				suppressedFiles.delete(vscode.window.activeTextEditor.document.uri.fsPath);
			}
			void openPreview(uri, false);
		}),
		vscode.commands.registerCommand('trailblazer.toggleWholeAuto', () => {
			if (!activeSession) return;
			activeSession.pinnedWholeAuto = !activeSession.pinnedWholeAuto;
			activeSession.selectedSegmentId = null;
			void runPreview(activeSession, true);
		}),
		vscode.commands.registerCommand('trailblazer.exportVideo', () => {
			if (!activeSession) {
				void vscode.window.showInformationMessage?.('Open a Trailblazer auto preview first.');
				return;
			}
			void activeSession.panel.webview.postMessage({ type: 'triggerVideoExport' });
		}),
		vscode.workspace.onDidGrantWorkspaceTrust(() => {
			prewarmWorkspaceIfTrusted();
			if (vscode.window.activeTextEditor) {
				void openPreview(vscode.window.activeTextEditor.document.uri, true);
			}
		}),
		vscode.window.onDidChangeActiveTextEditor((editor) => {
			if (!editor || !vscode.workspace.isTrusted) return;
			const autoOpen = vscode.workspace.getConfiguration('trailblazer.preview').get<boolean>('autoOpen', true);
			if (autoOpen && isTrailblazerAutoFile(editor.document.uri.fsPath, editor.document.getText())) {
				void openPreview(editor.document.uri, true);
			}
		}),
		vscode.window.onDidChangeTextEditorSelection((event) => {
			if (!activeSession || !vscode.workspace.isTrusted) return;
			if (event.textEditor.document.uri.fsPath !== activeSession.filePath) return;
			const nextLine = event.selections[0]?.active.line !== undefined ? event.selections[0].active.line + 1 : 1;
			if (nextLine === activeSession.cursorLine) return;
			activeSession.cursorLine = nextLine;
			if (activeSession.pinnedWholeAuto || activeSession.selectedSegmentId !== null) return;

			const target = resolveCursorTarget(
				activeSession.segments,
				nextLine,
				activeSession.pinnedWholeAuto,
				activeSession.lastResolvedSegmentId,
			);
			void activeSession.panel.webview.postMessage({
				type: 'cursorLineChanged',
				cursorLine: nextLine,
				segmentId: target.segmentId,
				sourceRevision: activeSession.sourceRevision,
			});

			if (cursorTimer) clearTimeout(cursorTimer);
			const session = activeSession;
			cursorTimer = setTimeout(() => {
				cursorTimer = null;
				void runPreview(session, false);
			}, 150);
		}),
		vscode.workspace.onDidChangeTextDocument((event) => {
			if (!activeSession || !vscode.workspace.isTrusted) return;
			if (event.document.uri.fsPath !== activeSession.filePath) return;
			if (event.contentChanges.length === 0) return;
			const session = activeSession;
			session.sourceText = event.document.getText();
			session.segments = analyzeJavaSourceSpans(session.sourceText);
			session.nominalCache.clear();
			const change = event.contentChanges.length === 1 ? event.contentChanges[0] : null;
			void session.panel.webview.postMessage({
				type: 'sourceEdited',
				sourceRevision: ++session.sourceRevision,
				editedWaypointLine: change ? change.range.start.line + 1 : null,
			});

			if (editTimer) clearTimeout(editTimer);
			editTimer = setTimeout(() => {
				editTimer = null;
				void runPreview(session, true);
			}, 220);
		}),
		vscode.workspace.onDidSaveTextDocument((doc) => {
			if (!activeSession || !vscode.workspace.isTrusted) return;
			const isAuto = doc.uri.fsPath === activeSession.filePath;
			if (!isAuto) {
				const relative = path.relative(activeSession.workspaceRoot, doc.uri.fsPath);
				if (relative.startsWith('..') || !relative.split(path.sep).join('/').includes('/src/main/java/')) return;
				activeSession.baselineCache.clear();
				activeSession.nominalCache.clear();
			} else {
				activeSession.nominalCache.clear();
			}
			if (editTimer) {
				clearTimeout(editTimer);
				editTimer = null;
			}
			if (isAuto) {
				activeSession.sourceText = doc.getText();
				activeSession.segments = analyzeJavaSourceSpans(activeSession.sourceText);
			}
			void runPreview(activeSession, true);
		}),
	);

	prewarmWorkspaceIfTrusted();
	if (
		vscode.workspace.isTrusted &&
		vscode.window.activeTextEditor &&
		vscode.workspace.getConfiguration('trailblazer.preview').get<boolean>('autoOpen', true)
	) {
		void openPreview(vscode.window.activeTextEditor.document.uri, true);
	}
}

export function deactivate() {
	if (cursorTimer) {
		clearTimeout(cursorTimer);
		cursorTimer = null;
	}
	if (editTimer) {
		clearTimeout(editTimer);
		editTimer = null;
	}
	if (activeSession) {
		activeSession.panel.dispose();
		activeSession = null;
	}
	for (const worker of activeWorkers.values()) {
		worker.dispose();
	}
	activeWorkers.clear();
}
