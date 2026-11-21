import * as vscode from 'vscode';
import * as path from 'node:path';
import { readFileSync } from 'node:fs';
import { randomBytes } from 'node:crypto';
import {
	analyzeJavaSourceSpans,
	discoverWorkspaceRoot,
	isTrailblazerAutoFile,
	resolveCursorTarget,
	resolveGitBaselineSource,
} from './project';
import { parsePreviewResult, type DiagnosticEntry, type PreviewResult, type SegmentDescriptor } from './schema';
import { renderWebview } from './webview';
import { WarmPreviewWorkerClient } from './worker';

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
	segments: SegmentDescriptor[];
	latestResult: PreviewResult | null;
	latestResultRevision: number;
	latestBaselineResult: PreviewResult | null;
	baselineCache: Map<string, PreviewResult>;
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
	): Promise<PreviewResult | null> {
		if (!session.diffMode) return null;
		const baseSource = resolveGitBaselineSource(session.workspaceRoot, session.filePath, session.sourceText);
		if (!baseSource) return null;

		const cacheKey = `${session.filePath}::${scope}::${segmentId ?? 'ALL'}::${session.alliance}::${session.robotConfigId}::${baseSource.length}`;
		const cached = session.baselineCache.get(cacheKey);
		if (cached) return cached;

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
				return parsed;
			}
		} catch {
			// Ignore if baseline fails or is superseded
		}
		return session.latestBaselineResult;
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

				const baselineResult = await computeBaselineIfNeeded(
					session,
					worker,
					validated.scope,
					validated.scope === 'SEGMENT' ? validated.activeSegmentId : null,
				);
				if (!isCurrent()) return;
				session.latestBaselineResult = baselineResult;

				void session.panel.webview.postMessage({
					type: 'previewResult',
					result: validated,
					baselineResult,
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
				void activeSession.panel.webview.postMessage({
					type: 'sourceChanged',
					sourceRevision: activeSession.sourceRevision,
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
			segments: analyzeJavaSourceSpans(docText),
			latestResult: null,
			latestResultRevision: -1,
			latestBaselineResult: null,
			baselineCache: new Map(),
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
				line?: number;
				sourceRevision?: number;
			}) => {
				if (activeSession !== session) return;
				switch (msg.type) {
					case 'ready':
					case 'refresh':
						session.baselineCache.clear();
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
					case 'revealLine':
						if (msg.sourceRevision === session.sourceRevision && typeof msg.line === 'number' && msg.line > 0) {
							session.cursorLine = msg.line;
							void revealSourceLine(session.filePath, msg.line);
						}
						break;
				}
			},
		);

		panel.onDidDispose(() => {
			msgSub.dispose();
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
