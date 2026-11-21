const test = require('node:test');
const assert = require('node:assert/strict');
const Module = require('node:module');
const path = require('node:path');

void test('extension host gates on workspace trust, opens split pane with ViewColumn.Beside and preserveFocus, and cleans up on deactivate', async () => {
	const commands = new Map();
	const trustListeners = [];
	const createdPanels = [];
	let trusted = false;

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
		Position: class {
			constructor(line, character) {
				this.line = line;
				this.character = character;
			}
		},
		Selection: class {
			constructor(anchor, active) {
				this.anchor = anchor;
				this.active = active;
			}
		},
		Range: class {
			constructor(start, end) {
				this.start = start;
				this.end = end;
			}
		},
		workspace: {
			get isTrusted() {
				return trusted;
			},
			workspaceFolders: [{ uri: { fsPath: repoRoot } }],
			getConfiguration: () => ({
				get: (_key, fallback) => fallback,
			}),
			onDidGrantWorkspaceTrust: (cb) => {
				trustListeners.push(cb);
				return { dispose() {} };
			},
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
			showErrorMessage: () => {},
			showTextDocument: async () => ({
				selection: null,
				revealRange() {},
			}),
			createWebviewPanel: (viewType, title, showOptions, options) => {
				const panel = {
					viewType,
					title,
					showOptions,
					options,
					disposed: false,
					webview: {
						cspSource: 'https://vscode-resource.test',
						html: '',
						asWebviewUri: (u) => u,
						postMessage: async () => true,
						onDidReceiveMessage: () => ({ dispose() {} }),
					},
					reveal() {},
					onDidDispose: (cb) => {
						panel._onDispose = cb;
						return { dispose() {} };
					},
					dispose() {
						this.disposed = true;
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
		const ext = require('../out/extension.cjs');
		const context = {
			extensionPath: extPath,
			extensionUri: { fsPath: extPath },
			subscriptions: [],
		};

		// 1. Activate in untrusted workspace -> no panel created
		trusted = false;
		ext.activate(context);
		assert.equal(createdPanels.length, 0);

		// Running command while untrusted remains blocked
		await commands.get('trailblazer.previewAuto')();
		assert.equal(createdPanels.length, 0);

		// 2. Grant workspace trust -> opens panel beside active editor with preserveFocus: true
		trusted = true;
		for (const listener of trustListeners) listener();
		assert.equal(createdPanels.length, 1);
		assert.equal(createdPanels[0].showOptions.viewColumn, mockVscode.ViewColumn.Beside);
		assert.equal(createdPanels[0].showOptions.preserveFocus, true);
		assert.match(createdPanels[0].webview.html, /Field2d_2026FRCFieldV2\/image\.png/);

		// 3. Deactivate disposes active panels and workers cleanly
		ext.deactivate();
		assert.equal(createdPanels[0].disposed, true);
	} finally {
		Module._load = origLoad;
	}
});
