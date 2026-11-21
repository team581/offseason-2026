import { execFileSync } from 'node:child_process';
import * as fs from 'node:fs';
import * as path from 'node:path';
import type { SegmentDescriptor, SourceSpan } from './schema';

const NON_PREVIEW_BASENAMES = new Set([
	'TrailblazerAuto.java',
	'AutoActions.java',
	'AutoContext.java',
	'AutoSelection.java',
	'AutoTelemetry.java',
	'Autos.java',
	'DoNothingAuto.java',
	'BaseImperativeAuto.java',
]);

/** Walks upward from `filePath` to find the Gradle workspace root containing `settings.gradle`. */
export function discoverWorkspaceRoot(filePath: string): string {
	let dir = path.resolve(filePath);
	try {
		if (!fs.existsSync(dir) || !fs.statSync(dir).isDirectory()) {
			dir = path.dirname(dir);
		}
	} catch {
		dir = path.dirname(dir);
	}
	while (true) {
		if (
			fs.existsSync(path.join(dir, 'settings.gradle')) ||
			fs.existsSync(path.join(dir, 'settings.gradle.kts')) ||
			fs.existsSync(path.join(dir, 'gradlew'))
		) {
			return dir;
		}
		const parent = path.dirname(dir);
		if (parent === dir) {
			throw new Error(`No Gradle workspace found for ${filePath}`);
		}
		dir = parent;
	}
}

/** Detects whether a Java file defines a Trailblazer 3 autonomous routine or segment. */
export function isTrailblazerAutoFile(filePath: string, sourceText?: string): boolean {
	if (!filePath.endsWith('.java')) return false;
	const base = path.basename(filePath);
	if (NON_PREVIEW_BASENAMES.has(base)) return false;

	const text = sourceText ?? (fs.existsSync(filePath) ? fs.readFileSync(filePath, 'utf8') : '');
	if (text) {
		if (/abstract\s+class\s+/.test(text)) return false;
		if (
			/\bextends\s+TrailblazerAuto\b/.test(text) ||
			/\bTrailblazer\.segment\s*\(/.test(text) ||
			/\bAutoSegment\s+[A-Za-z_][A-Za-z0-9_]*\s*=/.test(text)
		) {
			return /\bAutoSegment\b/.test(text) || /\bTrailblazer\.segment\b/.test(text);
		}
		return false;
	}
	return base.endsWith('Auto.java');
}

function stripLineComment(line: string): string {
	const idx = line.indexOf('//');
	return idx >= 0 ? line.slice(0, idx) : line;
}

/** Extracts 1-based line spans for AutoSegment declarations, helpers, and State switch branches. */
export function analyzeJavaSourceSpans(source: string): SegmentDescriptor[] {
	const lines = source ? source.split(/\r?\n/) : [];
	const rawSegments: Array<{
		fieldName: string;
		id: string;
		startLine: number;
		endLine: number;
		body: string;
	}> = [];
	const helpers: Array<{ name: string; startLine: number; endLine: number }> = [];
	const cases: Array<{ states: string[]; startLine: number; endLine: number; body: string }> = [];

	const segStartRe = /^\s*(?:(?:private|protected|public|static|final)\s+)*AutoSegment\s+([A-Za-z_][A-Za-z0-9_]*)\s*=/;
	const helperStartRe =
		/^\s*(?:(?:private|protected|public|static|final)\s+)*AutoSegment(?:\.Builder)?\s+([A-Za-z_][A-Za-z0-9_]*)\s*\(/;
	const caseStartRe = /^\s*case\s+([A-Za-z0-9_,\s]+?)(?:->|:)/;
	const literalIdRe = /(?:Trailblazer\.segment|returnToTrench|segment)\s*\(\s*"([^"]+)"/;
	const driveCallRe = /\bdrive(?:WithoutClearance)?\s*\(\s*([A-Za-z_][A-Za-z0-9_]*)\s*\)/g;

	for (let i = 0; i < lines.length; i++) {
		const segMatch = segStartRe.exec(lines[i]);
		if (segMatch) {
			const fieldName = segMatch[1];
			const startLine = i + 1;
			let endLine = startLine;
			let paren = 0;
			let brace = 0;
			const bodyLines: string[] = [];
			for (let j = i; j < lines.length; j++) {
				const clean = stripLineComment(lines[j]);
				bodyLines.push(clean);
				let done = false;
				for (const ch of clean) {
					if (ch === '(') paren++;
					else if (ch === ')') paren = Math.max(0, paren - 1);
					else if (ch === '{') brace++;
					else if (ch === '}') brace = Math.max(0, brace - 1);
					else if (ch === ';' && paren === 0 && brace === 0) {
						done = true;
						break;
					}
				}
				if (done) {
					endLine = j + 1;
					i = j;
					break;
				}
			}
			const body = bodyLines.join('\n');
			const idMatch = literalIdRe.exec(body);
			rawSegments.push({
				fieldName,
				id: idMatch ? idMatch[1] : fieldName,
				startLine,
				endLine,
				body,
			});
			continue;
		}

		const helperMatch = helperStartRe.exec(lines[i]);
		if (helperMatch) {
			const name = helperMatch[1];
			const startLine = i + 1;
			let endLine = startLine;
			let brace = 0;
			let seenBrace = false;
			for (let j = i; j < lines.length; j++) {
				const clean = stripLineComment(lines[j]);
				for (const ch of clean) {
					if (ch === '{') {
						brace++;
						seenBrace = true;
					} else if (ch === '}') {
						brace--;
					}
				}
				if (seenBrace && brace <= 0) {
					endLine = j + 1;
					i = j;
					break;
				}
			}
			helpers.push({ name, startLine, endLine });
		}
	}

	for (let i = 0; i < lines.length; i++) {
		const cleanFirst = stripLineComment(lines[i]);
		const caseMatch = caseStartRe.exec(cleanFirst);
		if (!caseMatch) continue;
		const states = caseMatch[1]
			.split(',')
			.map((s) => s.trim().split('.').pop()?.trim() ?? '')
			.filter(Boolean);
		if (!states.length) continue;

		const startLine = i + 1;
		let endLine = startLine;
		const arrowIdx = cleanFirst.indexOf('->');
		const afterCase = arrowIdx >= 0 ? cleanFirst.slice(arrowIdx + 2) : cleanFirst;
		const bodyLines: string[] = [];

		if (afterCase.includes('{')) {
			let brace = 0;
			let seen = false;
			for (let j = i; j < lines.length; j++) {
				const clean = stripLineComment(lines[j]);
				bodyLines.push(clean);
				for (const ch of clean) {
					if (ch === '{') {
						brace++;
						seen = true;
					} else if (ch === '}') {
						brace--;
					}
				}
				if (seen && brace <= 0) {
					endLine = j + 1;
					break;
				}
			}
		} else {
			for (let j = i; j < lines.length; j++) {
				const clean = stripLineComment(lines[j]);
				bodyLines.push(clean);
				if (clean.includes(';')) {
					endLine = j + 1;
					break;
				}
			}
		}
		cases.push({ states, startLine, endLine, body: bodyLines.join('\n') });
	}

	const stateToFields = new Map<string, Set<string>>();
	for (const c of cases) {
		driveCallRe.lastIndex = 0;
		let m: RegExpExecArray | null;
		while ((m = driveCallRe.exec(c.body)) !== null) {
			const fieldName = m[1];
			for (const st of c.states) {
				let set = stateToFields.get(st);
				if (!set) {
					set = new Set();
					stateToFields.set(st, set);
				}
				set.add(fieldName);
			}
		}
	}

	return rawSegments.map((seg, index) => {
		const relatedSpans: SourceSpan[] = [
			{
				kind: 'DECLARATION',
				name: seg.fieldName,
				startLine: seg.startLine,
				endLine: seg.endLine,
			},
		];
		for (const h of helpers) {
			const callRe = new RegExp(`\\b${h.name}\\s*\\(`);
			if (callRe.test(seg.body)) {
				relatedSpans.push({
					kind: 'HELPER_METHOD',
					name: h.name,
					startLine: h.startLine,
					endLine: h.endLine,
				});
			}
		}
		for (const c of cases) {
			const matchedState = c.states.find((st) => stateToFields.get(st)?.has(seg.fieldName));
			if (matchedState) {
				relatedSpans.push({
					kind: 'STATE_BRANCH',
					name: matchedState,
					startLine: c.startLine,
					endLine: c.endLine,
				});
			}
		}
		return {
			id: seg.id,
			fieldName: seg.fieldName,
			index,
			declarationStartLine: seg.startLine,
			declarationEndLine: seg.endLine,
			relatedSpans,
		};
	});
}

/**
 * Resolves the target preview scope (`SEGMENT` vs `WHOLE_AUTO`) and `segmentId` from the editor
 * cursor line (1-based) and whether `Whole Auto` mode is pinned.
 */
export function resolveCursorTarget(
	segments: readonly SegmentDescriptor[],
	cursorLine: number,
	pinnedWholeAuto: boolean,
	previousSegmentId?: string | null,
): { scope: 'SEGMENT' | 'WHOLE_AUTO'; segmentId: string | null } {
	if (pinnedWholeAuto || cursorLine <= 0 || segments.length === 0) {
		return { scope: 'WHOLE_AUTO', segmentId: null };
	}

	for (const seg of segments) {
		if (
			seg.declarationStartLine > 0 &&
			cursorLine >= seg.declarationStartLine &&
			cursorLine <= seg.declarationEndLine
		) {
			return { scope: 'SEGMENT', segmentId: seg.id };
		}
	}

	const stateMatches = segments.filter((seg) =>
		seg.relatedSpans.some((sp) => sp.kind === 'STATE_BRANCH' && cursorLine >= sp.startLine && cursorLine <= sp.endLine),
	);
	if (stateMatches.length === 1) {
		return { scope: 'SEGMENT', segmentId: stateMatches[0].id };
	}
	if (stateMatches.length > 1) {
		const prior = stateMatches.find((s) => s.id === previousSegmentId || s.fieldName === previousSegmentId);
		return { scope: 'SEGMENT', segmentId: (prior ?? stateMatches[0]).id };
	}

	const helperMatches = segments.filter((seg) =>
		seg.relatedSpans.some(
			(sp) => sp.kind === 'HELPER_METHOD' && cursorLine >= sp.startLine && cursorLine <= sp.endLine,
		),
	);
	if (helperMatches.length === 1) {
		return { scope: 'SEGMENT', segmentId: helperMatches[0].id };
	}
	if (helperMatches.length > 1) {
		const prior = helperMatches.find((s) => s.id === previousSegmentId || s.fieldName === previousSegmentId);
		return { scope: 'SEGMENT', segmentId: (prior ?? helperMatches[0]).id };
	}

	return { scope: 'WHOLE_AUTO', segmentId: null };
}

/**
 * Retrieves the Git baseline Java source for `filePath` from `HEAD` (when modified in working tree)
 * or from the merge-base with `origin/main` / `main` for branch diff comparison.
 */
export function resolveGitBaselineSource(
	workspaceRoot: string,
	filePath: string,
	currentSource?: string,
): string | null {
	const relPath = path.relative(workspaceRoot, filePath).split(path.sep).join('/');
	const tryGitShow = (ref: string): string | null => {
		try {
			return execFileSync('git', ['show', `${ref}:${relPath}`], {
				cwd: workspaceRoot,
				encoding: 'utf8',
				stdio: ['ignore', 'pipe', 'ignore'],
			});
		} catch {
			return null;
		}
	};

	const headSource = tryGitShow('HEAD');
	if (headSource && currentSource && headSource.trim() !== currentSource.trim()) {
		return headSource;
	}

	for (const baseBranch of ['origin/main', 'main', 'HEAD~1']) {
		try {
			const mergeBase = execFileSync('git', ['merge-base', 'HEAD', baseBranch], {
				cwd: workspaceRoot,
				encoding: 'utf8',
				stdio: ['ignore', 'pipe', 'ignore'],
			}).trim();
			if (mergeBase) {
				const baseSource = tryGitShow(mergeBase);
				if (baseSource) return baseSource;
			}
		} catch {
			// try next ref
		}
	}

	return headSource;
}
