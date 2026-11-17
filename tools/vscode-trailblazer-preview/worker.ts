import { type ChildProcessWithoutNullStreams, execFileSync, spawn } from 'node:child_process';
import * as fs from 'node:fs';
import * as os from 'node:os';
import * as path from 'node:path';
import * as readline from 'node:readline';
import type { WorkerResponse } from './schema';

export interface ClasspathManifestData {
	buildFilesHash: string;
	classpathEntries: string[];
	jniLibraryPath: string;
	sourceRoots: string[];
}

export interface PreviewWorkerRequestParams {
	sourceFile?: string;
	sourceContent?: string;
	autoClass?: string;
	scope: 'SEGMENT' | 'WHOLE_AUTO' | 'CURSOR';
	segmentId?: string | null;
	cursorLine?: number;
	previousSegmentId?: string | null;
	alliance: 'Blue' | 'Red';
	robotConfigId: '581' | '9991' | '9981';
	seed?: number;
}

/** Resolves the Java 21 JDK binary (`java`) for launching the warm preview worker. */
export function resolveJavaExecutable(javaHomeOverride?: string): string {
	const binName = process.platform === 'win32' ? 'java.exe' : 'java';
	const candidates: string[] = [];

	if (javaHomeOverride && javaHomeOverride.trim()) {
		candidates.push(path.join(javaHomeOverride.trim(), 'bin', binName));
	}
	if (process.env.JAVA_HOME) {
		candidates.push(path.join(process.env.JAVA_HOME, 'bin', binName));
	}
	const wpilibRoot =
		process.platform === 'win32'
			? path.join(process.env.PUBLIC ?? 'C:\\Users\\Public', 'wpilib')
			: path.join(os.homedir(), 'wpilib');
	candidates.push(path.join(wpilibRoot, '2026', 'jdk', 'bin', binName));

	if (process.platform === 'darwin') {
		try {
			const macJavaHome = execFileSync('/usr/libexec/java_home', ['-v', '21'], {
				encoding: 'utf8',
				stdio: ['ignore', 'pipe', 'ignore'],
			}).trim();
			if (macJavaHome) {
				candidates.push(path.join(macJavaHome, 'bin', binName));
			}
		} catch {
			// continue
		}
	}

	for (const candidate of candidates) {
		if (fs.existsSync(candidate)) {
			return candidate;
		}
	}
	return binName;
}

/** Reads `.gradle/trailblazer-preview/classpath.manifest` if present and valid on disk. */
export function readCachedManifest(workspaceRoot: string): ClasspathManifestData | null {
	const manifestFile = path.join(workspaceRoot, '.gradle', 'trailblazer-preview', 'classpath.manifest');
	if (!fs.existsSync(manifestFile)) return null;
	try {
		const parsed = JSON.parse(fs.readFileSync(manifestFile, 'utf8')) as ClasspathManifestData;
		if (!Array.isArray(parsed.classpathEntries) || parsed.classpathEntries.length === 0) {
			return null;
		}
		const existingEntries = parsed.classpathEntries.filter((entry) => fs.existsSync(entry));
		if (existingEntries.length === 0) {
			return null;
		}
		return { ...parsed, classpathEntries: existingEntries };
	} catch {
		return null;
	}
}

/** Runs `./gradlew writePreviewClasspathManifest` to generate or refresh the classpath manifest. */
export async function generateManifestViaGradle(workspaceRoot: string): Promise<ClasspathManifestData> {
	const isWin = process.platform === 'win32';
	const wrapper = path.join(workspaceRoot, isWin ? 'gradlew.bat' : 'gradlew');
	const cmd = isWin ? 'cmd.exe' : wrapper;
	const args = isWin
		? ['/c', wrapper, 'writePreviewClasspathManifest', '--console=plain', '-q']
		: ['writePreviewClasspathManifest', '--console=plain', '-q'];

	await new Promise<void>((resolve, reject) => {
		const child = spawn(cmd, args, { cwd: workspaceRoot, stdio: ['ignore', 'pipe', 'pipe'] });
		let stderr = '';
		child.stderr.on('data', (chunk: Buffer) => {
			stderr += chunk.toString('utf8');
		});
		child.on('error', reject);
		child.on('close', (code) => {
			if (code === 0) resolve();
			else reject(new Error(`Gradle manifest task exited with code ${String(code)}: ${stderr.trim()}`));
		});
	});

	const manifest = readCachedManifest(workspaceRoot);
	if (!manifest) {
		throw new Error('Failed to load .gradle/trailblazer-preview/classpath.manifest after Gradle refresh');
	}
	return manifest;
}

interface PendingRequest {
	id: string;
	resolve: (response: WorkerResponse) => void;
	reject: (error: Error) => void;
}

/**
 * Manages a persistent Java 21 JDK warm-JVM stdio worker process for a Gradle workspace.
 */
export class WarmPreviewWorkerClient {
	private child: ChildProcessWithoutNullStreams | null = null;
	private rl: readline.Interface | null = null;
	private startingPromise: Promise<void> | null = null;
	private disposed = false;
	private requestCounter = 0;
	private activePending: PendingRequest | null = null;
	public readonly workspaceRoot: string;
	private readonly getJavaHomeOverride?: () => string | undefined;

	public constructor(workspaceRoot: string, getJavaHomeOverride?: () => string | undefined) {
		this.workspaceRoot = workspaceRoot;
		this.getJavaHomeOverride = getJavaHomeOverride;
	}

	public isRunning(): boolean {
		return this.child !== null && this.child.exitCode === null && !this.child.killed;
	}

	/** Pre-warms the Java 21 JDK stdio worker in the background. */
	public async ensureStarted(): Promise<void> {
		if (this.disposed) {
			throw new Error('Preview worker client has been disposed');
		}
		if (this.isRunning()) {
			return;
		}
		if (this.startingPromise) {
			return this.startingPromise;
		}
		this.startingPromise = this.spawnWorker();
		try {
			await this.startingPromise;
		} finally {
			this.startingPromise = null;
		}
	}

	/**
	 * Sends a preview request to the warm-JVM worker, automatically cancelling any superseded
	 * in-flight request.
	 */
	public async requestPreview(params: PreviewWorkerRequestParams): Promise<WorkerResponse> {
		const requestNumber = ++this.requestCounter;
		const reqId = `req-${requestNumber}`;
		if (this.activePending) {
			const superseded = this.activePending;
			this.activePending = null;
			superseded.reject(new Error(`Superseded by ${reqId}`));
		}

		await this.ensureStarted();
		if (requestNumber !== this.requestCounter) {
			throw new Error(`Superseded by req-${this.requestCounter}`);
		}
		if (this.disposed || !this.child) {
			throw new Error('Preview worker is not running');
		}

		return new Promise<WorkerResponse>((resolve, reject) => {
			this.activePending = { id: reqId, resolve, reject };
			const payload = JSON.stringify({
				id: reqId,
				method: 'preview',
				params,
			});
			this.child!.stdin.write(`${payload}\n`, 'utf8', (err) => {
				if (err && this.activePending?.id === reqId) {
					this.activePending = null;
					reject(err);
				}
			});
		});
	}

	/** Cleanly stops the warm-JVM worker process and releases all resources. */
	public dispose(): void {
		this.disposed = true;
		if (this.activePending) {
			this.activePending.reject(new Error('Worker disposed'));
			this.activePending = null;
		}
		if (this.rl) {
			this.rl.close();
			this.rl = null;
		}
		if (this.child) {
			const proc = this.child;
			this.child = null;
			try {
				proc.stdin.write(`${JSON.stringify({ id: 'shutdown', method: 'shutdown' })}\n`);
				proc.stdin.end();
			} catch {
				// ignore
			}
			try {
				proc.kill();
			} catch {
				// ignore
			}
		}
	}

	private async spawnWorker(): Promise<void> {
		let manifest = readCachedManifest(this.workspaceRoot);
		if (!manifest) {
			manifest = await generateManifestViaGradle(this.workspaceRoot);
		}
		if (this.disposed) return;

		const javaBin = resolveJavaExecutable(this.getJavaHomeOverride?.());
		const cpSeparator = process.platform === 'win32' ? ';' : ':';
		const classpath = manifest.classpathEntries.join(cpSeparator);
		const jniDir = manifest.jniLibraryPath;

		const env: NodeJS.ProcessEnv = { ...process.env };
		if (jniDir) {
			env.LD_LIBRARY_PATH = jniDir;
			env.DYLD_LIBRARY_PATH = jniDir;
			env.DYLD_FALLBACK_LIBRARY_PATH = jniDir;
			if (process.platform === 'win32') {
				env.PATH = `${jniDir};${env.PATH ?? ''}`;
			}
		}

		const args = [
			...(jniDir ? [`-Djava.library.path=${jniDir}`] : []),
			'-cp',
			classpath,
			'com.team581.trailblazer3.preview.WarmPreviewWorker',
			'--stdio',
			'--workspace',
			this.workspaceRoot,
		];

		await new Promise<void>((resolve, reject) => {
			let settled = false;
			const proc = spawn(javaBin, args, {
				cwd: this.workspaceRoot,
				env,
				stdio: ['pipe', 'pipe', 'pipe'],
			});
			this.child = proc;

			const rl = readline.createInterface({ input: proc.stdout });
			this.rl = rl;

			rl.on('line', (line: string) => {
				const trimmed = line.trim();
				if (!trimmed.startsWith('{')) return;
				try {
					const msg = JSON.parse(trimmed) as Record<string, unknown>;
					if (msg.type === 'ready') {
						if (!settled) {
							settled = true;
							resolve();
						}
						return;
					}
					if (msg.type === 'error' && msg.code === 'MISSING_JAVA_COMPILER') {
						const err = new Error(typeof msg.message === 'string' ? msg.message : 'Java 21 JDK with javac is required');
						if (!settled) {
							settled = true;
							reject(err);
						}
						if (this.activePending) {
							const pending = this.activePending;
							this.activePending = null;
							pending.reject(err);
						}
						return;
					}
					if (msg.type === 'result' && this.activePending && msg.id === this.activePending.id) {
						const pending = this.activePending;
						this.activePending = null;
						pending.resolve(msg as unknown as WorkerResponse);
					}
				} catch {
					// ignore non-JSON lines
				}
			});

			proc.on('error', (err) => {
				if (!settled) {
					settled = true;
					reject(err);
				}
				if (this.activePending) {
					const pending = this.activePending;
					this.activePending = null;
					pending.reject(err);
				}
			});

			proc.on('exit', (code) => {
				this.child = null;
				if (!settled) {
					settled = true;
					reject(new Error(`Warm preview worker exited prematurely with code ${String(code)}`));
				}
				if (this.activePending) {
					const pending = this.activePending;
					this.activePending = null;
					pending.reject(new Error(`Warm preview worker exited with code ${String(code)}`));
				}
			});
		});
	}
}
