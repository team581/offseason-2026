const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { WarmPreviewWorkerClient } = require('../worker.ts');

void test(
	'rebuilds stale dependencies and restarts the JVM while keeping auto edits warm',
	{ skip: process.platform === 'win32' },
	async (t) => {
		const root = fs.mkdtempSync(path.join(os.tmpdir(), 'tb3-worker-refresh-'));
		t.after(() => fs.rmSync(root, { recursive: true, force: true }));
		const auto = path.join(root, 'comp-bot/src/main/java/frc/robot/autos/ExampleAuto.java');
		const helper = path.join(root, 'comp-bot/src/main/java/frc/robot/config/CompAutoField.java');
		fs.mkdirSync(path.dirname(auto), { recursive: true });
		fs.mkdirSync(path.dirname(helper), { recursive: true });
		fs.mkdirSync(path.join(root, 'jdk/bin'), { recursive: true });
		fs.mkdirSync(path.join(root, '.gradle/trailblazer-preview'), { recursive: true });
		fs.writeFileSync(auto, 'original auto');
		fs.writeFileSync(helper, 'fromRed v1');
		fs.writeFileSync(
			path.join(root, '.gradle/trailblazer-preview/classpath.manifest'),
			JSON.stringify({
				buildFilesHash: 'unchanged',
				classpathEntries: [root],
				jniLibraryPath: '',
				sourceRoots: [],
			}),
		);
		fs.writeFileSync(
			path.join(root, 'gradlew'),
			`#!${process.execPath}
const fs = require('node:fs');
const count = fs.existsSync('build-count') ? Number(fs.readFileSync('build-count', 'utf8')) : 0;
fs.writeFileSync('build-count', String(count + 1));
`,
			{ mode: 0o755 },
		);
		fs.writeFileSync(
			path.join(root, 'jdk/bin/java'),
			`#!${process.execPath}
const fs = require('node:fs');
const helper = fs.readFileSync(${JSON.stringify(helper)}, 'utf8');
console.log(JSON.stringify({type:'ready'}));
require('node:readline').createInterface({input:process.stdin}).on('line', line => {
 const request = JSON.parse(line);
 if (request.method === 'shutdown') process.exit(0);
 console.log(JSON.stringify({id:request.id,type:'result',ok:true,helper}));
});
`,
			{ mode: 0o755 },
		);
		const client = new WarmPreviewWorkerClient(root, () => path.join(root, 'jdk'));
		t.after(() => client.dispose());
		const params = { sourceFile: auto, scope: 'WHOLE_AUTO', alliance: 'Blue', robotConfigId: '581' };
		assert.equal((await client.requestPreview(params)).helper, 'fromRed v1');
		assert.equal(fs.readFileSync(path.join(root, 'build-count'), 'utf8'), '1');
		fs.writeFileSync(auto, 'edited waypoint');
		assert.equal((await client.requestPreview(params)).helper, 'fromRed v1');
		assert.equal(fs.readFileSync(path.join(root, 'build-count'), 'utf8'), '1');
		fs.writeFileSync(helper, 'fromRed v2');
		assert.equal((await client.requestPreview(params)).helper, 'fromRed v2');
		assert.equal(fs.readFileSync(path.join(root, 'build-count'), 'utf8'), '2');
		client.dispose();
		const reopened = new WarmPreviewWorkerClient(root, () => path.join(root, 'jdk'));
		t.after(() => reopened.dispose());
		assert.equal((await reopened.requestPreview(params)).helper, 'fromRed v2');
		assert.equal(fs.readFileSync(path.join(root, 'build-count'), 'utf8'), '2');
	},
);

void test(
	'serializes concurrent ensureStarted and requestPreview calls without killing an in-flight spawn',
	{ skip: process.platform === 'win32' },
	async (t) => {
		const root = fs.mkdtempSync(path.join(os.tmpdir(), 'tb3-worker-race-'));
		t.after(() => fs.rmSync(root, { recursive: true, force: true }));
		const auto = path.join(root, 'comp-bot/src/main/java/frc/robot/autos/ExampleAuto.java');
		fs.mkdirSync(path.dirname(auto), { recursive: true });
		fs.mkdirSync(path.join(root, 'jdk/bin'), { recursive: true });
		fs.mkdirSync(path.join(root, '.gradle/trailblazer-preview'), { recursive: true });
		fs.writeFileSync(auto, 'original auto');
		fs.writeFileSync(
			path.join(root, '.gradle/trailblazer-preview/classpath.manifest'),
			JSON.stringify({
				buildFilesHash: 'unchanged',
				classpathEntries: [root],
				jniLibraryPath: '',
				sourceRoots: [],
			}),
		);
		fs.writeFileSync(
			path.join(root, 'gradlew'),
			`#!${process.execPath}
const fs = require('node:fs');
setTimeout(() => {
 const count = fs.existsSync('build-count') ? Number(fs.readFileSync('build-count', 'utf8')) : 0;
 fs.writeFileSync('build-count', String(count + 1));
}, 40);
`,
			{ mode: 0o755 },
		);
		fs.writeFileSync(
			path.join(root, 'jdk/bin/java'),
			`#!${process.execPath}
setTimeout(() => console.log(JSON.stringify({type:'ready'})), 40);
require('node:readline').createInterface({input:process.stdin}).on('line', line => {
 const request = JSON.parse(line);
 if (request.method === 'shutdown') process.exit(0);
 console.log(JSON.stringify({id:request.id,type:'result',ok:true}));
});
`,
			{ mode: 0o755 },
		);

		const client = new WarmPreviewWorkerClient(root, () => path.join(root, 'jdk'));
		t.after(() => client.dispose());
		const prewarm = client.ensureStarted();
		const req1 = client.requestPreview({
			sourceFile: auto,
			scope: 'WHOLE_AUTO',
			alliance: 'Blue',
			robotConfigId: '581',
		});
		const req2 = client.requestPreview({
			sourceFile: auto,
			scope: 'WHOLE_AUTO',
			alliance: 'Red',
			robotConfigId: '581',
		});

		const [prewarmRes, r1Res, r2Res] = await Promise.allSettled([prewarm, req1, req2]);
		assert.equal(prewarmRes.status, 'fulfilled');
		assert.equal(r1Res.status, 'rejected');
		assert.match(r1Res.reason.message, /^Superseded by req-/);
		assert.equal(r2Res.status, 'fulfilled');
		assert.equal(r2Res.value.ok, true);
		assert.equal(fs.readFileSync(path.join(root, 'build-count'), 'utf8'), '1');
	},
);
