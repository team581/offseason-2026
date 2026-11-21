export interface WebviewAssets {
	cspSource: string;
	nonce: string;
	stylesheet: string;
	script: string;
	fieldImage: string;
}
const escapes: Record<string, string> = { '&': '&amp;', '"': '&quot;', '<': '&lt;', '>': '&gt;' };
const attribute = (value: string) => value.replace(/[&"<>]/g, (char: string) => escapes[char]);
export function renderWebview(template: string, assets: WebviewAssets): string {
	return template.replace(/\{\{(cspSource|nonce|stylesheet|script|fieldImage)\}\}/g, (_, key: keyof WebviewAssets) =>
		attribute(assets[key]),
	);
}
