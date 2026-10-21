/* Document renderer only: preserves LaTeX source and emits offline MathML.
 * Install local build dependencies with:
 * npm --prefix build/doc-math install --no-audit --no-fund --ignore-scripts --save-exact katex@0.19.0 marked@18.1.1
 * Then run: node tools/docs/render_math_guides.cjs
 */
const fs = require('fs');
const path = require('path');
const {pathToFileURL}=require('url');
const root=path.resolve(__dirname,'..','..');
const katex=require(path.join(root,'build/doc-math/node_modules/katex'));

async function main() {
  const {marked}=await import(pathToFileURL(path.join(root,'build/doc-math/node_modules/marked/lib/marked.esm.js')).href);
  marked.use({renderer:{heading({tokens,depth}) {
    const text=this.parser.parseInline(tokens);
    const slug=text.replace(/<[^>]*>/g,'').toLowerCase().replace(/[^\p{L}\p{N}\s-]/gu,'').trim().replace(/\s+/g,'-');
    return '<h'+depth+' id="'+slug+'">'+text+'</h'+depth+'>\n';
  }}});
  const guides=[
    ['docs/current-model-and-fc-integration.md','docs/current-model-and-fc-integration.html'],
    ['measurements/README.md','measurements/guide.html']
  ];
  let total=0;
  for(const [source,target] of guides) {
    const sourcePath=path.join(root,source);
    const md=fs.readFileSync(sourcePath,'utf8');
    const blocks=[];
    function render(tex,displayMode) {
      total++;
      return katex.renderToString(tex,{displayMode,output:'mathml',throwOnError:true,strict:'error',trust:false});
    }
    // Extract math before Markdown can interpret backslashes or matrix rows.
    let tokenized=md.replace(/^\$\$\r?\n([\s\S]*?)\r?\n\$\$\s*$/gm,(_,tex)=>{
      const id=blocks.length;
      blocks.push('<div class="equation" role="group" aria-label="Displayed equation">'+render(tex,true)+'</div>');
      return '\n\nQCOPTERDISPLAYMATH'+id+'END\n\n';
    });
    tokenized=tokenized.replace(/(?<!\$)\$([^$\n]+)\$(?!\$)/g,(_,tex)=>render(tex,false));
    let body=marked.parse(tokenized);
    body=body.replace(/<p>QCOPTERDISPLAYMATH(\d+)END<\/p>/g,(_,id)=>blocks[Number(id)]);
    // Link the two offline reading views to each other.
    body=body.replaceAll('../measurements/README.md','../measurements/guide.html')
      .replaceAll('../docs/current-model-and-fc-integration.md','../docs/current-model-and-fc-integration.html');
    const title=md.split('\n')[0].replace(/^#\s*/,'');
    const css=':root{color-scheme:light}*{box-sizing:border-box}body{margin:0;background:#edf2f6;color:#17324d;font:17px/1.7 system-ui,sans-serif}main{max-width:1120px;margin:32px auto;background:white;padding:48px 64px;border-radius:16px;box-shadow:0 10px 35px #17324d12}h1{font-size:2.2rem;line-height:1.2}h2{margin-top:2.4em;padding-bottom:.3em;border-bottom:2px solid #dbe9ee}h3{margin-top:1.8em}a{color:#007b83}table{border-collapse:collapse;width:100%;font-size:.91rem;margin:1.5em 0;display:block;overflow:auto}th,td{padding:11px 14px;text-align:left;border-bottom:1px solid #dbe3e9;vertical-align:top}th{background:#e8f2f4}blockquote{margin:24px 0;padding:12px 22px;background:#e8f2f4;border-left:4px solid #007f87}.equation{overflow-x:auto;padding:24px 12px;margin:24px 0;background:#f5f8fa;border:1px solid #e0e9ed;border-radius:8px;color:#142b43}.equation math{font-size:1.2em}math[display=block]{margin:0}img{max-width:100%}pre{overflow:auto;padding:16px;background:#f0f4f6}hr{border:0;border-top:1px solid #dbe3e9;margin:36px 0}.reading-note{font-size:.9rem;color:#526878} @media(max-width:700px){main{margin:0;padding:24px 18px;border-radius:0}body{font-size:16px}.equation{padding:18px 8px}} @media print{body{background:white}main{margin:0;padding:0;box-shadow:none}.equation{break-inside:avoid}h2,h3{break-after:avoid}}';
    const html='<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+title+'</title><style>'+css+'</style></head><body><main><p class="reading-note">Offline mathematical reading view · Equations authored in LaTeX, rendered as native MathML · <a href="'+path.basename(source)+'">Markdown source</a></p>'+body+'</main></body></html>\n';
    fs.writeFileSync(path.join(root,target),html);
    console.log(source+': '+blocks.length+' displayed equations → '+target);
  }
  console.log('Rendered '+total+' LaTeX expressions with strict parsing; no external scripts, fonts or network required by the HTML.');
}
main().catch(e=>{console.error(e);process.exitCode=1;});
