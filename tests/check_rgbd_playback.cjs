// Offline DOM/canvas contract check of the actual generated gallery script.
// Uses Node's standard library only; no browser permissions or packages.
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const root = path.resolve(process.argv[2]);
const manifest = JSON.parse(fs.readFileSync(path.join(root, 'manifest.json')));
const checked = [];
for (const name of manifest.scene_ids) {
  const folder = path.join(root, name);
  const html = fs.readFileSync(path.join(folder, 'index.html'), 'utf8');
  if (manifest.clutter_counts) {
    assert.ok(html.includes(manifest.clutter_counts[name]+' clutter obstacle'), 'display configured clutter count separately from target');
  }
  const code = html.match(/<script>([\s\S]*?)<\/script>/)[1];
  const elements = {}, images = [], scheduled = [], draws = [];
  for (const match of html.matchAll(/\bid="([^"]+)"/g)) {
    const id = match[1];
    elements[id] = {value: id === 'speed' ? '1' : '0', textContent: '',
      getContext() {return {drawImage(...args) {draws.push({id,args});}};}};
  }
  elements.scrub.max = html.match(/id="scrub"[^>]*max="(\d+)"/)[1];
  class ImageDouble {
    constructor() {this.complete=false; images.push(this);}
    set src(value) {this.source=value; this.complete=true;}
  }
  const sandbox = {document: {getElementById(id) {assert.ok(elements[id], id); return elements[id];}},
    Image: ImageDouble, requestAnimationFrame(fn) {scheduled.push(fn);}};
  vm.createContext(sandbox); vm.runInContext(code, sandbox);
  for (const image of images) image.onload();
  const data = vm.runInContext('data', sandbox);
  const fps = vm.runInContext('fps', sandbox);
  assert.equal(Object.keys(data).length, 5);
  const maximum = Number(elements.scrub.max);
  // Exercise every shared timeline tick, including held final frames.
  for (let tick=0; tick<=maximum; tick++) {
    elements.scrub.value=String(tick); elements.scrub.oninput();
    assert.equal(elements.clock.textContent,(tick/fps).toFixed(2)+' s');
    for (const [mode, values] of Object.entries(data)) {
      assert.ok(elements[mode+'_live'].textContent.startsWith(values.phase[tick]+' · '+values.pairs[tick]+' pairs'));
    }
  }
  elements.play.onclick(); elements.restart.onclick();
  assert.equal(elements.clock.textContent,'0.00 s');
  elements.speed.value='.5';
  const tick=scheduled[0]; tick(1000); tick(1250); tick(1500);
  assert.equal(elements.clock.textContent,(1/fps).toFixed(2)+' s');
  elements.play.onclick(); const paused=elements.clock.textContent; tick(1750);
  assert.equal(elements.clock.textContent,paused);
  const dimensions=new Map();
  for (const {args} of draws) {
    const [image,x,y,w,h] = args;
    if (!dimensions.has(image.source)) {
      const png=fs.readFileSync(path.join(folder,image.source));
      dimensions.set(image.source,[png.readUInt32BE(16),png.readUInt32BE(20)]);
    }
    const [width,height]=dimensions.get(image.source);
    assert.ok(x>=0 && y>=0 && x+w<=width && y+h<=height,'atlas rectangle bounds');
  }
  checked.push({scene:name,methods:5,shared_timeline_ticks:maximum+1,canvas_draws:draws.length,
    controls:['pause','play','restart','scrub','speed'],all_atlas_rectangles_valid:true});
}
process.stdout.write(JSON.stringify({kind:'Offline generated-script DOM/canvas contract; not a live browser QA claim',checked},null,2)+'\n');
