"""Generate a compact replay and Monte Carlo distribution visualization."""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path


if len(sys.argv) != 4:
    raise SystemExit("Usage: python make_uncertainty_dashboard.py <kalman.csv> <mass.csv> <output.html>")

kalman_path, mass_path, out_path = map(Path, sys.argv[1:])
with kalman_path.open(newline="", encoding="utf-8") as f:
    kf = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]
with mass_path.open(newline="", encoding="utf-8") as f:
    mass_rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]
if not kf or not mass_rows:
    raise SystemExit("Missing uncertainty samples")

def hist(values: list[float], bins: int = 42) -> list[dict[str, float]]:
    low, high = min(values), max(values)
    width = (high-low)/bins
    counts = [0]*bins
    for value in values:
        index = min(bins-1, int((value-low)/width))
        counts[index] += 1
    n = len(values)
    return [{"x": low+(i+0.5)*width, "density": count/(n*width)} for i,count in enumerate(counts)]

stats = json.loads(mass_path.with_name("mass-inertia-summary.json").read_text(encoding="utf-8"))
data = {
    "kf": kf,
    "mass_hist": hist([r["known_subassembly_mass_kg"] for r in mass_rows]),
    "inertia_hist": {key: hist([r[key] for r in mass_rows]) for key in ("Ixx_kg_m2","Iyy_kg_m2","Izz_kg_m2")},
    "mass_stats": stats["properties"]["known_subassembly_mass_kg"],
    "inertia_stats": {key: stats["properties"][key] for key in ("Ixx_kg_m2","Iyy_kg_m2","Izz_kg_m2")},
}
payload = json.dumps(data, separators=(",", ":"))
fragment = r'''<style>
#qcopter-uncertainty .qcopter-legend { display:flex; flex-wrap:wrap; gap:0.25rem 0.75rem; margin:0.25rem 0; }
#qcopter-uncertainty .qcopter-legend button { appearance:none; background:transparent; border:0; padding:4px 0; color:var(--foreground); display:inline-flex; align-items:center; gap:0.35rem; }
#qcopter-uncertainty .qcopter-swatch { display:inline-block; width:0.7rem; height:0.7rem; }
#qcopter-uncertainty svg { display:block; width:100%; }
@media (pointer:coarse) { #qcopter-uncertainty .qcopter-legend button { min-height:44px; } }
</style>
<div id="qcopter-uncertainty">
  <h2>F450 uncertainty and Kalman replay</h2>
  <div class="viz-controls">
    <button id="uq-play" type="button" aria-pressed="false">Play 5 s replay</button>
    <label class="form-label" for="uq-time">Trace time <output id="uq-time-label">0.000 s</output></label>
    <input id="uq-time" class="form-range" type="range" min="0" max="__LAST__" step="0.02" value="0" aria-label="Kalman Monte Carlo trace time">
  </div>
  <p id="uq-state" aria-live="polite">Recorded C Kalman Monte Carlo trace</p>
  <h3>Roll error and uncertainty bands</h3>
  <svg id="uq-kalman" role="img" aria-label="Monte Carlo roll estimation error distribution and predicted Kalman standard deviation over time"></svg>
  <h3>Known-subassembly mass prior</h3>
  <svg id="uq-mass" role="img" aria-label="Monte Carlo probability density of the modeled F450 frame and battery mass only"></svg>
  <h3>Inertia prior distributions</h3>
  <svg id="uq-inertia" role="img" aria-label="Monte Carlo probability densities for roll, pitch, and yaw-axis inertia of the provisional frame and battery model"></svg>
  <p class="text-muted">500 seeded trials through the production C roll Kalman update. Physical priors are provisional; motors, ESCs, electronics, and payload are excluded from mass and inertia.</p>
</div>
<script src="https://cdn.jsdelivr.net/npm/d3@7.9.0/dist/d3.min.js"></script>
<script>
(() => {
  const root=document.getElementById('qcopter-uncertainty');
  if(!root||typeof d3==='undefined') return;
  const d=__DATA__, points=d.kf, last=points[points.length-1].time_s;
  const slider=root.querySelector('#uq-time'), label=root.querySelector('#uq-time-label');
  const state=root.querySelector('#uq-state'), button=root.querySelector('#uq-play');
  const colors={mean:'var(--viz-series-1)',emp:'var(--viz-series-2)',kal:'var(--viz-series-3)',ixx:'var(--viz-series-4)',iyy:'var(--viz-series-5)',izz:'var(--viz-series-6)'};
  const enabled=new Set(Object.keys(colors));
  const bisect=d3.bisector(p=>p.time_s).center;
  function setup(selector,height,ylabel){
    const svg=d3.select(root.querySelector(selector)), width=Math.max(320,svg.node().parentElement.clientWidth);
    const m={top:12,right:16,bottom:46,left:72};
    svg.attr('viewBox',`0 0 ${width} ${height}`).attr('height',height).selectAll('*').remove();
    const plot=svg.append('g');
    svg.append('text').attr('class','axis-title').attr('data-axis','y').attr('fill','var(--foreground)').attr('font-size',12).attr('transform',`translate(16 ${(m.top+height-m.bottom)/2}) rotate(-90)`).attr('text-anchor','middle').text(ylabel);
    return {svg,plot,width,height,m};
  }
  function axes(c,x,y,xticks=5){
    c.plot.append('g').attr('transform',`translate(0,${c.height-c.m.bottom})`).call(d3.axisBottom(x).ticks(c.width<480?3:xticks));
    c.plot.append('g').attr('transform',`translate(${c.m.left},0)`).call(d3.axisLeft(y).ticks(4));
    c.plot.insert('rect',':first-child').attr('data-chart-frame','').attr('x',c.m.left).attr('y',c.m.top).attr('width',c.width-c.m.left-c.m.right).attr('height',c.height-c.m.top-c.m.bottom).attr('fill','transparent').attr('stroke','var(--border)');
    c.svg.selectAll('.domain,.tick line').attr('stroke','var(--border)');
    c.svg.selectAll('.tick text').attr('fill','var(--muted-foreground)').attr('font-size',12);
    c.svg.append('text').attr('class','axis-title').attr('data-axis','x').attr('fill','var(--foreground)').attr('font-size',12).attr('x',(c.m.left+c.width-c.m.right)/2).attr('y',c.height-6).attr('text-anchor','middle').text('Time (s)');
  }
  function drawKalman(t){
    const c=setup('#uq-kalman',290,'Roll error / standard deviation (°)');
    const x=d3.scaleLinear().domain(d3.extent(points,p=>p.time_s)).range([c.m.left,c.width-c.m.right]);
    const bands=points.flatMap(p=>[p.mean_error_deg-1.96*p.empirical_error_std_deg,p.mean_error_deg+1.96*p.empirical_error_std_deg,-2*p.kalman_std_deg,2*p.kalman_std_deg]);
    const [lo,hi]=d3.extent(bands), pad=Math.max((hi-lo)*0.08,0.01);
    const y=d3.scaleLinear().domain([lo-pad,hi+pad]).nice().range([c.height-c.m.bottom,c.m.top]); axes(c,x,y);
    c.plot.insert('rect',':first-child').attr('data-chart-frame','').attr('x',c.m.left).attr('y',c.m.top).attr('width',c.width-c.m.left-c.m.right).attr('height',c.height-c.m.top-c.m.bottom).attr('fill','transparent').attr('stroke','var(--border)');
    if(enabled.has('emp')) c.plot.append('path').datum(points).attr('fill','var(--viz-series-2)').attr('fill-opacity',0.10)
      .attr('d',d3.area().x(p=>x(p.time_s)).y0(p=>y(p.mean_error_deg-1.96*p.empirical_error_std_deg)).y1(p=>y(p.mean_error_deg+1.96*p.empirical_error_std_deg)));
    if(enabled.has('kal')) c.plot.append('path').datum(points).attr('fill','var(--viz-series-3)').attr('fill-opacity',0.10)
      .attr('d',d3.area().x(p=>x(p.time_s)).y0(p=>y(-2*p.kalman_std_deg)).y1(p=>y(2*p.kalman_std_deg)));
    const line=(key)=>d3.line().x(p=>x(p.time_s)).y(p=>y(p[key]));
    if(enabled.has('mean')) c.plot.append('path').datum(points).attr('fill','none').attr('stroke',colors.mean).attr('stroke-width',2).attr('d',line('mean_error_deg'));
    if(enabled.has('emp')) c.plot.append('path').datum(points).attr('fill','none').attr('stroke',colors.emp).attr('stroke-width',1.5).attr('d',line('empirical_error_std_deg'));
    if(enabled.has('kal')) c.plot.append('path').datum(points).attr('fill','none').attr('stroke',colors.kal).attr('stroke-width',1.5).attr('d',line('kalman_std_deg'));
    const xp=x(t), p=points[bisect(points,t)];
    c.plot.append('line').attr('x1',xp).attr('x2',xp).attr('y1',c.m.top).attr('y2',c.height-c.m.bottom).attr('stroke','var(--foreground)').attr('stroke-opacity',0.5);
    for(const key of ['mean_error_deg','empirical_error_std_deg','kalman_std_deg']) {const series=key==='mean_error_deg'?'mean':key==='empirical_error_std_deg'?'emp':'kal';if(enabled.has(series)) c.plot.append('circle').attr('cx',xp).attr('cy',y(p[key])).attr('r',3.5).attr('fill',colors[series]).attr('data-tooltip',`${key.replaceAll('_',' ')}: ${p[key].toFixed(4)}°`);}
    state.textContent=`t=${t.toFixed(2)} s · mean error=${p.mean_error_deg.toFixed(3)}° · empirical error σ=${p.empirical_error_std_deg.toFixed(3)}° · Kalman σ=${p.kalman_std_deg.toFixed(3)}° · ±1σ coverage=${(100*p.coverage_1sigma).toFixed(1)}% · ±2σ coverage=${(100*p.coverage_2sigma).toFixed(1)}%`;
  }
  function drawDensity(selector,height,ylabel,series,xlabel){
    const c=setup(selector,height,ylabel); const arrays=series.map(s=>s.values);
    const all=arrays.flat(), [lo,hi]=d3.extent(all), pad=(hi-lo)*0.04;
    const x=d3.scaleLinear().domain([lo-pad,hi+pad]).range([c.m.left,c.width-c.m.right]);
    const max=d3.max(all); const y=d3.scaleLinear().domain([0,max*1.08]).nice().range([c.height-c.m.bottom,c.m.top]);
    c.plot.append('g').attr('transform',`translate(0,${c.height-c.m.bottom})`).call(d3.axisBottom(x).ticks(c.width<480?3:5));
    c.plot.append('g').attr('transform',`translate(${c.m.left},0)`).call(d3.axisLeft(y).ticks(4));
    c.plot.insert('rect',':first-child').attr('data-chart-frame','').attr('x',c.m.left).attr('y',c.m.top).attr('width',c.width-c.m.left-c.m.right).attr('height',c.height-c.m.top-c.m.bottom).attr('fill','transparent').attr('stroke','var(--border)');
    c.svg.selectAll('.domain,.tick line').attr('stroke','var(--border)'); c.svg.selectAll('.tick text').attr('fill','var(--muted-foreground)').attr('font-size',12);
    c.svg.append('text').attr('class','axis-title').attr('data-axis','x').attr('fill','var(--foreground)').attr('font-size',12).attr('x',(c.m.left+c.width-c.m.right)/2).attr('y',height-6).attr('text-anchor','middle').text(xlabel);
    series.filter(s=>enabled.has(s.key)).forEach(s=>c.plot.append('path').datum(s.values).attr('fill','none').attr('stroke',s.color).attr('stroke-width',2).attr('d',d3.line().x(p=>x(p.x)).y(p=>y(p.density))));
  }
  const legend=root.appendChild(document.createElement('div')); legend.className='qcopter-legend';
  const legendItems=[['mean','Mean error'],['emp','Empirical ±1σ / band'],['kal','Kalman predicted ±1σ / band'],['ixx','Ixx roll inertia'],['iyy','Iyy pitch inertia'],['izz','Izz yaw inertia']];
  for(const [key,name] of legendItems){const b=document.createElement('button');b.type='button';b.setAttribute('aria-pressed','true');b.innerHTML=`<span class="qcopter-swatch" style="background:${colors[key]}"></span>${name}`;b.addEventListener('click',()=>{if(enabled.has(key))enabled.delete(key);else enabled.add(key);b.setAttribute('aria-pressed',String(enabled.has(key)));drawAll(Number(slider.value));});legend.append(b);}
  function drawAll(t){slider.value=String(t);label.value=`${t.toFixed(2)} s`;drawKalman(t);
    drawDensity('#uq-mass',220,'Probability density (1/kg)',[{key:'mean',values:d.mass_hist,color:colors.mean}], 'Known-subassembly mass (kg)');
    drawDensity('#uq-inertia',250,'Probability density (1/(kg·m²))',[
      {key:'ixx',values:d.inertia_hist.Ixx_kg_m2,color:colors.ixx},{key:'iyy',values:d.inertia_hist.Iyy_kg_m2,color:colors.iyy},{key:'izz',values:d.inertia_hist.Izz_kg_m2,color:colors.izz}], 'Moment of inertia (kg·m²)');
  }
  let timer=null; function stop(){if(timer)clearInterval(timer);timer=null;button.setAttribute('aria-pressed','false');button.textContent='Play 5 s replay';}
  slider.addEventListener('input',()=>{stop();drawAll(Number(slider.value));});
  button.addEventListener('click',()=>{if(timer){stop();return;}if(Number(slider.value)>=last)drawAll(0);button.setAttribute('aria-pressed','true');button.textContent='Pause replay';timer=setInterval(()=>{const n=Number(slider.value)+0.02;if(n>=last){drawAll(last);stop();}else drawAll(n);},20);});
  new ResizeObserver(()=>drawAll(Number(slider.value))).observe(root); drawAll(0);
})();
</script>'''
fragment = fragment.replace("__LAST__", f"{float(kf[-1]['time_s']):.3f}").replace("__DATA__", payload)
out_path.parent.mkdir(parents=True, exist_ok=True)
out_path.write_text(fragment, encoding="utf-8", newline="\n")
print(f"Wrote {out_path} ({out_path.stat().st_size} bytes)")
