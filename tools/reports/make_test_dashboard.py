"""Create the inline, replayable test-trace chart from a host-test CSV."""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path


if len(sys.argv) != 3:
    raise SystemExit("Usage: python make_test_dashboard.py <trace.csv> <dashboard.html>")

source = Path(sys.argv[1])
destination = Path(sys.argv[2])
with source.open(newline="", encoding="utf-8") as handle:
    rows = list(csv.DictReader(handle))
if not rows:
    raise SystemExit("Trace CSV has no samples")

columns = (
    "time_s", "setpoint_roll_deg", "accel_roll_deg", "estimated_roll_deg",
    "motor_fl", "motor_fr", "motor_rr", "motor_rl", "armed", "sensor_fault",
)
data = [{key: float(row[key]) if key not in ("armed", "sensor_fault")
         else int(row[key]) for key in columns} for row in rows]
packed = json.dumps(data, separators=(",", ":"))
html = r'''<div id="qcopter-test-replay">
  <h2>F450 controller test trace</h2>
  <div class="viz-controls">
    <button id="qcopter-play" type="button" aria-pressed="false">Play 5 s replay</button>
    <label class="form-label" for="qcopter-time">Trace time <output id="qcopter-time-label">0.000 s</output></label>
    <input id="qcopter-time" class="form-range" type="range" min="0" max="__LAST__" step="0.01" value="0" aria-label="Trace playback time">
  </div>
  <p id="qcopter-state" aria-live="polite">Loading recorded host-test trace…</p>
  <div class="qcopter-legend" aria-label="Plot series"></div>
  <section>
    <svg id="qcopter-attitude" role="img" aria-label="Roll setpoint, synthetic accelerometer angle, and estimated roll over five seconds"></svg>
    <svg id="qcopter-motors" role="img" aria-label="Four normalized mixer outputs over five seconds; controller remains disarmed"></svg>
  </section>
  <p class="text-muted">Recorded 500 Hz host math trace. Sensor values are scripted; no vehicle physics, hardware sensors, RF, or ESC output is simulated.</p>
</div>
<script src="https://cdn.jsdelivr.net/npm/d3@7.9.0/dist/d3.min.js"></script>
<script>
(() => {
  const root = document.getElementById('qcopter-test-replay');
  if (!root || typeof d3 === 'undefined') return;
  const data = __DATA__;
  const last = data[data.length - 1].time_s;
  const timeInput = root.querySelector('#qcopter-time');
  const timeLabel = root.querySelector('#qcopter-time-label');
  const state = root.querySelector('#qcopter-state');
  const playButton = root.querySelector('#qcopter-play');
  const definitions = [
    {key:'setpoint_roll_deg', name:'Roll setpoint', unit:'°'},
    {key:'accel_roll_deg', name:'Synthetic accel angle', unit:'°'},
    {key:'estimated_roll_deg', name:'Kalman roll estimate', unit:'°'},
    {key:'motor_fl', name:'Motor FL', unit:'normalized'},
    {key:'motor_fr', name:'Motor FR', unit:'normalized'},
    {key:'motor_rr', name:'Motor RR', unit:'normalized'},
    {key:'motor_rl', name:'Motor RL', unit:'normalized'}
  ];
  const enabled = new Set(definitions.map(d => d.key));
  const colors = new Map(definitions.map((d,i) => [d.key, `var(--viz-series-${(i%6)+1})`]));
  const charts = [
    {id:'#qcopter-attitude', keys:['setpoint_roll_deg','accel_roll_deg','estimated_roll_deg'], yLabel:'Roll angle (°)'},
    {id:'#qcopter-motors', keys:['motor_fl','motor_fr','motor_rr','motor_rl'], yLabel:'Normalized mixer output (0–1)'}
  ];
  const bisect = d3.bisector(d => d.time_s).center;
  const nearest = t => data[bisect(data,t)];
  const legend = root.querySelector('.qcopter-legend');
  for (const d of definitions) {
    const button = document.createElement('button');
    button.type = 'button';
    button.setAttribute('aria-pressed','true');
    button.innerHTML = `<span class="qcopter-swatch" style="background:${colors.get(d.key)}"></span>${d.name}`;
    button.addEventListener('click', () => {
      if (enabled.has(d.key)) enabled.delete(d.key); else enabled.add(d.key);
      button.setAttribute('aria-pressed', String(enabled.has(d.key)));
      drawAll(Number(timeInput.value));
    });
    legend.append(button);
  }
  function drawChart(chart, t) {
    const svg = d3.select(root.querySelector(chart.id));
    const width = Math.max(320, svg.node().parentElement.clientWidth);
    const height = 250;
    const margin = {top:12,right:18,bottom:46,left:66};
    svg.attr('viewBox', `0 0 ${width} ${height}`).attr('height', height);
    svg.selectAll('*').remove();
    const keys = chart.keys.filter(k => enabled.has(k));
    if (!keys.length) keys.push(chart.keys[0]);
    const values = keys.flatMap(key => data.map(row => row[key]));
    let [y0,y1] = d3.extent(values);
    const pad = Math.max((y1-y0)*0.12, 0.03);
    const x = d3.scaleLinear().domain(d3.extent(data,d => d.time_s)).range([margin.left,width-margin.right]);
    const y = d3.scaleLinear().domain([y0-pad,y1+pad]).nice().range([height-margin.bottom,margin.top]);
    const plot = svg.append('g');
    plot.append('rect').attr('data-chart-frame','').attr('x',margin.left).attr('y',margin.top)
      .attr('width',width-margin.left-margin.right).attr('height',height-margin.top-margin.bottom)
      .attr('fill','transparent').attr('stroke','var(--border)');
    plot.append('g').attr('transform',`translate(0,${height-margin.bottom})`)
      .call(d3.axisBottom(x).ticks(width<480?3:5).tickFormat(d3.format('.1f')));
    plot.append('g').attr('transform',`translate(${margin.left},0)`)
      .call(d3.axisLeft(y).ticks(4));
    svg.selectAll('.domain,.tick line').attr('stroke','var(--border)');
    svg.selectAll('.tick text').attr('fill','var(--muted-foreground)').attr('font-size',12);
    svg.append('text').attr('class','axis-title').attr('data-axis','x')
      .attr('x',(margin.left+width-margin.right)/2).attr('y',height-6)
      .attr('text-anchor','middle').text('Time (s)');
    svg.append('text').attr('class','axis-title').attr('data-axis','y')
      .attr('transform',`translate(15 ${(margin.top+height-margin.bottom)/2}) rotate(-90)`)
      .attr('text-anchor','middle').text(chart.yLabel);
    for (const key of keys) {
      const line = d3.line().x(d=>x(d.time_s)).y(d=>y(d[key]));
      plot.append('path').datum(data).attr('fill','none').attr('stroke',colors.get(key))
        .attr('stroke-width',2).attr('d',line);
    }
    const cursor = x(t);
    plot.append('line').attr('data-playhead','').attr('x1',cursor).attr('x2',cursor)
      .attr('y1',margin.top).attr('y2',height-margin.bottom).attr('stroke','var(--foreground)')
      .attr('stroke-opacity',0.55).attr('stroke-width',1);
    const sample = nearest(t);
    for (const key of keys) {
      plot.append('circle').attr('cx',x(t)).attr('cy',y(sample[key])).attr('r',3.5)
        .attr('fill',colors.get(key)).attr('data-tooltip',`${definitions.find(d=>d.key===key).name}: ${sample[key].toFixed(key.startsWith('motor_')?3:2)} ${definitions.find(d=>d.key===key).unit}`);
    }
  }
  function drawAll(t) {
    timeInput.value = String(t);
    timeLabel.value = `${t.toFixed(3)} s`;
    const row = nearest(t);
    state.textContent = `t=${t.toFixed(3)} s · roll estimate=${row.estimated_roll_deg.toFixed(2)}° · motors FL/FR/RR/RL=${row.motor_fl.toFixed(3)}/${row.motor_fr.toFixed(3)}/${row.motor_rr.toFixed(3)}/${row.motor_rl.toFixed(3)} · armed=${row.armed} · sensor fault=${row.sensor_fault}`;
    charts.forEach(chart=>drawChart(chart,t));
  }
  let timer = null;
  function stop() { if (timer) clearInterval(timer); timer=null; playButton.setAttribute('aria-pressed','false'); playButton.textContent='Play 5 s replay'; }
  timeInput.addEventListener('input',()=>{stop();drawAll(Number(timeInput.value));});
  playButton.addEventListener('click',()=>{
    if (timer) { stop(); return; }
    if (Number(timeInput.value)>=last) drawAll(0);
    playButton.setAttribute('aria-pressed','true'); playButton.textContent='Pause replay';
    timer=setInterval(()=>{const next=Number(timeInput.value)+0.01;if(next>=last){drawAll(last);stop();}else drawAll(next);},10);
  });
  new ResizeObserver(()=>drawAll(Number(timeInput.value))).observe(root);
  drawAll(0);
})();
</script>'''
html = html.replace("__LAST__", f"{float(data[-1]['time_s']):.3f}").replace("__DATA__", packed)
destination.parent.mkdir(parents=True, exist_ok=True)
destination.write_text(html, encoding="utf-8", newline="\n")
print(f"Wrote {destination} with {len(data)} recorded samples ({destination.stat().st_size} bytes)")
