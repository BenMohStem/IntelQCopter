"""Check delivered traces, DDS parity, links and video decoding; save original output."""
import hashlib,json,re,subprocess
from pathlib import Path
import numpy as np
import imageio_ffmpeg

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'artifacts/simulation/2026-10-20'

def main():
    summary=json.loads((OUT/'summary.json').read_text());assert all(r['success'] for r in summary['results'])
    name=summary['best_nominal_case'];d=np.genfromtxt(OUT/f'{name}.csv',names=True,delimiter=',')
    for k in d.dtype.names:assert np.isfinite(d[k]).all(),k
    assert np.all(d['hardware_armed']==0) and np.all(d['sensor_fault']==0)
    dds=np.genfromtxt(OUT/'ros2/dds-route.csv',names=True,delimiter=',')
    assert len(d)==len(dds)
    delta=max(float(np.max(abs(d[k]-dds[k]))) for k in ['x_m','y_m','z_m','motor_fl','motor_fr','estimated_height_m'])
    assert delta<1e-4,delta
    movie=OUT/'best-flight-cpu.mp4'
    decoded=subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-hide_banner','-i',str(movie),'-f','null','-'],capture_output=True,text=True)
    (ROOT/'artifacts/test-runs/2026-10-20/video-decode.log').write_text(decoded.stderr,encoding='utf-8')
    assert decoded.returncode==0
    links=[]
    for url in re.findall(r'(?:href|src)="([^"]+)"',(OUT/'index.html').read_text(encoding='utf-8')):
        if not url.startswith(('https:','http:','#')):
            assert (OUT/url).exists(),url;links.append(url)
    pdf=ROOT/'docs/modelisation/main.pdf';assert pdf.read_bytes().startswith(b'%PDF-')
    report={'all_six_route_cases_pass':True,'finite_trace':True,'hardware_armed_entire_trace':False,
      'maximum_selected_vs_dds_trace_difference':delta,'video_full_decode':'passed',
      'local_viewer_links_checked':len(links),'pdf_size_bytes':pdf.stat().st_size,
      'note':'These checks validate saved evidence, not physical flight or Isaac execution.'}
    (OUT/'delivery-validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    files=[p for p in OUT.rglob('*') if p.is_file() and p.name!='sha256-manifest.json']
    files.extend(p for p in (ROOT/'docs/modelisation').glob('*') if p.suffix in ['.tex','.pdf'])
    manifest={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
    (OUT/'sha256-manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print(json.dumps(report),flush=True)

if __name__=='__main__':main()
