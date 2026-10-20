"""Graphs and integrity checks derived only from recorded native engine runs."""
from pathlib import Path
import hashlib,json,subprocess,zipfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import imageio_ffmpeg
import cv2

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'artifacts/simulation/2026-10-20-engines'

def report(name,label):
    source=OUT/f'{name}-summary.json';summary=json.loads(source.read_text())
    data=np.genfromtxt(OUT/f'{name}.csv',delimiter=',',names=True);t=data['time_s']
    assert np.isfinite(np.column_stack([data[n] for n in data.dtype.names])).all()
    cruise=(data['phase']>=3)&(data['phase']<=4)
    metrics={'cruise_height_rmse_m':float(np.sqrt(np.mean((data['z_m'][cruise]-1.25)**2))),
        'height_estimation_rmse_m':float(np.sqrt(np.mean((data['estimated_height']-data['z_m'])**2))),
        'minimum_applied_motor_headroom':float(1-max(data[n].max() for n in ['actual_motor_fl','actual_motor_fr','actual_motor_rr','actual_motor_rl'])),
        'maximum_abs_roll_deg':float(np.abs(data['roll_deg']).max()),'maximum_abs_pitch_deg':float(np.abs(data['pitch_deg']).max()),
        'rows':len(data),'phases_recorded':sorted(np.unique(data['phase']).astype(int).tolist())}
    checks={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in summary['source_sha256_at_start'].items()}
    ffmpeg=imageio_ffmpeg.get_ffmpeg_exe()
    video=subprocess.run([ffmpeg,'-v','error','-i',str(OUT/f'{name}.mp4'),'-f','null','-'],capture_output=True,text=True)
    validation={'success':summary['success'],'source_hash_matches':checks,'physical_arming_all_zero':bool(np.all(data['hardware_armed']==0)),
        'phase_sequence_complete':set(range(1,8)).issubset(set(metrics['phases_recorded'])),
        'native_ground_contact':summary['final_ground_contact'],'virtual_motors_stopped':summary['final_actual_motor_max']<.001,
        'video_decode_exit':video.returncode,'video_decode_errors':video.stderr}
    (OUT/f'{name}-metrics.json').write_text(json.dumps(metrics,indent=2));(OUT/f'{name}-validation.json').write_text(json.dumps(validation,indent=2))
    fig,axes=plt.subplots(3,2,figsize=(13,10),constrained_layout=True);fig.suptitle(label+' | actual recorded experiment; assumed airframe')
    ax=axes[0,0];ax.plot(data['x_m'],data['y_m']);ax.scatter([0,8],[0,0],marker='x');ax.set(xlabel='x [m]',ylabel='y [m]',title='Native physics trajectory');ax.axis('equal')
    ax=axes[0,1];ax.plot(t,data['z_m'],label='true');ax.plot(t,data['estimated_height'],label='estimated');ax.plot(t,data['height_reference'],label='reference',alpha=.6);ax.set(title='Height',xlabel='simulated time [s]',ylabel='m');ax.legend()
    ax=axes[1,0]
    for n in ['roll_deg','pitch_deg']:ax.plot(t,data[n],label=n)
    ax.set(title='Attitude',xlabel='simulated time [s]',ylabel='degrees');ax.legend()
    ax=axes[1,1]
    for n in ['actual_motor_fl','actual_motor_fr','actual_motor_rr','actual_motor_rl']:ax.plot(t,data[n],label=n[-2:])
    ax.set(title='Applied motor states after lag and gate',xlabel='simulated time [s]',ylabel='normalized');ax.legend()
    ax=axes[2,0];ax.plot(t,data['estimated_height']-data['z_m'],label='height error');ax.plot(t,2*data['sigma_height'],'--',label='+2 predicted sigma');ax.plot(t,-2*data['sigma_height'],'--');ax.set(title='Estimator uncertainty (synthetic noise)',xlabel='simulated time [s]',ylabel='m');ax.legend()
    ax=axes[2,1];ax.step(t,data['phase'],label='mission phase');ax.step(t,data['observed_count'],label='observed panels');ax.plot(t,data['contact'],label='native contact',alpha=.6);ax.set(title='Mission and observation state',xlabel='simulated time [s]');ax.legend()
    for ax in axes.flat:ax.grid(alpha=.2)
    fig.savefig(OUT/f'{name}-graphs.png',dpi=160);fig.savefig(OUT/f'{name}-graphs.pdf');plt.close(fig)
    cap=cv2.VideoCapture(str(OUT/f'{name}.mp4'));cap.set(cv2.CAP_PROP_POS_MSEC,12000);ok,frame=cap.read();cap.release()
    if ok:cv2.imwrite(str(OUT/f'{name}-preview.png'),frame)
    with zipfile.ZipFile(OUT/f'{name}-source-snapshot.zip','w',zipfile.ZIP_DEFLATED) as archive:
        for path in summary['source_sha256_at_start']:archive.write(ROOT/path,path)
    print(name,summary['success'],metrics,validation,flush=True)
    return summary,metrics,validation

if __name__=='__main__':
    import argparse
    ap=argparse.ArgumentParser();ap.add_argument('name');ap.add_argument('--label',required=True);args=ap.parse_args();report(args.name,args.label)
