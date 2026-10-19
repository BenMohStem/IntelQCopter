"""Graphs and video rendered from actual saved CPU/DDS traces, never Isaac footage."""
import json,os
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.animation import FFMpegWriter
import imageio_ffmpeg

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/simulation/2026-10-20'
plt.rcParams.update({'font.size':9,'axes.grid':True,'grid.alpha':.25})

def load(path):return np.genfromtxt(path,delimiter=',',names=True)

def main():
    summary=json.loads((OUT/'summary.json').read_text());name=summary['best_nominal_case'];d=load(OUT/f'{name}.csv')
    m=summary['model'];t=d['time_s'];n=len(d)
    fig,ax=plt.subplots(3,2,figsize=(12,10),constrained_layout=True)
    fig.suptitle(f'IntelQcopter | {name} | actual CPU 6-DOF trace, C controller\nKnown map + simulated odometry; NOT Isaac or YOLO-driven flight')
    a=ax[0,0];a.plot(d['x_m'],d['y_m'],label='flown');path=np.loadtxt(OUT/f'{name}-path.csv',delimiter=',',skiprows=1)
    a.plot(*path.T,'--',alpha=.6,label='A* path')
    for x,y,r in m['obstacles_x_y_radius_m']:
        a.add_patch(Circle((x,y),r,color='gray'));a.add_patch(Circle((x,y),r+m['robot_collision_radius_m'],fill=False,color='red',ls=':'))
    a.scatter(*np.array([m['start_m'][:2],m['goal_m'][:2]]).T,marker='x');a.set(xlabel='x [m]',ylabel='y [m]',aspect='equal');a.legend()
    a=ax[0,1];a.plot(t,d['z_m'],label='true');a.plot(t,d['estimated_height_m'],label='estimated');a.plot(t,d['height_reference_m'],'--',label='reference');a.set(xlabel='time [s]',ylabel='height [m]');a.legend()
    a=ax[1,0]
    for k in ['roll','pitch']:a.plot(t,d[f'{k}_deg'],label=k);a.plot(t,d[f'{k}_command_deg'],'--',alpha=.6,label=f'{k} cmd')
    a.set(xlabel='time [s]',ylabel='angle [deg]');a.legend(ncol=2)
    a=ax[1,1]
    for k in ['fl','fr','rr','rl']:a.plot(t,d[f'motor_{k}'],label=k)
    a.set(xlabel='time [s]',ylabel='normalized desired thrust',ylim=(0,1));a.legend(ncol=4)
    a=ax[2,0];e=d['estimated_height_m']-d['z_m'];s=d['sigma_height_m']
    a.fill_between(t,-2*s,2*s,alpha=.2,label='model +/-2 sigma');a.plot(t,e,label='height error');a.set(xlabel='time [s]',ylabel='error [m]');a.legend()
    a=ax[2,1];a.plot(t,d['clearance_m']);a.axhline(0,color='r');a.set(xlabel='time [s]',ylabel='prop-envelope clearance [m]')
    for ext in ['png','pdf']:fig.savefig(OUT/f'flight-results.{ext}',dpi=160)
    plt.close(fig)
    fig,ax=plt.subplots(2,2,figsize=(11,7),constrained_layout=True)
    for axis,key,label in zip(ax.flat,['K_roll_angle','K_roll_bias','K_height','K_velocity'],['angle gain','bias gain [1/s]','height gain','velocity gain [1/s]']):
        axis.plot(t,d[key]);axis.set(xlabel='time [s]',ylabel=label)
    fig.suptitle('Actual C Kalman gains: time varying; altitude correction only at 50 Hz')
    fig.savefig(OUT/'kalman-gains.png',dpi=160);plt.close(fig)
    result=next(r for r in summary['results'] if r['name']==name)
    metrics={'best':name,'samples':n,'height_error_2sigma_coverage':float(np.mean(abs(e)<=2*s)),
       'roll_error_rmse_deg':float(np.sqrt(np.mean((d['estimated_roll_deg']-d['roll_deg'])**2))),
       'pitch_error_rmse_deg':float(np.sqrt(np.mean((d['estimated_pitch_deg']-d['pitch_deg'])**2))),
       'last_logged_gains':{k:float(d[k][-1]) for k in ['K_roll_angle','K_roll_bias','K_height','K_velocity']},
       'warning':'Coverage is one trajectory under mismatched motion/noise; not a statistical certification.'}
    (OUT/'estimation-metrics.json').write_text(json.dumps(metrics,indent=2),encoding='utf-8')
    # Original visualization of recorded data, real-time playback 1 simulation second/second.
    matplotlib.rcParams['animation.ffmpeg_path']=imageio_ffmpeg.get_ffmpeg_exe()
    fig=plt.figure(figsize=(12.8,7.2),constrained_layout=True)
    gs=fig.add_gridspec(2,2,width_ratios=[1.25,1]);mapax=fig.add_subplot(gs[:,0]);hax=fig.add_subplot(gs[0,1]);rax=fig.add_subplot(gs[1,1])
    fig.suptitle('IntelQcopter | CPU physics + production C flight controller\nKnown map / simulated odometry | NOT Isaac Sim | NOT YOLO navigation',fontsize=13)
    for x,y,r in m['obstacles_x_y_radius_m']:
        mapax.add_patch(Circle((x,y),r,color='#64748b'));mapax.add_patch(Circle((x,y),r+m['robot_collision_radius_m'],fill=False,color='red',ls=':'))
    mapax.plot(*path.T,ls='--',color='gray');mapax.scatter(8,0,marker='*',s=130,color='green',label='goal')
    trail,=mapax.plot([],[],color='#168aad',lw=2);arms=[mapax.plot([],[],color=c,lw=4)[0] for c in ['red','red','white','white']]
    # White rear arms outlined on a dark center disc for visibility.
    body=Circle((0,0),.06,color='black',zorder=6);mapax.add_patch(body)
    propellers=[Circle((0,0),.1143,fill=False,color='#94a3b8',lw=1) for _ in range(4)]
    for p in propellers:mapax.add_patch(p)
    mapax.set(xlim=(-.6,8.7),ylim=(-2.5,2.5),aspect='equal',xlabel='world x [m]',ylabel='world y [m]',facecolor='#e2e8f0');mapax.legend()
    text=mapax.text(.02,.98,'',transform=mapax.transAxes,va='top',family='monospace',bbox={'facecolor':'white','alpha':.85})
    hax.plot(t,d['height_reference_m'],'--',label='reference',color='gray');height,=hax.plot([],[],label='true');estimate,=hax.plot([],[],label='C estimate')
    hax.set(xlim=(0,t[-1]),ylim=(1.40,1.60),xlabel='time [s]',ylabel='altitude [m]');hax.legend()
    lines=[rax.plot([],[],label=k)[0] for k in ['roll','pitch']];rax.set(xlim=(0,t[-1]),ylim=(-12,12),xlabel='time [s]',ylabel='angle [deg]');rax.legend()
    writer=FFMpegWriter(fps=20,codec='libx264',bitrate=2200,extra_args=['-pix_fmt','yuv420p','-movflags','+faststart'])
    arm=m['motor_diagonal_m']/(2*np.sqrt(2));locations=np.array([[arm,arm],[arm,-arm],[-arm,-arm],[-arm,arm]])
    with writer.saving(fig,str(OUT/'best-flight-cpu.mp4'),dpi=100):
        for now in np.arange(0,t[-1]+.025,.05):
            i=min(np.searchsorted(t,now),n-1);trail.set_data(d['x_m'][:i+1],d['y_m'][:i+1]);yaw=np.deg2rad(d['yaw_deg'][i]);r=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
            center=np.array([d['x_m'][i],d['y_m'][i]]);ends=locations@r.T+center;body.center=center
            for line,end,prop in zip(arms,ends,propellers):line.set_data([center[0],end[0]],[center[1],end[1]]);prop.center=end
            height.set_data(t[:i+1],d['z_m'][:i+1]);estimate.set_data(t[:i+1],d['estimated_height_m'][:i+1])
            for line,k in zip(lines,['roll','pitch']):line.set_data(t[:i+1],d[f'{k}_deg'][:i+1])
            text.set_text(f"t={now:5.2f} s\nz={d['z_m'][i]:.3f} m\nclearance={d['clearance_m'][i]:.3f} m\nhardware motors: DISABLED")
            writer.grab_frame()
        fig.savefig(OUT/'video-preview.png',dpi=100)
    plt.close(fig)
    print(json.dumps({'video':str(OUT/'best-flight-cpu.mp4'),'selected':result,'estimation':metrics}),flush=True)

if __name__=='__main__':main()
