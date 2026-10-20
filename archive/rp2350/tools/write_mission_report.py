"""Build figures and a detailed LaTeX results chapter from original mission data."""
import json,shutil,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'artifacts/simulation/2026-10-20-missions'
sys.path.insert(0,str(ROOT/'sim/sitl'));from run import gain_table

def escape(s):return s.replace('_',r'\_')
def polynomial(a):
    terms=[];degree=len(a)-1
    for i,c in enumerate(a):
        if abs(c)<1e-10:continue
        power=degree-i;value=f'{abs(c):.7g}'
        term=value+('s' if power==1 else f's^{{{power}}}' if power>1 else '')
        terms.append(('+' if c>=0 and terms else '-' if c<0 else '')+term)
    return ''.join(terms)

def main():
    result=json.loads((OUT/'garden-final-summary.json').read_text());linear=json.loads((OUT/'linear-analysis.json').read_text())
    d=np.genfromtxt(OUT/'garden-final.csv',names=True,delimiter=',');t=d['time_s'];goal=np.array(result['model']['goal_m'])
    cruise=(d['phase']>=3)&(d['phase']<=5)
    metrics={'height_rmse_in_cruise_m':float(np.sqrt(np.mean((d['z_m'][cruise]-result['cruise_height_m'])**2))),
      'height_estimation_rmse_m':float(np.sqrt(np.mean((d['estimated_height']-d['z_m'])**2))),
      'minimum_logged_motor_headroom':float(1-max(d[k].max() for k in ['motor_fl','motor_fr','motor_rr','motor_rl'])),
      'maximum_roll_deg':float(max(abs(d['roll_deg']))),'maximum_pitch_deg':float(max(abs(d['pitch_deg']))),
      'maximum_sigma_height_m':float(max(d['sigma_height']))}
    metrics['minimum_applied_motor_headroom']=float(1-max(d[k].max() for k in ['actual_motor_fl','actual_motor_fr','actual_motor_rr','actual_motor_rl']))
    (OUT/'garden-final-metrics.json').write_text(json.dumps(metrics,indent=2),encoding='utf-8')
    plt.rcParams.update({'font.size':9,'axes.grid':True,'grid.alpha':.25})
    fig=plt.figure(figsize=(12,11),constrained_layout=True);grid=fig.add_gridspec(3,2)
    ax=fig.add_subplot(grid[0,0],projection='3d');ax.plot(d['x_m'],d['y_m'],d['z_m'],lw=2)
    # Evaluation-only true geometry, never read by the online mapper.
    for x,y,yaw in [(3.4,.15,-.08),(6.,.65,.12)]:
        corners=np.array([[0,-.6,0],[0,.6,0],[0,.6,1.8],[0,-.6,1.8]])
        r=np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
        poly=corners@r.T+np.array([x,y,0]);ax.add_collection3d(Poly3DCollection([poly],alpha=.45,facecolor='royalblue'))
    ax.set(xlabel='x [m]',ylabel='y [m]',zlabel='height [m]',xlim=(-.5,8.5),ylim=(-2.5,2.5),zlim=(0,2),title='Full ground-to-ground trajectory')
    ax=fig.add_subplot(grid[0,1]);ax.plot(d['x_m'],d['y_m'],label='flown');ax.scatter(goal[0],goal[1],marker='*',label='finish')
    observations=json.loads((OUT/'garden-final-observations.json').read_text())
    points=np.array([o['estimated_world'] for o in observations]);ax.scatter(points[:,0],points[:,1],s=10,alpha=.5,label='RGB-derived panel positions')
    ax.set(xlabel='x [m]',ylabel='y [m]',aspect='equal',title='No prior panel map supplied');ax.legend()
    ax=fig.add_subplot(grid[1,0]);ax.plot(t,d['z_m'],label='true');ax.plot(t,d['estimated_height'],alpha=.75,label='C estimate');ax.plot(t,d['height_reference'],'--',label='integrated target')
    ax.set(xlabel='time [s]',ylabel='height [m]');ax.legend()
    ax=fig.add_subplot(grid[1,1]);ax.plot(t,d['vz_mps'],label='true vertical speed');ax.plot(t,4*(d['throttle']-.5),label='vertical-speed request')
    ax.set(xlabel='time [s]',ylabel='speed [m/s]');ax.legend()
    ax=fig.add_subplot(grid[2,0])
    for key in ['motor_fl','motor_fr','motor_rr','motor_rl']:ax.plot(t,d['actual_'+key],label=key[-2:])
    ax.step(t,.95*d['simulation_actuator_gate'],'--',alpha=.5,label='virtual gate')
    ax.set(xlabel='time [s]',ylabel='applied normalized thrust proxy',ylim=(0,1));ax.legend(ncol=5)
    ax=fig.add_subplot(grid[2,1]);e=d['estimated_height']-d['z_m'];sigma=d['sigma_height']
    ax.fill_between(t,-2*sigma,2*sigma,alpha=.2,label='model +/-2 sigma');ax.plot(t,e,label='height estimation error')
    ax.set(xlabel='time [s]',ylabel='error [m]');ax.legend()
    fig.suptitle('Actual 3D PyBullet / C flight / C++ mission / RGB YOLO-PnP / downward lidar / ROS2 DDS\nKnown-size panels; simulated odometry and ground-contact signal; NOT Isaac or hardware flight')
    fig.savefig(OUT/'garden-final-graphs.png',dpi=160);fig.savefig(OUT/'garden-final-graphs.pdf');plt.close(fig)
    images=ROOT/'docs/modelisation/images'
    for name in ['garden-final-graphs.png','linear-step-responses.png']:shutil.copy2(OUT/name,images/name)
    text=r'''\chapter{New ground-to-ground garden experiments and numerical analysis}
\section{What changed from the original report}
The general matrix-multiplication and matrix-exponential lessons have been
removed. Their instructional style is applied to the actual free-body equations,
linearization, transfer functions, full feedback polynomials and stability proofs.
The earlier known-map airborne test is retained as historical evidence. The new
experiment starts on the ground, takes off, navigates, approaches, descends and
lands with actual contact dynamics. Flight control still has exactly six PID
objects and the same angle/rate/vertical-speed interface.

The current boot gain candidate is $\lambda=0.7$, replacing the earlier
$\lambda=1$ default. This choice follows both the wider local stability checks
and a completed mission. It is a provisional simulation/bench candidate; it is
not a hardware flight tune. Host tests explicitly load their own gains when needed.
\section{Current six-controller candidate and model coefficients}
\begin{center}\begin{tabular}{lrrr}\toprule Controller & $K_p$ & $K_i$ & $K_d$\\\midrule
'''
    gains=gain_table(result['model'],.7)
    for name,g in zip(['Roll angle','Roll rate','Pitch angle','Pitch rate','Yaw rate','Altitude'],gains):
        text+=f'{name} & {g[0]:.9g} & {g[1]:.9g} & {g[2]:.9g}'+r'\\'+'\n'
    text+=r'''\bottomrule\end{tabular}\end{center}
Angular units are degrees and degrees/s, as in the C code. Normalized output
is a desired-thrust proxy. The assumed gains from allocation and mechanics are
'''
    p=linear['parameters']
    text+=f"$b_x=b_y={p['b_roll_deg']:.8f}$ deg/s$^2$/command, $b_\\psi={p['b_yaw_deg']:.8f}$ deg/s$^2$/command, and $b_h=20$ m/s$^2$/command. The actuator lag is 0.04s and translational drag rate is 0.1s$^{{-1}}$.\n"
    text+=r'''For the conservative candidate, the following polynomials are
formed from the equations in the derivation chapters. They retain actuator lag;
height additionally retains drag. Write $T=N/D$ with descending powers:
'''
    for key,label in [('angle','\phi'),('yaw','r'),('height','h')]:
        m=linear['models'][f'{key}-0.7']
        text+=r'\begin{align}'+'\n'+f'N_{{{label}}}(s)&={polynomial(m["numerator"])},'+r'\\'+'\n'+f'D_{{{label}}}(s)&={polynomial(m["denominator"])}.'+'\n'+r'\end{align}'+'\n'
    text+=r'''\section{Poles, step response, Routh values and sampling}
The table reports actual local transfer-function responses, including numerator
zeros. Settling means the last exit from a two-percent band over the simulated
20s linear response; it is not a nonlinear aircraft landing time.
\begin{center}\small\begin{tabular}{llrrr}\toprule Scale & Loop & Overshoot [\%] & Settling [s] & Sampled radius\\\midrule
'''
    for scale in [.7,1,1.3]:
        for key in ['angle','roll_rate','yaw','height']:
            m=linear['models'][f'{key}-{float(scale)}'];st=m['step'];settle=st['settling_2percent_s']
            text+=f'{scale} & {escape(key)} & {st["overshoot_percent"]:.3f} & {settle:.3f} & {m["sampled_spectral_radius"]:.7f}'+r'\\'+'\n'
    text+=r'''\bottomrule\end{tabular}\end{center}
The local continuous poles for the conservative loops are listed explicitly:
\begin{description}
'''
    for key in ['angle','roll_rate','yaw','height']:
        values=linear['models'][f'{key}-0.7']['poles']
        text+=r'\item['+escape(key)+'] $'+',\ '.join(f'{a:.5f}{b:+.5f}j' for a,b in values)+r'$ s$^{-1}$.'+'\n'
    text+=r'''\end{description}
The saved JSON contains full numerator/denominator arrays, every Routh first
column, phase margins for the rate loops, and sampled eigenvalues with and
without one flight-sample delay. The sixth-order horizontal models have positive
leading Hurwitz determinants for the tested candidates. Their multirate radii are:
\begin{center}\begin{tabular}{llrr}\toprule Scale & Guidance & 500Hz/20Hz radius & With 50ms delay\\\midrule
'''
    for scale in [.7,1,1.3]:
        for key in ['navigate','approach']:
            m=linear['horizontal_guidance'][f'{key}-{float(scale)}']
            text+=f'{scale} & {key} & {m["500Hz_FC_20Hz_guidance_spectral_radius"]:.7f} & {m["one_guidance_period_delay_spectral_radius"]:.7f}'+r'\\'+'\n'
    text+=r'''\bottomrule\end{tabular}\end{center}
These tests omit estimator states and map latency. They must not be confused
with proving real processor deadlines or global nonlinear stability.
\begin{figure}[htbp]\centering\includegraphics[width=\textwidth]{images/linear-step-responses.png}
\caption{Local transfer-function step responses. Overshoot differs from the nominal
second-order design because the complete models contain extra poles and zeros.}\end{figure}

\section{The uncertainty grid exposes a real tuning limitation}
The finite grid uses fifteen plant-gain factors from 0.65 to 1.35 and fifteen
actuator lags from 0.02 to 0.09s, for angle, yaw-rate and height models:
\begin{center}\begin{tabular}{lrrr}\toprule Scale & Stable tests & Total & Largest real pole\\\midrule
'''
    for scale,g in linear['finite_grid'].items():
        text+=f'{scale} & {g["stable"]} & {g["total"]} & {g["largest_real_pole"]:.6f}'+r'\\'+'\n'
    text+=r'''\bottomrule\end{tabular}\end{center}
Thus the faster candidate is not robust over all these assumed points. The
separate analytic interval argument in the stability chapter proves the local
decoupled conservative-candidate inequalities throughout the stated continuous
box, including a height-drag interval. The complete camera-guided nonlinear
aircraft is not covered by that theorem. Actual motor lag remains unmeasured.

\section{Camera-only panel distance, derived from the measurement geometry}
The hardware direction is one RGB camera plus one downward 1D lidar. No depth
image is supplied to this planner. For this controlled course only, fixtures
are rectangular blue boards 1.2m wide and 1.8m high with white borders. Their
visible blue inner rectangle is 1.09m by 1.69m. These are chosen test-fixture
dimensions, not an assertion of a universal standard panel size.

YOLO identifies candidate image regions. Colour/contour fitting supplies four
ordered inner corners; clipped or nonconvex quadrilaterals are rejected. Given
calibrated intrinsics $K$, define planar object points
\begin{equation}
P_1=(-w/2,-H/2,0),\ P_2=(w/2,-H/2,0),\
P_3=(w/2,H/2,0),\ P_4=(-w/2,H/2,0).
\end{equation}
Each corner satisfies
\begin{equation}
\lambda_i\begin{bmatrix}u_i\\v_i\\1\end{bmatrix}=K(R_{CO}P_i+t_{CO}).
\end{equation}
The planar pose solver estimates $R_{CO},t_{CO}$. Positive depth and a
reprojection RMS below2pixels are required. A low reprojection error alone
does not prove correct metric scale or remove planar pose ambiguity.

In the frontal special case, apparent height $h_p$ pixels gives
$h_p=f_yH/Z$, so $Z=f_yH/h_p$. Differentiate this physical equation:
\begin{equation}
\frac{\partial Z}{\partial f_y}=\frac Z{f_y},\quad
\frac{\partial Z}{\partial H}=\frac ZH,\quad
\frac{\partial Z}{\partial h_p}=-\frac Z{h_p}.
\end{equation}
For independent small calibration/size/pixel errors,
\begin{equation}
\frac{\sigma_Z^2}{Z^2}\simeq
\frac{\sigma_f^2}{f_y^2}+\frac{\sigma_H^2}{H^2}+\frac{\sigma_p^2}{h_p^2}.
\end{equation}
This proves why distant small images have larger relative range uncertainty.
It also proves the unresolved scale problem: replacing both $H$ and $Z$ by
1.2 times their values leaves $h_p$ unchanged. A regression test explicitly
confirms that a20\% wrong fixture size produces a20\% wrong range scale.
The four rasterized geometry tests at2.5--5m have errors saved in the report;
they are synthetic calibration tests, not real camera accuracy measurements.

For the general four-corner fit, local pose covariance can be approximated by
$(J^{\mathsf T}\Sigma_{\rm pix}^{-1}J)^{-1}$ when the reprojection Jacobian has
sufficient rank and one pose solution dominates. A multimodal/ambiguous solution
must not be summarized as a single confident Gaussian. The implementation uses
a provisional conservative range scale $\sigma_r=0.04Z+0.03$m and obstacle
inflation; this heuristic is not a calibrated posterior covariance.

Optical right/down/forward transforms to body forward/left/up as
\begin{equation}(X_B,Y_B,Z_B)=(Z_C,-X_C,-Y_C).\end{equation}
Add the 0.18 m forward / 0.02 m upward camera lever arm, then use
$p_W=R_{WB}(p_B+t_{BC})+p_{WB}$.
World-map uncertainty also includes pose and extrinsic calibration errors;
simulated odometry supplies millimetre-scale translation noise and the true
orientation in this prototype. A real VIO implementation is still missing.

\section{Training and operating-domain limitations}
The actual YOLO11n derivative was trained for12epochs at320pixels on180 synthetic
training images and40 synthetic validation images. Validation poses are held out;
the simulation tests also change panel layouts. This is not a real-world
generalization test. Raw training output and its CSV are retained, including an
initial plotting-dependency error; the missing dependency was repaired later.
Real images, hard negatives, non-blue panels, lighting changes and unknown-size
objects remain unvalidated. No true obstacle coordinates, camera depth or
segmentation labels enter the online mapper. Renderer IDs are used only to
generate training labels, and true geometry is used only for offline scoring.

The downward ray lidar is a simulated range sensor, not a validated TOFSense
driver. On flat ground it supplies $h=(d+0.085)\cos\phi\cos\theta$ with a known
lever arm; invalid/near-ground returns are rejected. The C two-state height
filter corrects fresh lidar observations with variance0.0004m$^2$ in addition
to the barometer. Real surface reflectivity, slope, sunlight and sensor latency
have not been identified. Virtual ground contact is used by the mission's
touchdown confirmation; a reliable real landed detector remains unfinished.

\section{Recorded mission progression and outcomes}
The first attempt reached the ground without collision but missed the horizontal
finish by about0.48m and failed the0.4m criterion. Position feedback during
approach/landing fixed that problem. Later refinements require a stable approach
before descent, inhibit RT boost during approach/landing, and correct the visible
inner-panel dimensions. Earlier iteration files are preserved; not every early
iteration has a source hash. The delivery run records source hashes at startup.
\begin{center}\scriptsize\begin{tabular}{lrrrrl}\toprule Run & Time[s] & Final XY[m] & Clearance[m] & PnP obs & Result\\\midrule
'''
    names=['initial-landing-drift','garden-nominal','garden-heldout','garden-camera-dropout','garden-boost-slew','garden-conservative','garden-selected-ros2','garden-final']
    for name in names:
        r=json.loads((OUT/f'{name}-summary.json').read_text())
        text+=f'{escape(name)} & {r["duration_s"]:.2f} & {r["final_xy_error_m"]:.5f} & {r["minimum_panel_clearance_m"]:.4f} & {r["valid_range_observations"]} & {"pass" if r["success"] else "fail"}'+r'\\'+'\n'
    text+=r'''\bottomrule\end{tabular}\end{center}
Different layouts, seeds and software iterations are not a controlled speed
ranking. The delivered video selects the current conservative-gain, calibrated,
shared-C++ policy run with actual ROS2 selection/transport, rather than claiming
a global optimum from those different courses. Boost can worsen mission settling;
it is an allowance, not evidence of better-than-FPV performance.
'''
    text+=f"The delivery run takes {result['duration_s']:.2f}s, ends {result['final_xy_error_m']:.6f}m from the requested XY finish, reaches {result['maximum_height_m']:.6f}m and has minimum evaluated panel clearance {result['minimum_panel_clearance_m']:.6f}m including the conservative rotor-envelope collision shape. Height RMS error during hover/navigation/approach is {metrics['height_rmse_in_cruise_m']:.6f}m, and height-estimation RMS error across the complete run is {metrics['height_estimation_rmse_m']:.6f}m. These are conditional simulation values.\n"
    text+=r'''\begin{figure}[p]\centering\includegraphics[width=\textwidth]{images/garden-final-graphs.png}
\caption{Original complete-flight data, including takeoff, landing and controller uncertainty.
True geometry shown in evaluation figures is not supplied to the online planner.}\end{figure}
\clearpage
\section{ROS2 selection, timing and implementation boundaries}
'''
    transport=json.loads((OUT/'garden-ros2-transport.json').read_text())
    text+=f"The final adapter run completes {transport['requests']:,} sequenced requests through two ROS2 DDS nodes. Median measured request latency is {transport['median_ms']:.4f}ms, p99 is {transport['p99_ms']:.4f}ms and maximum is {transport['max_ms']:.4f}ms.\n"
    text+=r'''The test includes C resets,500Hz control steps and lidar corrections.
A Python3.10 renderer reaches bundled Python3.12 ROS bindings through local
TCP IPC; DDS then carries the test request to the C subscriber and returns its
result. This is actual DDS execution, not proof that every Windows update meets
2ms. The complete ROS C++ wrappers are authored but still unbuilt on this PC.

The working bundled selector accepts seven values: start XYZ, finish XYZ and
cruise height in local metres. ROS acknowledges the chosen values, and the
mission rejects a start that differs from the actual launch pose. The final
demonstration uses a separate initial-condition spawn, not an in-flight teleport.
The C++ package exposes RequestMission and a CLI client, plus a land-at-current-XY
request service. Camera PnP publishes metric observations into the online map.
No service, gesture or simulator gate enables physical motor arming.

\section{Current missing information and unresolved problems}
The separate project-status document orders the hardware questions. The main
remaining requirements are: measured takeoff mass/centre of mass/inertia;
motor/9047 thrust/current/lag data; exact ESC protocol; camera model and calibrated
intrinsics/extrinsics/timing; an implemented real local-pose/VIO source; exact
LoRa module/band and airtime budget; sensor drivers; routed PCB/power/current
measurement; real landed detection and tested sensor-loss/battery authority.
The garden is an original illustrative campus scene based on visual references,
not a surveyed USTHB map. Single RGB plus one lidar ray does not establish
general metric obstacle coverage without additional geometric assumptions or
validated motion-based reconstruction. Physical motors remain disabled.
'''
    for old,new in {'below2pixels':'below 2 pixels','a20\\%':'a 20\\%','at2.5--5m':'at 2.5--5 m',
        'for12epochs at320pixels on180':'for 12 epochs at 320 pixels on 180','and40':'and 40',
        'variance0.0004m':'variance 0.0004 m','about0.48m':'about 0.48 m','the0.4m':'the 0.4 m',
        '500Hz':'500 Hz','20Hz':'20 Hz','50ms':'50 ms','2ms':'2 ms','20s linear':'20 s linear',
        '0.04s':'0.04 s','0.1s':'0.1 s','ROS2':'ROS 2','ms,':' ms,','ms.':' ms.'}.items():text=text.replace(old,new)
    (ROOT/'docs/modelisation/mission_results.tex').write_text(text,encoding='utf-8',newline='\n')
    print(json.dumps({'figures':'written','chapter':'mission_results.tex','metrics':metrics}),flush=True)

if __name__=='__main__':main()
