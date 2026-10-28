"""Illustrative continuous PID loop analysis; no flight gains or hardware validation."""
from pathlib import Path
import json
import hashlib
from functools import lru_cache
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PARAMETERS = {'J_kg_m2': .006421, 'd_Nm_s_rad': .01, 'motor_tau_s': .035,
              'delay_s': .015, 'derivative_corner_rad_s': float(2*np.pi*80),
              'Kp_Nm_s_rad': .12, 'Ki_Nm_rad': .2, 'Kd_Nm_s2_rad': .0008}


@lru_cache(maxsize=64)
def loop_polynomials(multiplier=1.0, motor_tau=None):
    p = PARAMETERS
    s = np.poly1d([1., 0.])
    wf = p['derivative_corner_rad_s']
    # Filtered parallel PID: Kp + Ki/s + Kd*wf*s/(s+wf).
    cn = multiplier*(p['Kp_Nm_s_rad']*s*(s+wf) + p['Ki_Nm_rad']*(s+wf) + p['Kd_Nm_s2_rad']*wf*s*s)
    cd = s*(s+wf)
    # Damped rate plant, actuator lag and first-order Pade delay approximation.
    gn = np.poly1d([-p['delay_s']/2, 1.])
    gd = (p['J_kg_m2']*s+p['d_Nm_s_rad'])*((p['motor_tau_s'] if motor_tau is None else motor_tau)*s+1)*(p['delay_s']/2*s+1)
    return cn*gn, cd*gd


def response(s, multiplier=1.0):
    num, den = loop_polynomials(multiplier)
    return np.polyval(num, s)/np.polyval(den, s)


def bisect(fun, left, right):
    fl = fun(left)
    for _ in range(70):
        mid = (left+right)/2
        fm = fun(mid)
        if fl*fm <= 0:
            right = mid
        else:
            left, fl = mid, fm
    return (left+right)/2


def crossings(fun, grid):
    values = np.asarray(fun(grid))
    return [bisect(fun, grid[i], grid[i+1]) for i in np.where(values[:-1]*values[1:] < 0)[0]]


def analyze(multiplier, grid_count=4000, epsilon=1e-4, outer=1e6):
    num, den = loop_polynomials(multiplier)
    roots = np.roots(den+num)
    open_roots = np.roots(den)
    P = int(np.sum(open_roots.real > 1e-7))
    Z = int(np.sum(roots.real > 1e-7))
    # Clockwise RHP contour, indented to the right of the pole at s=0.
    w = np.geomspace(epsilon, outer, 12000)
    contour = np.concatenate((-1j*w[::-1], epsilon*np.exp(1j*np.linspace(-np.pi/2,np.pi/2,3000)),
                              1j*w, outer*np.exp(1j*np.linspace(np.pi/2,-np.pi/2,3000))))
    values = 1+response(contour, multiplier)
    values = np.append(values, values[0])
    winding_ccw = float(np.angle(values[1:]/values[:-1]).sum()/(2*np.pi))
    N_clockwise = int(round(-winding_ccw))
    assert abs(winding_ccw-round(winding_ccw)) < 1e-6
    assert N_clockwise == Z-P, 'Nyquist contour and polynomial root count disagree'
    grid = np.geomspace(1e-4, 1e6, grid_count)
    gc = crossings(lambda w: abs(response(1j*w,multiplier))-1, grid)
    pc = [w for w in crossings(lambda w: response(1j*w,multiplier).imag, grid)
          if response(1j*w,multiplier).real < 0]
    phase_margins=[]
    for w in gc:
        phase = float(np.angle(response(1j*w,multiplier),deg=True))
        if phase>0: phase-=360
        phase_margins.append(180+phase)
    gain_margins=[float(-20*np.log10(abs(response(1j*w,multiplier)))) for w in pc]
    residuals = [abs(np.polyval(den+num,z))/max(1.,np.sum(np.abs((den+num).c)*np.abs(z)**np.arange(len((den+num).c)-1,-1,-1))) for z in roots]
    return {'gain_multiplier':multiplier,'open_loop_num':num.c.tolist(),'open_loop_den':den.c.tolist(),
            'closed_loop_poles':[{'real':float(z.real),'imag':float(z.imag)} for z in roots],
            'open_RHP_poles_P':P,'closed_RHP_poles_Z':Z,'clockwise_encirclements_N':N_clockwise,
            'gain_crossovers_rad_s':gc,'phase_margins_deg':phase_margins,
            'phase_crossovers_rad_s':pc,'gain_margins_db':gain_margins,
            'maximum_normalized_root_residual':float(max(residuals)),
            'contour':{'epsilon':epsilon,'outer_radius':outer,'origin_indented_right':True}}


def main():
    cases=[analyze(x) for x in [1.,8.]]
    for case in cases:
        alternate=analyze(case['gain_multiplier'],8000,1e-3,1e5)
        assert alternate['clockwise_encirclements_N']==case['clockwise_encirclements_N']
        assert np.allclose(alternate['phase_margins_deg'],case['phase_margins_deg'],atol=1e-7)
        assert case['maximum_normalized_root_residual']<1e-12
    assert cases[0]['closed_RHP_poles_Z']==0 and cases[1]['closed_RHP_poles_Z']==2
    result={'model_id':'PID-RATE-ILLUSTRATIVE-001','source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'status':'ILLUSTRATIVE_ANALYSIS_ONLY','engine':'NumPy polynomial frequency evaluation/root solving; no Gazebo or MCU run',
            'parameters':PARAMETERS,'assumptions':'J from supplied simulation input; damping, actuator lag, delay and gains are chosen educational assumptions. First-order Pade is not an exact delay model.',
            'negative_feedback':True,'cases':cases,
            'checks':['Root count versus indented Nyquist winding','Alternative contour radii and frequency-grid refinement','Normalized characteristic-polynomial residuals'],
            'limitations':['Not measured flight gains','Not sampled 1kHz control acceptance','One rate loop; other five loops/coupling not certified','Pade approximation and assumed damping restrict inference']}
    path=ROOT/'analysis/control-design-example.json'
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8',newline='\n')
    for case in cases:print(case['gain_multiplier'],'P/Z/N=',case['open_RHP_poles_P'],case['closed_RHP_poles_Z'],case['clockwise_encirclements_N'],'PM=',case['phase_margins_deg'],'GMdB=',case['gain_margins_db'])


if __name__=='__main__':
    main()
