from pathlib import Path
import json

root = Path(r'C:\Users\Mohamed\Desktop\Work\IntelQCopter-USTHB')
p = root/'tools/advanced_figures.py'
s = p.read_text(encoding='utf-8')
start = s.index("    if 'nyquist-example' in selected:")
end = s.index("    if 'poles-3d' in selected:", start)
s = s[:start] + '''    if 'nyquist-example' in selected:
        fig,(ax,bx)=plt.subplots(1,2,figsize=(8.6,3.15),layout='constrained')
        w=np.geomspace(1e-4,1e6,18000)
        for mu,c in [(1,GREEN),(8,RED)]:
            z=response(1j*w,mu)
            # Include every finite sampled frequency; log compression prevents clipping.
            for target in (ax,bx):
                target.plot(z.real,z.imag,c=c,lw=1.6,label=f'μ={mu:g}: +ω')
                target.plot(z.real,-z.imag,c=c,lw=1.1,ls='--',alpha=.8)
            for f in [.01,.1,1,5,15,45,100,300]:
                a=response(1j*f,mu);b=response(1j*(f*1.12),mu)
                arrow(ax,(a.real,a.imag),(b.real,b.imag),c)
                if -3<a.real<1 and -2.5<a.imag<2.5:
                    arrow(bx,(a.real,a.imag),(b.real,b.imag),c)
        ax.set_xscale('symlog',linthresh=1);ax.set_yscale('symlog',linthresh=1)
        ax.margins(.08);ax.set(title='A. Entire sampled frequency branches',xlabel='Re L (symmetric-log scale)',ylabel='Im L (symmetric-log scale)')
        bx.set(xlim=(-3,1),ylim=(-2.5,2.5),aspect='equal',title='B. Critical-point detail: linear axes',xlabel='Re L',ylabel='Im L')
        for target in (ax,bx):
            target.scatter([-1],[0],c=DARK,marker='x',s=48,zorder=6)
            target.axhline(0,c=GRAY,lw=.6);target.axvline(0,c=GRAY,lw=.6);target.grid(alpha=.3)
        ax.legend(fontsize=8,loc='upper right');bx.annotate('−1 + j0',(-1,0),xytext=(-2.75,.65),fontsize=8,arrowprops={'arrowstyle':'->','color':DARK})
        save(fig,'nyquist-example','A shows all sampled branches, 10^-4 ≤ |ω| ≤ 10^6 rad/s: solid +ω, dashed conjugate −ω; arrows follow increasing positive frequency. Symmetric-log axes compress distances, not topology. B magnifies −1 on equal linear axes. The integrator makes the branches unbounded as ω approaches zero; full contour closure is shown in appendix A6.')
''' + s[end:]
start=s.index("        w=np.geomspace(.02,1e5,5000)",s.index("    if 'nyquist-contour' in selected:"))
end=s.index("    if 'stability-surface' in selected:",start)
s=s[:start]+'''        eps=1e-4;outer=1e6;w=np.geomspace(eps,outer,12000)
        pieces=[(-1j*w[::-1],BLUE,'Negative-frequency branch'),
                (eps*np.exp(1j*np.linspace(-np.pi/2,np.pi/2,3000)),GOLD,'Origin indentation'),
                (1j*w,GREEN,'Positive-frequency branch'),
                (outer*np.exp(1j*np.linspace(np.pi/2,-np.pi/2,3000)),RED,'Outer closure')]
        for points,c,label in pieces:
            z=response(points,8);bx.plot(z.real,z.imag,c=c,lw=1.4,label=label)
            i=len(z)//2;arrow(bx,(z[i].real,z[i].imag),(z[i+20].real,z[i+20].imag),c)
        bx.scatter([-1],[0],c=DARK,marker='x',s=40,zorder=6)
        bx.set_xscale('symlog',linthresh=1);bx.set_yscale('symlog',linthresh=1);bx.margins(.10)
        bx.set(xlabel='Re L (symmetric-log)',ylabel='Im L (symmetric-log)',title='Entire mapped contour: μ=8, Ncw=2');bx.grid(alpha=.3);bx.legend(fontsize=6.4,loc='upper left')
        save(fig,'nyquist-contour','Left: s-plane contour schematic. Right: all four mapped pieces, including the very large origin-indentation arc, with no clipped data. Symmetric-log axes preserve encirclement but not Euclidean distances. ε=10^-4; outer radius10^6; closure near L=0 is tiny. Winding is computed from untransformed complex values.')
    if 'anti-windup-example' in selected:
        # Scripted error isolates the integrator; this is deliberately not a plant simulation.
        dt=.001;t=np.arange(3001)*dt;e=np.where(t<2,1.,0.)
        fig,(ax,bx)=plt.subplots(1,2,figsize=(8.6,2.75),layout='constrained')
        for kaw,c,label in [(0,RED,'No anti-windup'),(10,GREEN,'Back-calculation')]:
            integral=0.;history=[];raw=[];applied=[]
            for ek in e:
                request=.2*ek+integral;limited=np.clip(request,0,.45)
                history.append(integral);raw.append(request);applied.append(limited)
                integral+=dt*(ek+kaw*(limited-request))
            ax.plot(t,history,c=c,label=label);bx.plot(t,applied,c=c,label=label)
        ax.set(xlabel='Time (s)',ylabel='Integral contribution I',title='Stored integral: bounded versus growing')
        bx.axhline(.45,c=GRAY,ls=':',lw=.8);bx.set(xlabel='Time (s)',ylabel='Bounded output u',title='At 2 s, scripted error becomes zero')
        for target in (ax,bx):target.axvline(2,c=GRAY,ls=':',lw=.8);target.grid(alpha=.3);target.legend(fontsize=7)
        save(fig,'anti-windup-example','Synthetic integrator-only experiment: e=1 for2s then0; Kp=0.2, Ki=1, Kd=0, limits[0,0.45], Δt=1ms, Kaw=10/s. No aircraft, sensor, actuator or closed-loop plant is simulated. Units are normalized controller-output units; 0.45 is an example limit, not a flight-tuning result.','ILLUSTRATIVE ANTI-WINDUP RECURRENCE')
''' +s[end:]
p.write_text(s,encoding='utf-8',newline='\n')

for folder in ['report','full']:
    p=root/folder/'report.md';s=p.read_text(encoding='utf-8')
    s=s.replace('the plot shows the useful local view.','panel A retains every sampled frequency branch and panel B zooms around -1. The origin pole sends the low-frequency branches to infinity; a finite picture cannot contain that limit at linear scale.')
    p.write_text(s,encoding='utf-8',newline='\n')

p=root/'full/report.md';s=p.read_text(encoding='utf-8')
marker='---\n## A5. Frequency model'
extra=r'''---
## A4.1 Anti-windup | why return the saturation error?

[[VIS:anti-windup-example]]

[[EQ:u_{raw}=v+I,\quad v=K_pe+D,\quad u=\operatorname{clip}(u_{raw},u_{min},u_{max})]]

[[EQ:\dot I=K_ie+K_{aw}(u-u_{raw})]]

**Derive the sign:** upper saturation means u < u_raw. Their difference is negative, so it opposes further integral accumulation. Lower saturation reverses this sign. Without saturation u = u_raw, so ordinary integration is recovered exactly.

Hold e and v constant during upper saturation. Substitute u = u_max and collect I terms:

[[EQ:\dot I+K_{aw}I=K_ie+K_{aw}(u_{max}-v),\quad I_* =u_{max}-v+\frac{K_ie}{K_{aw}}]]

[[EQ:I(t)=I_*+[I(0)-I_*]e^{-K_{aw}t},\quad T_{aw}=1/K_{aw}]]

This solution holds only while that saturation regime persists. Kaw has units 1/s; T_aw is the tracking time constant. The integrator approaches a finite value rather than growing indefinitely. It need not become zero when the error becomes zero: a nonzero integral can supply steady load compensation.

For explicit Euler, the tracking error multiplier is 1 - Δt Kaw. Its isolated constant-regime recurrence decays if 0 < Δt Kaw < 2; monotonic decay needs at most 1. This condition alone does not prove closed-loop or switching stability.

**Quadcopter integration gate:** actual motor clipping happens after mixing. Recover achievable collective/torques through a validated allocator before feeding an equivalent output error back to each integrator; subtracting raw motor PWM directly from axis torque is dimensionally wrong. This experiment changes no firmware or motor policy.

'''
assert marker in s;s=s.replace(marker,extra+marker,1)
marker='---\n## A7. Bode'
extra=r'''---
## A6.1 Nyquist | where do the large turns come from?

The loop contains Ki/s. Near s = 0 the derivative term vanishes, the actuator and Padé factors tend to one, and the damped rate plant tends to 1/d. Therefore:

[[EQ:C(s)=K_i/s+O(1),\quad G(s)=1/d+O(s),\quad L_\mu(s)=\frac{\mu K_i}{d s}+O(1)]]

[[EQ:L_\mu(j\omega)\sim-j\frac{\mu K_i}{d\omega},\qquad |L_\mu(j\omega)|\longrightarrow\infty\quad(\omega\to0^+)]]

**Meaning:** the apparent missing ends in a linear close-up are not missing data or a finite oval. They extend without bound. The conjugate branch goes upward. Both are included in the symmetric-log overview; the linear close-up retains undistorted distance to -1.

On the right indentation s = ε exp(jθ), θ increases from -π/2 to +π/2. Substitution gives:

[[EQ:L_\mu(\epsilon e^{j\theta})\sim\frac{\mu K_i}{d\epsilon}e^{-j\theta},\quad |L|\approx\frac{\mu K_i}{d\epsilon}]]

For ε = 10^-4, Ki = 0.2 and d = 0.01, the large mapped arc has approximate radius 200000 μ: 200000 for baseline and 1600000 for μ = 8. This joins the two low-frequency ends through the right side in the leading-order approximation. The exact rational values, not this asymptote, are plotted in A6.

| Contour piece, in traversal order | Image in the loop plane |
|---|---|
| Negative branch: -jR to -jε | Conjugate frequency branch |
| Right origin indentation | Large connecting arc; do not omit |
| Positive branch: +jε to +jR | Positive-frequency branch |
| Clockwise outer RHP arc | Tiny closure near L = 0 as R grows |

The symmetric-log transform is continuous, one-to-one and increasing on each axis. It preserves whether a closed curve encloses the transformed critical point, but changes lengths, angles and curvature. Compute Δarg(1+L) from the original complex values, never from displayed distances. The actual characteristic-root count independently checks the winding result.

'''
assert marker in s;s=s.replace(marker,extra+marker,1);p.write_text(s,encoding='utf-8',newline='\n')

p=root/'report/editions.json';d=json.loads(p.read_text());d['full']['expected_pages']=55;p.write_text(json.dumps(d,indent=2)+'\n',encoding='utf-8')
p=root/'README.md';s=p.read_text(encoding='utf-8').replace('53 pages','55 pages').replace('Applied derivations, Bode,','Complete Nyquist contours, anti-windup proof, applied derivations, Bode,');p.write_text(s,encoding='utf-8',newline='\n')
