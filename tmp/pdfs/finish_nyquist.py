from pathlib import Path

root=Path(r'C:\Users\Mohamed\Desktop\Work\IntelQCopter-USTHB')
p=root/'tools/advanced_figures.py';s=p.read_text(encoding='utf-8')
s=s.replace("        ax.legend(fontsize=8,loc='upper right');bx.annotate", "        ax.set_xticks([-10,-1,0,100,10000,1000000]);ax.set_yticks([-1000000,-1000,-1,0,1,1000,1000000]);ax.tick_params(labelsize=8)\n        ax.legend(fontsize=7,loc='upper right');bx.annotate")
s=s.replace("bx.grid(alpha=.3);bx.legend(fontsize=6.4,loc='upper left')", "bx.set_xticks([-10,-1,0,100,10000,1000000]);bx.set_yticks([-1000000,-1000,-1,0,1,1000,1000000]);bx.tick_params(labelsize=8);bx.grid(alpha=.3);bx.legend(fontsize=6.4,loc='upper left')")
# Store a quantitative plot-coverage check alongside the rendered figure manifest.
needle="        save(fig,'nyquist-example'"
pos=s.index(needle)
check='''        coverage=[]
        for mu in [1.,8.]:
            points=np.concatenate((-1j*w[::-1],1e-4*np.exp(1j*np.linspace(-np.pi/2,np.pi/2,3000)),1j*w,1e6*np.exp(1j*np.linspace(np.pi/2,-np.pi/2,3000))))
            z=response(points,mu);limits=(ax.get_xlim(),ax.get_ylim())
            assert np.isfinite(z).all()
            assert limits[0][0]<=z.real.min()<=z.real.max()<=limits[0][1]
            assert limits[1][0]<=z.imag.min()<=z.imag.max()<=limits[1][1]
            f=1+np.append(z,z[0]);turns=-np.angle(f[1:]/f[:-1]).sum()/(2*np.pi)
            expected=next(case['clockwise_encirclements_N'] for case in data['cases'] if case['gain_multiplier']==mu)
            assert abs(turns-expected)<1e-6
            coverage.append({'gain_multiplier':mu,'sample_count':len(z),'every_point_inside_overview':True,'clockwise_turns':float(turns),'expected_from_roots':expected,'real_bounds':[float(z.real.min()),float(z.real.max())],'imag_bounds':[float(z.imag.min()),float(z.imag.max())]})
'''
s=s[:pos]+check+s[pos:]
end=s.index("    if 'poles-3d' in selected:")
s=s[:end]+"        records['nyquist-example']['coverage_checks']=coverage\n"+s[end:]
p.write_text(s,encoding='utf-8',newline='\n')
p=root/'tools/build_report.py';s=p.read_text(encoding='utf-8').replace('REV 7.0','REV 7.1').replace(" / 7.0'"," / 7.1'");p.write_text(s,encoding='utf-8',newline='\n')
p=root/'tools/check_report.py';s=p.read_text(encoding='utf-8');needle="    for entry in json.loads((folder/'evidence-manifest.json').read_text()):"
check="""    for case in figures['nyquist-example']['coverage_checks']:
        assert case['every_point_inside_overview'] and case['sample_count']==42000
        assert abs(case['clockwise_turns']-case['expected_from_roots'])<1e-6
    assert len(figures['nyquist-example']['coverage_checks'])==2
"""
assert needle in s;s=s.replace(needle,check+needle);p.write_text(s,encoding='utf-8',newline='\n')
