#!/usr/bin/env python3
"""Independent, research-informed magnetometer diagnostics for IntelQCopter.

Fits an SPD ellipsoid to *training* magnetic observations and checks a
stage-stratified chronological holdout; evaluates 3D coverage, block bootstrap
sensitivity, and independent-session generalization. No heading truth inferred.

See ../README.md, source chapter MAGNETOMETER_VALIDATION_METHOD.md, for limits and references.
"""
from __future__ import annotations
import argparse, csv, json, math, re
from pathlib import Path
from collections import defaultdict
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

COLUMNS = ('mx_uT','my_uT','mz_uT')
STAGES = ('YAW','PITCH','ROLL','TUMBLE')


def collect(files):
    from attempt_filter import only_latest
    out=defaultdict(list)
    per_file={}
    all_rows=[]
    for file in sorted(files,key=lambda p:str(Path(p))):
        file=Path(file)
        with file.open(newline='',encoding='utf-8-sig') as handle:
            for row in csv.DictReader(handle):
                row['_source_file']=str(file.resolve())
                all_rows.append(row)
    for row in only_latest(all_rows):
        if (row.get('pose')!='MAG' or row.get('mag_ok')!='1'
            or row.get('mag_new')!='1' or row.get('phase_accepted','1')!='1'):
            continue
        try:v=[float(row[k]) for k in COLUMNS]
        except (ValueError,KeyError,TypeError):continue
        if not np.all(np.isfinite(v)):continue
        typ=row.get('mag_sensor_type','unknown')
        if typ not in ('QMC5883P','QMC5883L','HMC5883L','unknown'):continue
        file=Path(row['_source_file'])
        stem=re.sub(r'_part\d+$','',file.stem)
        key=str(file.parent/stem)
        out[key].append((row.get('mag_stage','UNKNOWN'),v,typ))
        per_file[str(file)]=per_file.get(str(file),0)+1
    return out,per_file


def unit_vectors(x):
    n = np.linalg.norm(x,axis=1)
    return x[n>1e-9]/n[n>1e-9,None]


def sphere_coverage(x, dirs=128, degrees=18):
    u=unit_vectors(x)
    if len(u)<3:return {'coverage_fraction':0.0,'direction_bins_occupied':0,'direction_entropy_normalized':0.0}
    idx = np.arange(dirs)+.5
    z = 1-2*idx/dirs
    theta = idx*math.pi*(3-math.sqrt(5))
    grid = np.column_stack((np.sqrt(1-z*z)*np.cos(theta),np.sqrt(1-z*z)*np.sin(theta),z))
    tree = cKDTree(u)
    distances,_ = tree.query(grid)
    coverage = float(np.mean(distances <= math.sqrt(2-2*math.cos(math.radians(degrees)))))
    nearest=cKDTree(grid).query(u)[1]
    bins=np.bincount(nearest,minlength=dirs)
    p=bins[bins>0]/len(u)
    entropy=-float(np.sum(p*np.log(p)))/math.log(dirs)
    return {'directions_tested':dirs,'coverage_angle_deg':degrees,
            'coverage_fraction':coverage,'direction_bins_occupied':int(np.count_nonzero(bins)),
            'direction_entropy_normalized':entropy,
            'least_covered_grid_direction_ids':[int(i) for i in np.argsort(-distances)[:8]],
            'sampling_imbalance_note':'Repeated samples in one orientation do not increase coverage.'}


def params_to_spd(p):
    L=np.array([[np.exp(p[3]),0,0],[p[4],np.exp(p[5]),0],
                [p[6],p[7],np.exp(p[8])]])
    return L@L.T


def fit_spd(raw, limit=1900):
    """Normalized robust 9-parameter SPD ellipsoid fit; bias in uT."""
    raw=np.asarray(raw,dtype=float)
    if len(raw)<100:raise ValueError('Insufficient fit samples')
    if len(raw)>limit:
        idx=np.linspace(0,len(raw)-1,limit,dtype=int)
        raw=raw[idx]
    mu=raw.mean(axis=0)
    scale=float(np.mean(np.ptp(raw,axis=0)))
    if scale<1e-5:raise ValueError('Degenerate input span')
    v=(raw-mu)/scale
    b0=(np.max(v,axis=0)+np.min(v,axis=0))*.5
    cov=np.cov(v.T)
    eig,vecs=np.linalg.eigh(cov)
    if eig[0]<1e-8:raise ValueError('Almost planar measurements (ill-conditioned ellipsoid)')
    S0=vecs @ np.diag(1/np.sqrt(np.maximum(eig,1e-8))) @ vecs.T
    radius=np.median(np.linalg.norm((v-b0)@S0.T,axis=1))
    S0/=max(radius,1e-9)
    L=np.linalg.cholesky(S0)
    p0=np.array([*b0,np.log(L[0,0]),L[1,0],np.log(L[1,1]),L[2,0],L[2,1],np.log(L[2,2])])
    def r(p):
        S=params_to_spd(p)
        return np.linalg.norm((v-p[:3])@S.T,axis=1)-1
    solution=least_squares(r,p0,loss='soft_l1',f_scale=.03,max_nfev=220,
                           bounds=([-5,-5,-5,-6,-15,-6,-15,-15,-6],
                                   [5,5,5,6,15,6,15,15,6]))
    if not solution.success:raise ValueError('Ellipsoid fit not converged: '+solution.message)
    b=mu+scale*solution.x[:3]
    Sun=params_to_spd(solution.x)/scale
    # Preserve the raw radius in microteslas; no external field reference.
    R=float(np.median(np.linalg.norm(raw-b,axis=1)))
    return b,Sun*R,Sun


def normalized_residual(x,b,Sun):
    return np.linalg.norm((x-b)@Sun.T,axis=1)-1


def summarize_residual(r):
    r=np.asarray(r)
    if not len(r):return None
    return {'count':len(r),'rms_fraction':float(np.sqrt(np.mean(r*r))),
            'p95_absolute_fraction':float(np.percentile(np.abs(r),95)),
            'p99_absolute_fraction':float(np.percentile(np.abs(r),99)),
            'median_fraction':float(np.median(r))}


def stage_split(entries):
    by=defaultdict(list)
    for stage,v,t in entries:by[stage].append(v)
    train=[];test=[];counts={}
    for stage,values in by.items():
        a=np.asarray(values)
        if len(a)<10:train.extend(a);counts[stage]=len(a);continue
        boundary=max(8,int(len(a)*.8))
        train.extend(a[:boundary]);test.extend(a[boundary:])
        counts[stage]=len(a)
    return np.asarray(train).reshape(-1,3),np.asarray(test).reshape(-1,3),counts


def analyze(sessions,bootstrap=12):
    if not sessions:raise ValueError('No accepted fresh MAG records in supplied CSVs')
    types=set()
    for s in sessions.values():types.update(t for _,_,t in s)
    if len(types)>1:raise ValueError('Cannot combine sensor types: '+str(sorted(types)))
    all_entries=[entry for sess in sessions.values() for entry in sess]
    values=np.asarray([v for stage,v,t in all_entries])
    if len(values)<300:raise ValueError('Need >=300 fresh magnetic samples')
    # Stage-chronological holdout, preferable to random per-sample splitting.
    train=[];test=[];stages=defaultdict(int)
    for key,entries in sessions.items():
        tr,te,cnts=stage_split(entries)
        train.extend(tr);test.extend(te)
        for st,n in cnts.items():stages[st]+=n
    train=np.asarray(train).reshape(-1,3)
    test=np.asarray(test).reshape(-1,3)
    b,S,Sun=fit_spd(train)
    cv_train=summarize_residual(normalized_residual(train,b,Sun))
    cv_test=summarize_residual(normalized_residual(test,b,Sun))
    full_b,full_S,full_Sun=fit_spd(values)
    corr=(values-full_b)@full_Sun.T
    coverage=sphere_coverage(corr)
    # Cross-session generalization: evaluate fit on each other independently recorded
    # experiment; checkpoint pieces have already been grouped by session ID.
    generalization={}
    for key,entries in sessions.items():
        if len(sessions)<2:break
        held=np.asarray([v for _,v,_ in entries])
        others=np.asarray([v for k,ent in sessions.items() if k!=key for _,v,_ in ent])
        if len(held)<100 or len(others)<300:continue
        try:
            bb,ss,su=fit_spd(others)
            generalization[Path(key).name]=summarize_residual(normalized_residual(held,bb,su))
        except ValueError as exc:generalization[Path(key).name]={'warning':str(exc)}
    # Block bootstrap: resample time-contiguous ~1-second groups, not individual
    # correlated 50 Hz observations. Stratify by motion stage and session.
    rng=np.random.default_rng(20261102)
    blocks=[]
    for entries in sessions.values():
        by=defaultdict(list)
        for stage,v,t in entries:by[stage].append(v)
        for stage, vv in by.items():
            v=np.asarray(vv)
            blocks.extend([v[i:i+50] for i in range(0,len(v),50) if len(v[i:i+50])>=5])
    estimates=[]
    if len(blocks)>=8:
        nrep=min(bootstrap,40)
        for it in range(nrep):
            chosen=rng.integers(0,len(blocks),size=len(blocks))
            data=np.vstack([blocks[i] for i in chosen])
            try:
                bb,ss,su=fit_spd(data,limit=1100)
                estimates.append((bb,ss))
            except (ValueError,OverflowError,np.linalg.LinAlgError):pass
    bootstrap_result={'attempted':min(bootstrap,40),'successful':len(estimates)}
    if len(estimates)>=5:
        centers=np.asarray([a for a,b in estimates]);mats=np.asarray([b for a,b in estimates])
        bootstrap_result.update({
            'bias_uT_p2_5':np.percentile(centers,2.5,axis=0).tolist(),
            'bias_uT_p97_5':np.percentile(centers,97.5,axis=0).tolist(),
            'soft_matrix_p2_5':np.percentile(mats,2.5,axis=0).tolist(),
            'soft_matrix_p97_5':np.percentile(mats,97.5,axis=0).tolist(),
            'interpretation':'Empirical block-bootstrap sensitivity intervals, not metrology-grade confidence or heading accuracy bounds.'})
    # Model sanity: condition number of full SPD shape, rather than raw covariance.
    eig=np.linalg.eigvalsh(full_S)
    warnings=[]
    if coverage['coverage_fraction']<.90:warnings.append('Orientation coverage below 90% at 18-degree radius')
    if cv_test and cv_test['rms_fraction']>max(.05,2*cv_train['rms_fraction']):
        warnings.append('Withheld-stage radial error is large relative to training: possible distribution shift')
    if eig[0]<=0 or eig[-1]/eig[0]>4:warnings.append('Large fitted correction anisotropy: check mounting/field distortion')
    if len(sessions)<2:warnings.append('Only one independent session: no truly independent holdout. Repeat full 3D rotations as a NEW recording.')
    if any(s['rms_fraction']>.06 for s in generalization.values() if 'rms_fraction' in s):
        warnings.append('One or more leave-one-session-out residuals exceed 6%: unstable field or calibration')
    return {'schema':'IQ_MAG_QUALITY_RESEARCH_V1','sensor_types':sorted(types),
      'fresh_magnetic_rows':len(values),'stage_counts':dict(stages),
      'independent_sessions':len(sessions),
      'train_holdout_method':'Last 20% of each motion stage withheld chronologically; temporal adjacency remains, so this is not independent heading truth.',
      'training_radial_error':cv_train,'heldout_radial_error':cv_test,
      'leave_one_session_out':generalization,
      'full_fit':{'hard_iron_offset_uT':full_b.tolist(),
                  'symmetric_correction_matrix':full_S.tolist(),
                  'shape_condition_number':float(eig[-1]/eig[0]),
                  'radial_error':summarize_residual(normalized_residual(values,full_b,full_Sun))},
      'coverage':coverage,'bootstrap':bootstrap_result,'warnings':warnings,
      'limits':['No magnetic heading accuracy without external orientation/field reference.',
                'Spatial translation is harmless only when the external magnetic field is uniform.',
                'A connected USB cable, current, steel screws, and motors can alter the field.',
                'A low ellipsoid radial residual does not establish calibration axis alignment or full heading accuracy.']}


def plot_quality(sessions,report,filename):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    values=np.asarray([v for entries in sessions.values() for _,v,_ in entries])
    B=np.asarray(report['full_fit']['hard_iron_offset_uT'])
    S=np.asarray(report['full_fit']['symmetric_correction_matrix'])
    radius=float(np.median(np.linalg.norm(values-B,axis=1)))
    corrected=(values-B)@S.T
    rng=np.random.default_rng(4)
    idx=np.sort(rng.choice(len(values),size=min(3500,len(values)),replace=False))
    fig=plt.figure(figsize=(12,5))
    ax=fig.add_subplot(121,projection='3d')
    v=values[idx]
    ax.scatter(v[:,0],v[:,1],v[:,2],c=v[:,2],s=1,alpha=.32,cmap='viridis')
    ax.set_title('Original magnetic measurements')
    ax.set_xlabel('X (µT)');ax.set_ylabel('Y (µT)');ax.set_zlabel('Z (µT)')
    ax2=fig.add_subplot(122,projection='3d')
    v=corrected[idx]
    ax2.scatter(v[:,0],v[:,1],v[:,2],c=v[:,2],s=1,alpha=.32,cmap='viridis')
    ax2.set_title('Ellipsoid-corrected (in-sample)')
    ax2.set_xlabel('X (µT)');ax2.set_ylabel('Y (µT)');ax2.set_zlabel('Z (µT)')
    for a in (ax,ax2):
        a.set_box_aspect((1,1,1));a.grid(True)
    fig.tight_layout()
    fig.savefig(filename,dpi=140)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv',nargs='+',type=Path,help='CSV captures; checkpoints _partNN belong to same session')
    parser.add_argument('--output-prefix',type=Path,default=None)
    parser.add_argument('--bootstrap',type=int,default=12,help='0 to disable; max 40')
    ns=parser.parse_args()
    sessions,counts=collect(ns.csv)
    report=analyze(sessions,bootstrap=max(0,ns.bootstrap))
    report['input_files']=[str(f) for f in ns.csv]
    report['fresh_rows_by_file']=counts
    prefix=ns.output_prefix or ns.csv[0].with_suffix('').with_name(ns.csv[0].stem+'_mag_quality')
    prefix.parent.mkdir(parents=True,exist_ok=True)
    output_json=Path(str(prefix)+'.json')
    output_png=Path(str(prefix)+'.png')
    output_json.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    plot_quality(sessions,report,output_png)
    print('Fresh magnetic samples:',report['fresh_magnetic_rows'])
    print('Session count:',report['independent_sessions'])
    print('3D coverage:',f"{report['coverage']['coverage_fraction']*100:.1f}%")
    if report['heldout_radial_error']:
        print('Withheld radial RMS:',f"{report['heldout_radial_error']['rms_fraction']*100:.2f}%")
    for w in report['warnings']:print('WARNING:',w)
    print('JSON:',output_json,'\nPNG:',output_png)

if __name__=='__main__':main()
