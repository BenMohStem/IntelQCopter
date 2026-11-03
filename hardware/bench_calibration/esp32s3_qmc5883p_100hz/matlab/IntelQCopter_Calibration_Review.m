function summary = IntelQCopter_Calibration_Review(csvFiles, calibrationJSON)
% IntelQCopter offline MATLAB calibration and timestamp-validation lab.
% MATLAB R2021b+; base MATLAB (no Statistics/Optimization toolbox required).
% Usage:
%   report = IntelQCopter_Calibration_Review({'captures/a.csv','captures/b.csv'}, ...
%       'captures/combined_calibration.json');
% CSV checkpoint files can be passed as a cell array/string array.
% Uses only accepted and fresh samples; NEVER uses a LiDAR measurement
% unless the user separately logs it and validates geometry/latency.
if ischar(csvFiles) || isstring(csvFiles), csvFiles = cellstr(csvFiles); end
if nargin<2, calibrationJSON=''; end
T=table();
for i=1:numel(csvFiles)
    U=readtable(csvFiles{i},'TextType','string');
    if ~ismember('phase_accepted',U.Properties.VariableNames)
        U.phase_accepted=ones(height(U),1);
    end
    T=[T; U]; %#ok<AGROW> preserve all labeled rows for phase statistics
end
if isempty(T),error('No calibration CSV rows. Capture phases first.');end
ok = T.phase_accepted == 1;
mpu = ok & T.mpu_ok == 1;
mag = ok & T.mag_ok == 1 & T.mag_new == 1 & T.pose == "MAG";
bmp = ok & T.bmp_ok == 1 & T.bmp_new == 1 & (T.pose == "REST" | T.pose == "BMP");
rest = mpu & T.pose == "REST";
summary = struct();
summary.rows=height(T);
summary.mpu_valid=sum(mpu);
summary.mag_fresh=sum(mag);
summary.bmp_fresh=sum(bmp);
if sum(rest)>=100
    G=[T.gx_dps(rest),T.gy_dps(rest),T.gz_dps(rest)];
    summary.gyro_bias_dps=mean(G,1);
    summary.gyro_sigma_dps=std(G,0,1);
    fprintf('Gyro bias (deg/s): [%+.5f %+.5f %+.5f]\n',summary.gyro_bias_dps);
else
    warning('Fewer than 100 stationary REST readings; gyro bias unavailable.');
end
poses=["+X","-X","+Y","-Y","+Z","-Z"];
means=zeros(6,3);valid=true;
for p=1:6
    rows = mpu & T.pose == poses(p);
    if sum(rows)<100
        warning('%s has <100 accepted MPU readings',poses(p));valid=false;continue;
    end
    means(p,:)=mean([T.ax_mps2(rows),T.ay_mps2(rows),T.az_mps2(rows)],1);
end
if valid
    % Each axis comes from its own + / - face pair; gravity in m/s².
    summary.accel_bias_mps2=0.5*[means(1,1)+means(2,1), means(3,2)+means(4,2), means(5,3)+means(6,3)];
    span=[means(1,1)-means(2,1), means(3,2)-means(4,2), means(5,3)-means(6,3)];
    summary.accel_scale_diagonal=2*9.80665./span;
    fprintf('Acceleration bias (m/s²): [%+.5f %+.5f %+.5f]\n', summary.accel_bias_mps2);
    fprintf('Accel diagonal scale: [%.6f %.6f %.6f]\n',summary.accel_scale_diagonal);
end
if sum(bmp)>30
    pp=T.pressure_pa(bmp);
    summary.p0_pa=median(pp);
    summary.baro_sigma_pa=std(pp);
    fprintf('Barometer p0 (reference only) %.2f Pa; sigma %.3f Pa\n',summary.p0_pa,summary.baro_sigma_pa);
end
% Packet timing uses MCU read timestamps, not Windows serial arrival times.
% CSV concatenation may introduce reboot/phase gaps: analyze PER FILE instead.
perFile=cell(numel(csvFiles),1);
for i=1:numel(csvFiles)
    U=readtable(csvFiles{i});
    t=double(U.t_us);s=double(U.seq);
    dt=mod(diff(t),2^32);ds=mod(diff(s),2^32);
    good=dt>0 & dt<1e6 & ds==1;
    d=dt(good);
    if isempty(d),perFile{i}=struct('name',csvFiles{i},'rate_hz',NaN);continue;end
    perFile{i}=struct('name',csvFiles{i},'rate_hz',1e6/mean(d), ...
        'dt_p95_us',localQuantile(d,0.95),'dt_p99_us',localQuantile(d,0.99), ...
        'gaps_over_10ms',sum(d>10000));
    fprintf('%s : timestamp rate %.2f Hz, p99 %.0f us, >10ms %d\n', ...
        csvFiles{i},perFile{i}.rate_hz,perFile{i}.dt_p99_us,perFile{i}.gaps_over_10ms);
end
summary.timing_by_file=perFile;
if sum(mag)>100
    M=[T.mx_uT(mag),T.my_uT(mag),T.mz_uT(mag)];
    % Independently fit a robust SPD ellipsoid using BASE MATLAB fminsearch.
    % This is intentionally an independent numerical implementation; compare
    % its center and residual against Python SciPy, not just copied constants.
    try
        [bFit,SFit,magCV] = fitEllipsoidBaseMATLAB(M);
        summary.matlab_mag_bias_uT=bFit;
        summary.matlab_mag_matrix_3x3=SFit;
        summary.matlab_mag_radial_cv=magCV;
        fprintf('Independent MATLAB MAG radial CV %.3f%% (no heading truth)\n',100*magCV);
    catch ME
        warning('MATLAB ellipsoid fit unavailable: %s',ME.message);
    end
    % From the raw magnetic vectors alone no accurate absolute yaw can be certified.
    % For cross-language comparisons use Python-fitted robust SPD matrix.
    fig=figure('Name','IntelQCopter QMC5883P independent review');
    subplot(1,2,1); scatter3(M(1:5:end,1),M(1:5:end,2),M(1:5:end,3),8,'filled');
    axis equal;grid on; xlabel('X / µT');ylabel('Y / µT');zlabel('Z / µT');title('Raw MAG');
    if strlength(string(calibrationJSON))>0
        C=jsondecode(fileread(calibrationJSON));
        if isfield(C,'magnetometer')
            b=reshape(C.magnetometer.hard_iron_offset_uT,1,[]);
            S=C.magnetometer.soft_iron_matrix_3x3_dimensionless;
            Corrected=(M-b)*S.';
            subplot(1,2,2);
            scatter3(Corrected(1:5:end,1),Corrected(1:5:end,2),Corrected(1:5:end,3),8,'filled');
            axis equal;grid on; xlabel('X / µT');ylabel('Y / µT');zlabel('Z / µT');title('Python SPD calibration replay');
            rad=sqrt(sum(Corrected.^2,2));
            summary.calibrated_magnetic_radius_cv=std(rad)/mean(rad);
            fprintf('Calibrated MAG radial CV: %.3f%% (in-sample, not heading truth)\n',100*summary.calibrated_magnetic_radius_cv);
        end
    end
    drawnow;
end
end

function q=localQuantile(x,f)
% Simple linear interpolation, avoids Statistics Toolbox requirement.
x=sort(x(:));a=1+(numel(x)-1)*f;lo=floor(a);hi=ceil(a);
q=(hi-a)*x(lo)+(a-lo)*x(hi);
if lo==hi,q=x(lo);end
end

function [b,S,radialCV]=fitEllipsoidBaseMATLAB(raw)
% Robust positive-definite normalized 9-parameter ellipsoid fit, no toolbox.
% Training subset is balanced by equal TIME stride; coverage must be audited
% with held-out samples and separate physical sessions.
raw=double(raw);
if size(raw,1)<300,error('Need >=300 fresh QMC samples');end
x=raw(1:max(1,ceil(size(raw,1)/1700)):end,:);
mu=mean(x,1);sc=mean(max(x,[],1)-min(x,[],1));
if sc<1e-5,error('Degenerate magnetic extent');end
v=(x-mu)/sc;
c0=0.5*(max(v,[],1)+min(v,[],1));
C=cov(v);[U,D]=eig(C);
if min(diag(D))<1e-6,error('Degenerate orientation coverage');end
S0=U*diag(1./sqrt(3*diag(D)))*U.';
lengths=sqrt(sum(((v-c0)*S0.').^2,2));
S0=S0/median(lengths);
L0=chol(S0,'lower');
p0=[c0,log(L0(1,1)),L0(2,1),log(L0(2,2)), ...
    L0(3,1),L0(3,2),log(L0(3,3))];
opts=optimset('Display','off','MaxIter',1800,'MaxFunEvals',15000,'TolX',1e-9);
p=fminsearch(@(par) robustMagCost(par,v),p0,opts);
b=mu+sc*p(1:3);
Su=magSPD(p)/sc; % 1/uT, maps corrected vectors to unit sphere
radius=median(sqrt(sum((raw-b).^2,2)));
S=radius*Su;  % dimensionless, preserves nominal magnetic field magnitude
rad=sqrt(sum(((raw-b)*Su.').^2,2));
radialCV=std(rad)/mean(rad);
end

function z=robustMagCost(p,v)
try
    M=magSPD(p);
    r=sqrt(sum(((v-p(1:3))*M.').^2,2))-1;
    z=mean(2*.025^2*(sqrt(1+(r/.025).^2)-1)); % soft-L1 residual
    if ~isfinite(z),z=realmax;end
catch
    z=realmax;
end
end

function M=magSPD(p)
L=[exp(p(4)),0,0;p(5),exp(p(6)),0;p(7),p(8),exp(p(9))];
M=L*L.';
end
