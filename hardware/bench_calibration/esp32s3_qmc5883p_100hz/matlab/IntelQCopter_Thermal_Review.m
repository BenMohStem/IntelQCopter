function report = IntelQCopter_Thermal_Review(csvfiles)
% INTELQCOPTER_THERMAL_REVIEW  Independent Matlab session-level thermal audit.
% report=IntelQCopter_Thermal_Review({"captures/cold.csv","captures/room.csv","captures/warm.csv"});
% Research/analysis only. NO motor output. DOES NOT fit BMP pressure bias without a reference.
%
% CSV checkpoints with suffix _partXX are the SAME independent experiment.
if ischar(csvfiles) || isstring(csvfiles), csvfiles=cellstr(csvfiles); end
assert(iscell(csvfiles) && ~isempty(csvfiles),'Supply a cell array of filenames');
keys = strings(0); groups=cell(0,1);
for i=1:numel(csvfiles)
    f=string(csvfiles{i});
    [~,stem,~]=fileparts(f);
    stem=regexprep(stem,'_part[0-9]+$','');
    ix=find(keys==stem,1);
    if isempty(ix),keys(end+1)=stem;groups{end+1}=table();ix=numel(keys);end
    tbl=readtable(f,'VariableNamingRule','preserve');
    if ~ismember('phase_accepted',tbl.Properties.VariableNames), tbl.phase_accepted=ones(height(tbl),1);end
    groups{ix}=[groups{ix};tbl];
end
N=numel(groups); bias=nan(N,3);gt=nan(N,1);gyroStd=nan(N,3);
bmpTemp=nan(N,1);bmpPressure=nan(N,1);bmpN=zeros(N,1);
for i=1:N
    t=groups{i};
    rest=strcmp(string(t.pose),'REST') & t.mpu_ok==1 & t.phase_accepted==1;
    cols={'gx_dps','gy_dps','gz_dps'};
    if nnz(rest)>=100
        V=[t.gx_dps(rest),t.gy_dps(rest),t.gz_dps(rest)];
        bias(i,:)=mean(V,1,'omitnan');gyroStd(i,:)=std(V,0,1,'omitnan');
        gt(i)=median(t.mpu_temp_c(rest),'omitnan');
        fprintf('%s: gyro mean [deg/s]=%+.5f %+.5f %+.5f @ %.2f C\n',keys(i),bias(i,:),gt(i));
    end
    b=(strcmp(string(t.pose),'BMP')|strcmp(string(t.pose),'REST')) & t.bmp_ok==1 & t.bmp_new==1 & t.phase_accepted==1;
    if nnz(b)>=30
        bmpN(i)=nnz(b);bmpTemp(i)=median(t.bmp_temp_c(b),'omitnan');
        bmpPressure(i)=median(t.pressure_pa(b),'omitnan');
    end
end
valid=isfinite(gt)&all(isfinite(bias),2)&all(gyroStd<0.35,2);
report=struct();report.names=keys;report.gyroTempC=gt;report.gyroBiasDps=bias;
report.gyroNoiseStdDps=gyroStd;report.bmpTemperatureC=bmpTemp;
report.bmpPressurePa=bmpPressure;report.bmpFreshN=bmpN;
if nnz(valid)>=3 && range(gt(valid))>=3
    tref=median(gt(valid));X=[ones(nnz(valid),1),gt(valid)-tref];
    beta=X\bias(valid,:);
    report.referenceTempC=tref;report.gyroBiasInterceptDps=beta(1,:);
    report.gyroBiasSlopeDpsPerC=beta(2,:);
    report.status='EXPLORATORY_THERMAL_FIT_NOT_FLIGHT_VALIDATED';
    fprintf('Exploratory gyro slope [deg/s/C]: %+g %+g %+g\n',beta(2,:));
else
    report.status='INSUFFICIENT_INDEPENDENT_THERMAL_SESSIONS';
    fprintf('Need >=3 valid stationary sessions spanning >=3 C.\n');
end
figure('Name','IntelQCopter thermal gyro review');
tiledlayout(2,1);
nexttile;plot(gt,bias,'o-');grid on;
xlabel('MPU6050 die temperature [C]');ylabel('Gyro bias [deg/s]');legend('X','Y','Z');
nexttile;plot(bmpTemp,bmpPressure,'o-');grid on;
xlabel('BMP280 die temperature [C]');ylabel('Pressure [Pa]');
title('DIAGNOSTIC ONLY: no independent pressure reference');
end
