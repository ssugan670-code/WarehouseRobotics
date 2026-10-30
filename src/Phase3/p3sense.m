function ok = p3sense()
%P3SENSE  Check-up of the Round 12 sensing (lidar localization, camera carton detection, lidar tracking / protective field).  Run it BEFORE phase3:
%       >> p3sense
%   It needs MATLAB (Navigation Toolbox, Sensor Fusion and Tracking Toolbox, Simulink 3D Animation + Unreal lidar blocks) but NO Unreal / 3-D window.
%   Prints one PASS / FAIL line per check and writes everything to  <Phase3 data>\out\Logs\p3sense_<time>.txt  - send that file if something fails.
P = phase3Paths(); if ~isfolder(P.diag), mkdir(P.diag); end
lf = fullfile(P.diag,['p3sense_' datestr(now,'yyyymmdd_HHMMSS') '.txt']); fid = fopen(lf,'w');
res = {}; 
say = @(varargin) localSay(fid,varargin{:});
say('p3sense  %s   MATLAB %s\n',datestr(now),version);
% ---- 1. toolboxes / functions / blocks
need = {'monteCarloLocalization','likelihoodFieldSensorModel','odometryMotionModel','occupancyMap','lidarScan','multiObjectTracker','objectDetection','initcvkf','rigidBodyTree'};
miss = need(cellfun(@(f) isempty(which(f)),need));
res(end+1,:) = {'toolbox functions',isempty(miss),strjoin(miss,', ')};
v = ver; vn = {v.Name}; vn = vn(~cellfun(@isempty,regexpi(vn,'Navigation|Sensor Fusion|Automated Driving|Unreal|UAV|Computer Vision|Image Processing|Robotics|Lidar|3D Animation')));
say('installed (relevant): %s\n',strjoin(vn,' | '));
blk = {};
[fl,triedL] = p3FindBlock('Simulation 3D Lidar'); blk{end+1} = localTern(~isempty(fl),'',['Simulation 3D Lidar block not found; libraries searched: ' strjoin(triedL,', ')]);
[fc,~] = p3FindBlock('Simulation 3D Camera'); blk{end+1} = localTern(~isempty(fc),'','Simulation 3D Camera block not found');
say('lidar block: %s\ncamera block: %s\n',fl,fc);
res(end+1,:) = {'Unreal lidar + camera blocks',all(cellfun(@isempty,blk)),strjoin(blk(~cellfun(@isempty,blk)),'; ')};
try
    mdl = 'p3sense_tmp'; if bdIsLoaded(mdl), close_system(mdl,0); end, new_system(mdl);
    add_block(fl,[mdl '/L']);
    dp = get_param([mdl '/L'],'DialogParameters'); fn = fieldnames(dp); say('--- Simulation 3D Lidar dialog parameters ---\n');
    for q = 1:numel(fn), say('%s | %s\n',fn{q},dp.(fn{q}).Prompt); end
    close_system(mdl,0);
catch ME, say('lidar dialog dump skipped: %s\n',ME.message); try, close_system('p3sense_tmp',0); catch, end, end
% ---- 2. camera carton detection on synthetic pictures
try, [okc,txt] = localCamTest(); catch ME, okc = false; txt = ME.message; end
res(end+1,:) = {'camera carton detection (40 synthetic pictures)',okc,txt};
% ---- 3. lidar localization (AMCL) + protective field through the real Phase3Sense block
try, [okl,txt,oks,txt2] = localSenseTest(say); catch ME, okl = false; txt = ME.message; oks = false; txt2 = 'not run'; say('%s\n',getReport(ME)); end
res(end+1,:) = {'AMCL localization (odometry noise + lidar vs map)',okl,txt};
res(end+1,:) = {'lidar tracking + protective field',oks,txt2};
% ---- report
ok = true;
for i = 1:size(res,1)
    say('%-55s %s   %s\n',res{i,1},localTern(res{i,2},'PASS','FAIL'),res{i,3}); ok = ok && res{i,2};
end
say('\nlog file: %s\n',lf); fclose(fid);
if ok, fprintf('\n[p3sense] ALL CHECKS PASSED\n'); else, fprintf('\n[p3sense] SOME CHECKS FAILED - send the log file above\n'); end
end

function localSay(fid,varargin)
fprintf(varargin{:}); fprintf(fid,varargin{:});
end
function s = localTern(c,a,b), if c, s = a; else, s = b; end, end

function [ok,txt] = localCamTest()
rng(3); col = [0.50 0.30 0.12]; dims = [0.45 0.50 0.36]; lift = 0.9; errs = zeros(0,2); nf = 0; n = 40; tt = 0;
for trial = 1:n
    z = 1.0 + 0.8*rand; ang = (rand-0.5)*0.5; expRel = [z*cos(ang) z*sin(ang) 1.20]; yaw = (rand-0.5)*0.4 + (rand<0.3)*pi/2;
    tru = expRel + [(rand-0.5)*0.2 (rand-0.5)*0.2 0];
    img = uint8(repmat(reshape([120 125 135],1,1,3),480,640)); img = img + uint8(randi(10,480,640,3)-5);
    cs = zeros(8,3); q = 0; R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
    for sx = [-1 1], for sy = [-1 1], for sz = [-1 1], q = q+1; p = R*[sx*dims(1)/2; sy*dims(2)/2]; cs(q,:) = [tru(1)+p(1) tru(2)+p(2) tru(3)+sz*dims(3)/2]; end, end, end
    uv = p3CamProject(cs,lift); k = convhull(uv(:,1),uv(:,2)); [X,Y] = meshgrid(1:640,1:480); in = inpolygon(X,Y,uv(k,1),uv(k,2));
    for c = 1:3, ch = img(:,:,c); ch(in) = uint8(255*col(c)*(0.8+0.4*rand)); img(:,:,c) = ch; end
    ub = max(uv(:,2)); in2 = X >= min(uv(:,1))-10 & X <= max(uv(:,1))+10 & Y > ub & Y < ub+60; cst = [0.95 0.48 0.05];
    for c = 1:3, ch = img(:,:,c); ch(in2) = uint8(255*cst(c)); img(:,:,c) = ch; end
    t0 = tic; [f,rel] = p3CamLocate(img,expRel,yaw,dims,lift,col); tt = tt + toc(t0);
    if f, nf = nf + 1; errs(end+1,:) = (rel(1:2)-tru(1:2))*1000; end %#ok<AGROW>
end
img = uint8(repmat(reshape([120 125 135],1,1,3),480,640)); fa = p3CamLocate(img,[1.3 0 1.2],0,dims,lift,col);
ok = nf >= 38 && max(abs(errs(:))) < 25 && ~fa;
txt = sprintf('found %d/%d, mean error x %.1f mm y %.1f mm, max %.1f mm, empty picture -> found=%d, %.2f s per picture',nf,n,mean(abs(errs(:,1))),mean(abs(errs(:,2))),max(abs(errs(:))),fa,tt/n);
end

function [okl,txt,oks,txt2] = localSenseTest(say)
C = WarehousePhase3Config(); C.Sense.LidarMode = 'raycast'; C.Sense.Localization = 'amcl'; C.Sense.Tracking = true;
names = {C.P3.robots.name}; F = struct('robots',struct('name',names,'pose',{[0 0 0],[0 0 0],[0 0 0],[0 0 0],[0 0 0],[0 0 0]},'carry',{false,false,false,false,false,false}));
for j = 1:6, F.robots(j).pose = [C.P3.robots(j).home 0]; end
assignin('base','Phase3Cfg',C); assignin('base','Phase3Fleet',F);
[G,res,off] = p3StaticGrid(C,C.Sense.MapRes); Gi = conv2(double(G),ones(2*round(0.4*res)+1),'same') > 0;
rid = 2; home = C.P3.robots(rid).home; s = Phase3Sense(); s.RobotId = rid;
% route: first free straight 8 m run from the bay, there and back (robot drives 1.0 m/s)
path = []; for a = 0:pi/4:7*pi/4
    q = home + (0:0.1:8).'*[cos(a) sin(a)]; if ~any(p3GridHit(Gi,res,off,q)), path = [q; flipud(q(1:end-1,:))]; yaw0 = a; break, end
end
if isempty(path), error('no free 8 m run found next to the bay (layout changed?)'); end
dt = C.SampleTime; step = 1.0*dt; len = cumsum([0; hypot(diff(path(:,1)),diff(path(:,2)))]); [len,iu] = unique(len); path = path(iu,:); nT = floor(len(end)/step);
pose = [home 0]; est = pose; errs = zeros(nT,1); ang = linspace(-pi,pi,C.Sense.LidarBeams+1); ang(end) = [];
for i = 1:nT
    p = interp1(len,path,i*step); h = yaw0;   % constant heading: the way back is driven in reverse (no instant 180 deg turn)
    pose = [p(1:2) h]; [est,bus] = s(pose,zeros(1,3)); errs(i) = hypot(est(1)-pose(1),est(2)-pose(2)); %#ok<ASGLU>
end
okl = mean(errs) < 0.10 && max(errs) < 0.30;
txt = sprintf('%.0f m driven, error mean %.3f m, max %.3f m (limits 0.10 / 0.30)',len(end),mean(errs),max(errs));
release(s);
% ---- protective field: another robot straight ahead at 0.7 m (must stop) and at 2.0 m (must not)
res2 = zeros(1,2); d = [0.7 2.0]; stopSeen = [false false];
for t = 1:2
    s2 = Phase3Sense(); s2.RobotId = rid; pose = [home yaw0]; j = 3;
    F.robots(j).pose = pose(1:2) + d(t)*[cos(yaw0) sin(yaw0)]; assignin('base','Phase3Fleet',F);
    for i = 1:round(2.5/dt), [~,bus] = s2(pose,zeros(1,3)); stopSeen(t) = stopSeen(t) || bus(2) > 0.5; end
    release(s2); F.robots(j).pose = [C.P3.robots(j).home 0]; assignin('base','Phase3Fleet',F);
end
oks = stopSeen(1) && ~stopSeen(2);
txt2 = sprintf('robot 0.7 m ahead -> stop=%d (must be 1), robot 2.0 m ahead -> stop=%d (must be 0)',stopSeen(1),stopSeen(2));
say('%s\n',txt);
end
