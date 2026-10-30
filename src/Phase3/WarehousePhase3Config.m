function C = WarehousePhase3Config(varargin)
%WAREHOUSEPHASE3CONFIG  Phase 2 configuration + everything the multi-robot warehouse needs (C.P3).
%   C = WarehousePhase3Config('cartons',88,'stored',60,'dock',14,'decor',0,'seed',7,'missing',[])   (stored + dock + 14 spare = cartons; every box is a real, handled carton; decor>0 adds dummy boxes)
%   C.P3.robots  : 6 robots (blue red green black pink yellow), home bay, colour
%   C.P3.slots   : every accessible shelf slot  S<shelf>-L<level>-P<pos>   (shelf 8 = receiving dock rack)
%   C.P3.cartons : handled carton pool (initial slot / park position)
%   C.P3.static  : slots filled with static stock (not handled by robots, never offered as free)
p = inputParser;
addParameter(p,'cartons',54); addParameter(p,'stored',30); addParameter(p,'dock',10);
addParameter(p,'decor',0); addParameter(p,'seed',7); addParameter(p,'missing',[]); addParameter(p,'labels','num');
parse(p,varargin{:}); o = p.Results;

C = WarehouseRobotPhase2Config();
C = localWidenStations(C);       % a station takes as many boxes as its table is long (not only 3)
C.NavSpeed = 1.55; C.NavSpeedCarry = 1.15; C.SafetyRadius = 0.45; C.CarryReach = 0.60; C.ArmReach = 0.50; C.StowReach = 0.35;      % extra reach beyond the chassis circle: carried box / arm that is out (traffic keeps these apart too)      % a little faster than Phase 2 (fleet only)
% ---- Round 17: arm / lift speeds ~20-25 % higher than Phase 2 (fleet only; Phase 2 file untouched).  Contact moves (SpeedTouch, SpeedDescend) are NOT changed.
%      The trajectory is still sampled every 0.02 s, so a faster move only has fewer samples: Simulink / FPS load per second is the same.
C.SpeedApproach = 0.15; C.SpeedLift = 0.05; C.SpeedCarry = 0.125; C.SpeedPlace = 0.125; C.SpeedLevel = 0.10; C.LiftSpeed = 0.30; C.ReverseSpeed = 0.18; C.ArmMoveTime = 2.4;
WH = C.WH; R = C.BaseRadius; dims = C.TargetDimensions(:).';

% ---- SENSING (Round 12): lidar localization (Navigation Toolbox AMCL), camera carton detection (Computer Vision), lidar tracking + protective field
%      (Sensor Fusion and Tracking Toolbox).  Fixed design, no run-time adaptation.  Set Localization='truth', Camera='sim', Tracking=false to get the previous behaviour.
S = struct();
S.Localization = 'amcl';        % 'amcl' = wheel odometry (true increments + encoder noise) corrected by AMCL against the lidar | 'truth' = vehicle pose as before
S.Camera       = 'camera';      % 'camera' = the robot camera finds the carton (p3CamLocate) | 'sim' = scene readback as before
S.Tracking     = true;          % lidar clusters + fleet broadcast -> multiObjectTracker, protective field
S.LidarMode    = 'unreal';      % 'unreal' = Simulation 3D Lidar block (falls back to 'raycast' by itself if its returns do not match the map) | 'raycast' | 'off'
S.LidarZ = 0.55; S.LidarOffset = [-0.28 0]; S.LidarBeams = 180; S.LidarMinRange = 0.45; S.LidarMaxRange = 12; S.LidarPeriod = 0.10; S.LidarNoise = 0.02;
S.MapRes = 20; S.MapInflate = 0.20;                                    % static grid [cells/m]; inflation used for "explained by the map" [m]
S.OdomSigmaS = 0.02; S.OdomSigmaR = 0.02; S.OdomDrift = 0.01;          % encoder noise: 2 % distance, 2 % rotation, 0.01 rad per m drift
S.AmclParticles = [300 800]; S.AmclBeams = 60; S.AmclMeasNoise = 0.15; S.AmclRandomWeight = 0.15; S.AmclMotionNoise = [0.1 0.1 0.05 0.05];
S.AmclInitCov = [0.02 0.02 0.01]; S.AmclUpdate = [0.03 0.03 0.03];
S.TrackMinPts = 3; S.SafetyConeDeg = 35; S.SafetyRobot = 0.85; S.SafetyUnknown = 1.45; S.SafetySelf = 1.05;
S.CamPeriod = 0.20; S.CamFrames = 5; S.CamPlan = 'readback'; S.CamMinFound = 2; S.CamMinIoU = 0.45; S.CamFallback = true;
S.ErrCap = 0.6;              % Round 32: localisation error cap [m]: AMCL re-initialised after 1 s above it and once per dock (0 = off)
S.PhysFinal = true;           % Round 31: last 3 m to a put-down place on the physical (close-range) pose, like the pick dock; false = belief pose as in round 30      % camera frames per scan; fallback = readback if the camera misses (logged as [VISION FALLBACK])
C.Sense = S;

names = {'blue','red','green','black','pink','yellow'};
cols  = [0.10 0.25 0.95; 0.95 0.10 0.10; 0.10 0.75 0.20; 0.05 0.05 0.05; 1.00 0.62 0.75; 0.98 0.88 0.05];
assert(size(WH.robotXY,1)>=6,'Phase3:Bays','Layout has %d robot bays, 6 needed (run makePhase3Layout).',size(WH.robotXY,1));
robots = struct('id',{},'name',{},'colour',{},'home',{});
for k = 1:6, robots(k) = struct('id',k,'name',names{k},'colour',cols(k,:),'home',WH.robotXY(k,:)); end

% ---- slots.  Shelf slots: <shelf><side>-L<level>-P<pos>, e.g. 3B-L2-P05.  side A = west face, side B = east face (both are service faces).
%      Staging stands (receiving area, level "L0"): D01 .. D14.   Every slot knows its outward normal n, face coordinate faceD (n . facePoint),
%      the robot facing yaw and the carton yaw, so the controller can work on any face.
slots = p3BuildSlots(WH,dims,R,C.ChassisFrontClearance,C.LiftBaseBoxZ,@(stg) checkOccupancy(C.WH.navMap,[stg(1)+WH.X/2 stg(2)])==0);
isStore = strcmp({slots.role},'store'); isDock = strcmp({slots.role},'dock');

% ---- static stock: deterministic, same every run (both sides of every shelf)
rs = RandStream('mt19937ar','Seed',o.seed);
static = false(1,numel(slots));
for i = find(isStore), static(i) = rand(rs) < o.decor; end

% ---- carton pool
M = o.cartons; nSt = o.stored; nDk = min(o.dock,sum(isDock));
freeStore = find(isStore & ~static); perm = freeStore(randperm(rs,numel(freeStore)));
if isinf(nSt), nSt = max(0,numel(perm)-8); M = max(M,nSt+nDk+14); end      % fill max: every free shelf slot gets a real box, 8 stay free for put-away
nSt = min(nSt,numel(perm)); stored = perm(1:nSt);
dockIdx = find(isDock); [~,ord] = sortrows([[slots(dockIdx).level].' [slots(dockIdx).pos].']); dockIdx = dockIdx(ord);
assert(nSt+nDk<=M,'Phase3:Pool','cartons (%d) must be >= stored (%d) + dock (%d).',M,nSt,nDk);
cartons = struct('id',{},'slot',{},'state',{},'xyz',{},'park',{},'yaw',{});
for j = 1:M
    col = mod(j-1,4); row = floor((j-1)/4);
    park = [-3+0.9*col 30+0.9*row -3];             % reserve / shipped cartons wait under the floor (out of sight); they pop up on a stand when they arrive
    slot = 0; st = 'RESERVE'; xyz = park; yw = 0;
    if j<=nSt, slot = stored(j); st = 'STORED'; xyz = slots(slot).xyz; yw = slots(slot).boxYaw;
    elseif j<=nSt+nDk, slot = dockIdx(j-nSt); st = 'DOCK'; xyz = slots(slot).xyz; yw = slots(slot).boxYaw; end
    if any(j==o.missing), xyz = park; end          % test hook: DB says stored, the carton is physically elsewhere
    cartons(j) = struct('id',j,'slot',slot,'state',st,'xyz',xyz,'park',park,'yaw',yw);
end

C.P3 = struct('robots',robots,'slots',slots,'static',find(static),'cartons',cartons,'nCartons',M, ...
    'dockShelf',0,'stands',WH.stands,'cartonColour',[0.50 0.30 0.12],'missing',o.missing,'labelMode',localLabelMode(o.labels),'decor',o.decor,'seed',o.seed);
fprintf('[P3CONFIG] %d slots (%d store, %d dock), %d static-stock slots, %d cartons (%d stored, %d at dock, %d reserve)\n', ...
    numel(slots),sum(isStore),sum(isDock),sum(static),M,nSt,nDk,M-nSt-nDk);
end

function C = localWidenStations(C)
% places along the table edge: centre first, then alternately left / right, pitch 0.8 m, same geometry as the Phase 2 slots (0.22 m onto the table, approach 0.65 m off its edge)
pitch = 0.8; WH = C.WH;
for t = 1:numel(C.Stations)
    S = C.Stations(t); T = WH.tables(t,:);
    n = [cos(S.yaw) sin(S.yaw)]; lat = [-n(2) n(1)];
    half = abs(n(1))*T(3)/2 + abs(n(2))*T(4)/2; latExt = abs(lat(1))*T(3) + abs(lat(2))*T(4);
    ns = min(11,max(3,floor((latExt-1.0)/pitch)+1));
    offs = zeros(1,ns); for q = 2:ns, offs(q) = (-1)^(q-1)*ceil((q-1)/2)*pitch; end
    face = T(1:2) - n*half; slotZ = S.slotXYZ(1,3);
    slots = zeros(ns,3); appr = zeros(ns,2); keep = true(1,ns);
    for j = 1:ns
        slots(j,:) = [face + n*0.22 + lat*offs(j), slotZ];
        appr(j,:) = face - n*0.65 + lat*offs(j);
        chk = face - n*1.0 + lat*offs(j);
        if j > 1 && checkOccupancy(WH.navMap,[chk(1)+WH.X/2 chk(2)]) ~= 0, keep(j) = false; end
    end
    C.Stations(t).slotXYZ = slots(keep,:); C.Stations(t).approach = appr(keep,:);
    fprintf('[STATIONS] %s: %d places along the table\n',S.name,sum(keep));
end
end

function a = localWrap(a)
a = mod(a+pi,2*pi)-pi;
end

function m = localLabelMode(v)
if islogical(v) || isnumeric(v), m = p3tern(v,'num','off'); else, m = lower(char(v)); end
if ~any(strcmp(m,{'num','full','off'})), m = 'num'; end
end
