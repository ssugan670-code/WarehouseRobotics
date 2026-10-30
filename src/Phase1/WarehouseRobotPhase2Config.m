function C = WarehouseRobotPhase2Config()
%WAREHOUSEROBOTPHASE2CONFIG Single source of Phase 2 geometry and timing.
P3=phase3Paths(); makePhase3Layout(); layoutFile=P3.layout;   % Phase 3: own layout copy (6 bays + dock rack)
assert(isfile(layoutFile),'Phase2:MissingLayout','Missing %s.',layoutFile);
S=load(layoutFile,'WH'); C.WH=S.WH;

% One rate for scene, Robot, controller, Stateflow, and differential drive.
C.SampleTime=0.02;
C.CameraSampleTime=0.10;
C.BaseRadius=0.34;
% Include a path-tracking margin beyond the physical 0.30 m base radius.
% The planner therefore leaves room for a curved differential-drive turn at
% each grid corner instead of tracing exactly along an obstacle boundary.
C.Clearance=0.30;
C.WH.navMap=localSafeNavMap(C.WH,C.BaseRadius+C.Clearance);
% Wider map used while a carton is held: the swung box is much larger than the bare chassis, so the path stays further from racks.
C.CarryExtraClearance=0.45;
C.WH.navMapCarry=localSafeNavMap(C.WH,C.BaseRadius+C.Clearance+C.CarryExtraClearance);
% WH.robotXY is already expressed in the centred-X warehouse frame used by
% both the floor and the Differential Drive block. This point is [-3, 3],
% safely inside x=-28..28, y=0..72; no visual/world offset is applied.
C.HomeXY=C.WH.robotXY(1,:);

% Rigid camera mount in right-handed mobile-base coordinates: forward,left,up.
C.CameraMount.Translation=[0.20 -0.18 1.00];
% The active camera pitch is supplied by buildWarehouseRobotSim through the
% Camera Mount block and is passed to WarehousePerceptionSystem at runtime.
% Keep this neutral default only for standalone helper calls.
C.CameraMount.PitchDown=0;
C.CameraFocalLength=[450 450];
C.CameraPrincipalPoint=[320 240];
C.CameraImageSize=[480 640];
C.Perception.MinArea=40;
C.Perception.MaxArea=8000;
C.Perception.MinDepth=0.30;
C.Perception.MaxDepth=4.0;
C.Perception.FramesRequired=3;
C.Perception.Mode='sim';        % 'sim' or 'camera'
C.Perception.SimRange=3.5;      % maximum camera-to-target range [m]
C.Perception.SimNoise=0.01;     % 1-sigma scene-detector position noise [m]
C.Test.SpawnOffset=[0 0];       % target-only [dx dy] scene displacement [m]
% Diagnostic mode records measurement/IK/FK evidence only. It does not feed
% any truth value into the controller or alter a command/state transition.
C.Debug.Enabled=true;
C.Debug.SnapshotDir='C:\WarehouseRobotics\debug';
if ~isfolder(C.Debug.SnapshotDir), mkdir(C.Debug.SnapshotDir); end

% Shared shelf kit geometry. Changing level/slot moves the visual target and
% all geometry-derived navigation poses together; the controller still gets
% the grasp point only from live perception.
C.ShelfFill=0.55;
C.Target=struct('shelf',min(4,size(C.WH.shelves,1)),'level',1,'slot',[]);
[targetXYZ,G]=helperTargetPose(C.WH,C.Target);
C.TargetXYZ=targetXYZ;
C.TargetDimensions=G.targetDims;
C.TargetYaw=0;                 % TargetBox is spawned axis-aligned.
% Start the scan 2 m from the shelf face so the end panel cannot fill the
% camera view while the target remains inside the field of view.
C.ShelfStagingXY=[G.faceX+2.00 targetXYZ(2)];
% The dock centre stays one chassis radius plus 0.35 m from the shelf face.
% With the 0.34 m chassis radius this is faceX+0.69 m, leaving the requested
% 0.35 m physical front-clearance while keeping the arm within level-1 reach.
C.ChassisFrontClearance=0.35;
C.ShelfDockXY=[G.faceX+C.BaseRadius+C.ChassisFrontClearance targetXYZ(2)];
C.ShelfApproachXY=C.ShelfStagingXY;
C.ShelfFacingYaw=pi;

% ---- Round 30: three cartons on three different shelves and three different levels ----
% Level L carton centre is 0.53 / 1.43 / 2.33 m.  Gen3 reaches ~0.9 m, so the arm rides on a prismatic lift column:
% lift height = boxZ - 0.53 (0, 0.90, 1.80) keeps the validated level-1 arm geometry for every box.
C.LiftBaseBoxZ=0.53; C.LiftMax=1.90; C.LiftSpeed=0.25;
C.BoxColors=[0.90 0.05 0.75; 0.10 0.55 0.95; 0.95 0.60 0.05];
nSh=size(C.WH.shelves,1); levels=[1 2 3]; C.Boxes=struct('id',{},'shelf',{},'level',{},'xyz',{},'staging',{},'dock',{},'lift',{},'dims',{},'faceX',{});
cand=[C.Target.shelf, C.Target.shelf-1:-1:1, nSh:-1:C.Target.shelf+1]; cand=unique(cand,'stable');
for s=cand
    if numel(C.Boxes)>=3, break, end
    L=levels(numel(C.Boxes)+1);
    try
        [xyzS,GS]=helperTargetPose(C.WH,struct('shelf',s,'level',L,'slot',[]));
    catch ME
        fprintf('[BOXES] shelf %d level %d skipped (%s)\n',s,L,ME.message); continue
    end
    stg=[GS.faceX+2.00 xyzS(2)];
    if checkOccupancy(C.WH.navMap,[stg(1)+C.WH.X/2 stg(2)])~=0
        fprintf('[BOXES] shelf %d staging point is blocked in the nav map; skipped\n',s); continue
    end
    k=numel(C.Boxes)+1;
    C.Boxes(k)=struct('id',k,'shelf',s,'level',L,'xyz',xyzS,'staging',stg, ...
        'dock',[GS.faceX+C.BaseRadius+C.ChassisFrontClearance xyzS(2)],'lift',max(0,xyzS(3)-C.LiftBaseBoxZ),'dims',GS.targetDims,'faceX',GS.faceX);
end
assert(numel(C.Boxes)==3,'Phase2:Boxes','Could not place 3 cartons on 3 different shelves (found %d). Check WH.shelves.',numel(C.Boxes));
C.Test.MissingBox=[];
try, C.Test.MissingBox=reshape(evalin('base','Phase2TestMissingBox'),1,[]); catch, end   % runRound30(...,'missing',[2]) sets this   % e.g. [2] spawns carton 2 away from its slot so the robot must report it absent and skip it
C.Test.MissingXY=[0 0]; % set below once the table is known
for k=1:3, fprintf('[BOXES] box %d: shelf %d level %d centre=[%.2f %.2f %.2f] lift=%.2f m\n',k,C.Boxes(k).shelf,C.Boxes(k).level,C.Boxes(k).xyz,C.Boxes(k).lift); end
C.ReverseDistance=0.65;   % minimum back-off from the rack before any turn (controller extends it from the real swing radius)
C.TurnMargin=0.12;        % extra gap between the swung carton and the rack face
C.NavSwitchRadius=0.45; C.NavSwitchRadiusCarry=0.60; C.NavLookahead=0.70;
C.NavRotateEnter=0.60; C.NavRotateExit=0.10;

C.TableTopZ=0.82;
C.TableSurfaceZ=C.TableTopZ+0.05;   % slab is centred at TableTopZ and is 0.10 m thick -> real surface
C.TableApproachRun=1.6;
% ---- Round 32: delivery stations.  Every table of the layout is a "Packing Station"; the operator chooses the station per carton.
% Each station has 3 slots (centre, -lateral, +lateral).  The robot approaches from the first free side (south, north, west, east).
C.StationNames=arrayfun(@(k) sprintf('Packing Station %d',k),1:size(C.WH.tables,1),'UniformOutput',false);   % rename here if you like
sides=struct('name',{'south','north','west','east'},'n',{[0 1],[0 -1],[1 0],[-1 0]});
C.Stations=struct('id',{},'name',{},'side',{},'yaw',{},'slotXYZ',{},'approach',{},'centre',{});
for t=1:size(C.WH.tables,1)
    T=C.WH.tables(t,:); % [cx cy widthX depthY]
    chosen=[];
    for si=1:numel(sides)
        n=sides(si).n; lat=[-n(2) n(1)];
        half=abs(n(1))*T(3)/2+abs(n(2))*T(4)/2; latExt=abs(lat(1))*T(3)+abs(lat(2))*T(4);
        sp=min(0.62,max(0.56,(latExt-0.6)/2)); offs=[0 -1 1]*sp;
        face=T(1:2)-n*half; slotZ=C.TableSurfaceZ+0.5*C.TargetDimensions(3)+0.02;
        slots=zeros(3,3); appr=zeros(3,2); free=true;
        for j=1:3
            slots(j,:)=[face+n*0.22+lat*offs(j) slotZ];
            appr(j,:)=face-n*0.65+lat*offs(j);
            gate=appr(j,:)-n*C.TableApproachRun;
            chk=face-n*1.0+lat*offs(j);   % the approach point itself sits inside the inflated margin; test a point just outside it and the gate point
            for pt=[chk; gate]'
                if checkOccupancy(C.WH.navMap,[pt(1)+C.WH.X/2 pt(2)])~=0, free=false; end
            end
        end
        if (free && (si<=2 || latExt>=1.6)) || (t==1 && si==1), chosen=struct('id',t,'name',C.StationNames{t},'side',sides(si).name,'yaw',atan2(n(2),n(1)),'slotXYZ',slots,'approach',appr,'centre',T(1:2)); break, end
    end
    if isempty(chosen)   % nothing free in the carry map: fall back to the south side (same as Round 31)
        n=[0 1]; lat=[-1 0]; half=T(4)/2; sp=min(0.62,max(0.56,(T(3)-0.6)/2)); offs=[0 -1 1]*sp; face=T(1:2)-n*half; slotZ=C.TableSurfaceZ+0.5*C.TargetDimensions(3)+0.02;
        slots=zeros(3,3); appr=zeros(3,2); for j=1:3, slots(j,:)=[face+n*0.22+lat*offs(j) slotZ]; appr(j,:)=face-n*0.65+lat*offs(j); end
        chosen=struct('id',t,'name',C.StationNames{t},'side','south','yaw',pi/2,'slotXYZ',slots,'approach',appr,'centre',T(1:2));
        fprintf('[STATIONS] %s: no free approach found, using the south side\n',C.StationNames{t});
    end
    C.Stations(t)=chosen;
    fprintf('[STATIONS] %s (table %d): approach from the %s side, slot 1 at [%.2f %.2f]\n',chosen.name,t,chosen.side,chosen.slotXYZ(1,1:2));
end
% legacy single-table fields = station 1 (used by the Stateflow chart labels and old logs)
st1=C.Stations(1); C.TableFacingYaw=st1.yaw; C.TableApproachXY=st1.approach(1,:); C.TablePlaceXYZ=st1.slotXYZ(1,:); C.TableFaceY=st1.centre(2)-C.WH.tables(1,4)/2;
for k=1:3
    C.Boxes(k).slotXYZ=st1.slotXYZ(k,:); C.Boxes(k).tableApproach=st1.approach(k,:);
end
C.Test.MissingXY=[st1.centre(1)+2.5 C.TableFaceY-2.5];
C.UnrealPhysics=true;   % Unreal rigid-body physics on the carton; false = controller contact solver only
C.MissionLoop=false;    % false = single pick-and-place then MISSION_DONE
C.BoxYawSign=-1;   % Unreal-frame yaw = -(right-handed yaw); flip to +1 if the carried box turns the wrong way

C.TargetActor="TargetBox";
% Native Simulation 3-D grasp is used as a vacuum-style, proximity attach.
% The 8 mm no-penetration seal gap is inside this translation tolerance.
C.GraspRegionTranslation=[0.50 0.50 0.50];   % [m] generous: frame-agnostic (world or tool frame); the controller already guarantees proximity
% Expected box pose in the ToolTip frame at attach (box centre is ~0.245 m ahead of the cup along the tool axis).
% Rotation is [yaw pitch roll] rad of the box relative to the tool frame for a ~20 deg downward tool tilt and base yaw ~pi.
C.GraspOffsetTrans=[-0.04 0 0.245];
C.GraspOffsetRot=[0 1.2217 3.1416];
C.GraspRegionRotation=[7 7 7];   % rad. Open gate (> 2*pi). NaN made every comparison false, so the native grasp could never trigger
C.ToolTipOffset=0.10;         % EndEffector_Link +Z to the added ToolTip [m]
C.ToolRadius=0.015;
C.PregraspDistance=0.20;
C.SealGap=0.016;              % NORMAL gap tool tip -> box face [m] (suction cup rim stays clear)
C.ContactHeight=0.05;         % contact point height above box centre [m]
C.LiftDistance=0.06;
C.ArmMoveTime=3.0;
C.ReadyMoveTime=2.8;
C.ArmMaxStep=0.15;            % per 1.5 cm Cartesian sample; a branch flip is > 1 rad
C.ArmJointMargin=deg2rad(5);
end

function map=localSafeNavMap(WH,inflation)
resolution=10; map=binaryOccupancyMap(WH.X,WH.Y,resolution);
nx=ceil(WH.X*resolution); ny=ceil(WH.Y*resolution);
setOccupancy(map,[linspace(0,WH.X,nx)' zeros(nx,1)],1);
setOccupancy(map,[linspace(0,WH.X,nx)' WH.Y*ones(nx,1)],1);
setOccupancy(map,[zeros(ny,1) linspace(0,WH.Y,ny)'],1);
setOccupancy(map,[WH.X*ones(ny,1) linspace(0,WH.Y,ny)'],1);
for k=1:size(WH.shelves,1)
    localRect(map,WH.shelves(k,1)+WH.X/2,WH.shelves(k,2),WH.shelves(k,4),WH.shelves(k,3),resolution);
end
for k=1:size(WH.tables,1)
    localRect(map,WH.tables(k,1)+WH.X/2,WH.tables(k,2),WH.tables(k,3),WH.tables(k,4),resolution);
end
if WH.offCx>-100, localRect(map,WH.offCx+WH.X/2,WH.offCy,WH.offW,WH.offD,resolution); end
if isfield(WH,'stands')      % Phase 3: wooden stands of the staging area are obstacles (the robot docks in front of them)
    for k=1:size(WH.stands,1), localRect(map,WH.stands(k,1)+WH.X/2,WH.stands(k,2),WH.standSize,WH.standSize,resolution); end
end
inflate(map,inflation);
end

function localRect(map,cx,cy,w,d,res)
[x,y]=meshgrid((cx-w/2):1/res:(cx+w/2),(cy-d/2):1/res:(cy+d/2));
inside=x(:)>=0 & x(:)<=map.XWorldLimits(2) & y(:)>=0 & y(:)<=map.YWorldLimits(2);
setOccupancy(map,[x(inside) y(inside)],1);
end
