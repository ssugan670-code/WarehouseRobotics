function modelName = buildFleetSim(N)
%BUILDFLEETSIM  Build WarehouseFleetSim.slx : N robots (1..6), carton pool with router, one selectable camera, colour markers.
%   Every robot is the Phase 2 chain (Robot + WarehouseRobotControlSystemV23 + RobotTaskFSM + differential drive) inside a subsystem 'Robot k';
%   the controller block has RobotId = k, which switches it to fleet mode (jobs / speed limit / hold from Phase3Fleet).
%   Needs in the base workspace:  Phase3Cfg (WarehousePhase3Config)  and  Phase3Fleet (Phase3Fleet object) - phase3.m creates both.
modelName = 'WarehouseFleetSim';
C = evalin('base','Phase3Cfg'); F = evalin('base','Phase3Fleet'); M = C.P3.nCartons;
if ~isfield(C,'MarkerRadius'), C.MarkerRadius = 0.338; end
if ~isfield(C,'MarkerZ'), C.MarkerZ = 0.20; end
assert(N>=1 && N<=6,'Phase3:N','1..6 robots');
if evalin('base','exist(''world'',''var'')'), try, close(evalin('base','world')); catch, end; try, delete(evalin('base','world')); catch, end; evalin('base','clear world'); end
if bdIsLoaded(modelName), close_system(modelName,0); end
modelFile = fullfile(pwd,[modelName '.slx']); if exist(modelFile,'file'), delete(modelFile); end
load_system('robotsim3dlib'); load_system('sim3dlib'); load_system('sim3dcameralib'); load_system('robotmobilelib');
new_system(modelName);
set_param(modelName,'Solver','ode4','FixedStep',num2str(C.SampleTime),'StopTime','inf');
localTrySet(modelName,'EnablePacing','on'); localTrySet(modelName,'PacingRate','1');
ws = get_param(modelName,'ModelWorkspace');
[gen3,qStow] = helperBuildMobileManipulator();
assignin(ws,'gen3',gen3); assignin(ws,'qStow',qStow); assignin(ws,'Zero',0);
for k = 1:N, assignin(ws,sprintf('HomePose%d',k),[C.P3.robots(k).home 0]); end

scene = localAdd(modelName,'sim3dlib','Simulation 3D Scene Configuration','Scene Configuration',[1120 560 1340 620]);
set_param(scene,'ProjectFormat','Default Scenes','SceneDesc','Empty scene','Priority','0');
env = localAdd(modelName,'sim3dlib','Simulation 3D Actor','Phase3 Warehouse',[900 560 1060 650]);
set_param(env,'ActorName','WarehouseRoot','ParentName','Scene Origin','Operation','Create at setup','InitScriptText','setupFleetScene(Actor,World);','Priority','1');

% ---------------------------------------------------------------- carton pool: Transform Get -> delay -> pool vector
poolMux = [modelName '/Pool Pack']; add_block('simulink/User-Defined Functions/MATLAB Function',poolMux,'Position',[1500 100 1620 100+20*M]);
localSetMATLABFunction(poolMux,['function y = fcn(' strjoin(arrayfun(@(j) sprintf('a%d',j),1:M,'UniformOutput',false),',') ')' char(10) ...
    'y = [' strjoin(arrayfun(@(j) sprintf('reshape(a%d,1,[])',j),1:M,'UniformOutput',false),' ') '];' char(10)]);
for j = 1:M
    tag = sprintf('Carton%02d',j); xyz = C.P3.cartons(j).xyz; ic = [xyz(1) -xyz(2) xyz(3)];
    y0 = 40+70*(j-1);
    g = localAdd(modelName,'sim3dlib','Simulation 3D Actor Transform Get',[tag ' Transform'],[1180 y0 1340 y0+50]);
    localTrySet(g,'ActorTag',tag); localTrySet(g,'NumberOfParts','1'); localTrySet(g,'Priority','1');
    d = localAdd(modelName,'simulink','Unit Delay',[tag ' Delay'],[1360 y0+10 1420 y0+40]);
    set_param(d,'InitialCondition',mat2str(ic),'SampleTime',num2str(C.SampleTime));
    localLine(modelName,[tag ' Transform/1'],[tag ' Delay/1']); localLine(modelName,[tag ' Delay/1'],sprintf('Pool Pack/%d',j));
end

% ---------------------------------------------------------------- robots
for k = 1:N, localRobot(modelName,k,C,M); end
for k = 1:N, localLine(modelName,'Pool Pack/1',sprintf('Robot %d/1',k)); end   % every robot sees the positions of all cartons

% ---------------------------------------------------------------- Round 12: sensing rig of every robot (lidar, camera, localization, tracking, carton detection)
for k = 1:N, C = localSenseRig(modelName,k,C,N); end
assignin('base','Phase3Cfg',C);          % lidar / camera blocks that could not be built are recorded in C.Sense (the Phase3Sense / controller blocks read it at start-up)

% ---------------------------------------------------------------- fleet block + router + drivers
fb = localAdd(modelName,'simulink','MATLAB System','Fleet Carton Cmd',[1700 40 1860 140]);
set_param(fb,'System','Phase3FleetBlock'); set_param(fb,'NumCartons',num2str(M)); set_param(fb,'SimulateUsing','Interpreted execution');
rt = [modelName '/Carton Router']; add_block('simulink/User-Defined Functions/MATLAB Function',rt,'Position',[1700 200 1880 200+40*max(N,3)]);
ins = strjoin(arrayfun(@(k) sprintf('p%d',k),1:N,'UniformOutput',false),',');
packs = strjoin(arrayfun(@(k) sprintf('p%d',k),1:N,'UniformOutput',false),';');
code = sprintf(['function [trans,rot,drv] = fcn(%s,fc,fr,fa)\n' ...
    '%% every robot sends [carton(1x3) rot(1x3) drive idx]; the fleet block sends park / arrival commands. A robot that drives carton j beats the fleet.\n' ...
    'M=%d; trans=fc; rot=fr; drv=(fa>0.5);\n' ...
    'P=[%s];\n' ...
    'for r=1:%d\n' ...
    '  j=round(P(r,8));\n' ...
    '  if j>=1 && j<=M && P(r,7)>0.5\n' ...
    '    trans(3*j-2:3*j)=P(r,1:3); rot(3*j-2:3*j)=P(r,4:6); drv(j)=true;\n' ...
    '  end\n' ...
    'end\n'],ins,M,packs,N);
localSetMATLABFunction(rt,code);
for k = 1:N, localLine(modelName,sprintf('Robot %d/2',k),sprintf('Carton Router/%d',k)); end
localLine(modelName,'Fleet Carton Cmd/1',sprintf('Carton Router/%d',N+1)); localLine(modelName,'Fleet Carton Cmd/2',sprintf('Carton Router/%d',N+2)); localLine(modelName,'Fleet Carton Cmd/3',sprintf('Carton Router/%d',N+3));
tdm = localAdd(modelName,'simulink','Demux','Trans Demux',[1950 200 1960 300]); set_param(tdm,'Outputs',mat2str(repmat(3,1,M)));
rdm = localAdd(modelName,'simulink','Demux','Rot Demux',[1950 320 1960 420]); set_param(rdm,'Outputs',mat2str(repmat(3,1,M)));
ddm = localAdd(modelName,'simulink','Demux','Drv Demux',[1950 440 1960 540]); set_param(ddm,'Outputs',num2str(M));
localLine(modelName,'Carton Router/1','Trans Demux/1'); localLine(modelName,'Carton Router/2','Rot Demux/1'); localLine(modelName,'Carton Router/3','Drv Demux/1');
scl = localAdd(modelName,'simulink','Constant','Box Scale Hold',[1700 600 1780 630]); set_param(scl,'Value','[1 1 1]','OutDataTypeStr','double','VectorParams1D','off');
for j = 1:M
    tag = sprintf('Carton%02d',j); sname = sprintf('Carton Driver %d',j); ss = [modelName '/' sname];
    add_block('simulink/Ports & Subsystems/Enabled Subsystem',ss,'Position',[2050 40+70*(j-1) 2190 100+70*(j-1)]);
    ln = find_system(ss,'SearchDepth',1,'FindAll','on','Type','line'); for q = 1:numel(ln), delete_line(ln(q)); end
    delete_block([ss '/Out1']); set_param([ss '/In1'],'Name','Translation');
    add_block('simulink/Sources/In1',[ss '/Rotation']); add_block('simulink/Sources/In1',[ss '/Scale']);
    st = localAdd(ss,'sim3dlib','Simulation 3D Actor Transform Set',[tag ' Set'],[200 40 380 140]);
    set_param(st,'ActorTag',tag); localTrySet(st,'NumberOfParts','1'); localTrySet(st,'Priority','1');
    add_line(ss,'Translation/1',[tag ' Set/1']); add_line(ss,'Rotation/1',[tag ' Set/2']); add_line(ss,'Scale/1',[tag ' Set/3']);
    localLine(modelName,sprintf('Trans Demux/%d',j),[sname '/1']); localLine(modelName,sprintf('Rot Demux/%d',j),[sname '/2']);
    localLine(modelName,'Box Scale Hold/1',[sname '/3']); localLine(modelName,sprintf('Drv Demux/%d',j),[sname '/Enable']);
end

% ---------------------------------------------------------------- colour markers on the robots' lower back (follower actors)
for k = 1:N
    % "paint" on the lower back: three thin slats following the curve of the chassis (angles -15 0 +15 deg), flush with the surface
    mc = localAdd(modelName,'simulink','MATLAB Function',sprintf('Marker %d Calc',k),[900 700+60*(k-1) 1030 740+60*(k-1)]);
    localSetMATLABFunction(mc,sprintf(['function [t1,r1,t2,r2,t3,r3] = fcn(pose)\n' ...
        '%% slat q sits at body angle (pi + th) on the chassis radius; its thin axis is radial.  Unreal frame: y negated, yaw negated.\n' ...
        'R=%.4f; z=%.3f; th=[-0.2618 0 0.2618];\n' ...
        'p=zeros(3,3); r=zeros(3,3);\n' ...
        'for q=1:3\n' ...
        '  f=pose(3)+pi+th(q); p(q,:)=[pose(1)+R*cos(f) -(pose(2)+R*sin(f)) z]; r(q,:)=[0 0 -f];\n' ...
        'end\n' ...
        't1=p(1,:); r1=r(1,:); t2=p(2,:); r2=r(2,:); t3=p(3,:); r3=r(3,:);\n'],C.MarkerRadius,C.MarkerZ));
    localLine(modelName,sprintf('Robot %d/1',k),sprintf('Marker %d Calc/1',k));
    for q = 1:3
        ms = localAdd(modelName,'sim3dlib','Simulation 3D Actor Transform Set',sprintf('Marker %d%c Set',k,'a'+q-1),[1060 700+60*(k-1)+14*(q-1) 1200 735+60*(k-1)+14*(q-1)]);
        set_param(ms,'ActorTag',sprintf('RobotMarker%d_%d',k,q)); localTrySet(ms,'NumberOfParts','1'); localTrySet(ms,'Priority','1');
        localLine(modelName,sprintf('Marker %d Calc/%d',k,2*q-1),sprintf('Marker %d%c Set/1',k,'a'+q-1));
        localLine(modelName,sprintf('Marker %d Calc/%d',k,2*q),sprintf('Marker %d%c Set/2',k,'a'+q-1));
        localLine(modelName,'Box Scale Hold/1',sprintf('Marker %d%c Set/3',k,'a'+q-1));
    end
end

% ---------------------------------------------------------------- one camera for all robots (selected robot / scanning robot) + viewer
posesMux = [modelName '/Poses Mux']; add_block('simulink/User-Defined Functions/MATLAB Function',posesMux,'Position',[700 900 780 900+20*N]);
localSetMATLABFunction(posesMux,['function y = fcn(' strjoin(arrayfun(@(k) sprintf('a%d',k),1:N,'UniformOutput',false),',') ')' char(10) ...
    'y = [' strjoin(arrayfun(@(k) sprintf('reshape(a%d,1,3)',k),1:N,'UniformOutput',false),' ') '];' char(10)]);
liftMux = [modelName '/Lifts Mux']; add_block('simulink/User-Defined Functions/MATLAB Function',liftMux,'Position',[700 1000 780 1000+20*N]);
localSetMATLABFunction(liftMux,['function y = fcn(' strjoin(arrayfun(@(k) sprintf('a%d',k),1:N,'UniformOutput',false),',') ')' char(10) ...
    'y = [' strjoin(arrayfun(@(k) sprintf('a%d(1)',k),1:N,'UniformOutput',false),' ') '];' char(10)]);
for k = 1:N, localLine(modelName,sprintf('Robot %d/1',k),sprintf('Poses Mux/%d',k)); localLine(modelName,sprintf('Robot %d/3',k),sprintf('Lifts Mux/%d',k)); end
m = helperVisionCameraMount(); pitch = m(4);
cp = localAdd(modelName,'simulink','MATLAB Function','Fleet Camera Pose',[800 900 980 1000]);
localSetMATLABFunction(cp,sprintf([ ...
    'function [translation,rotation,selPose,selLift,sel] = fcn(poses,lifts)\n' ...
    'coder.extrinsic(''phase3SelRobot'');\n' ...
    'sel=1; sel=phase3SelRobot();\n' ...
    'sel=min(max(round(sel),1),%d);\n' ...
    'selPose=poses(3*sel-2:3*sel); selLift=lifts(sel);\n' ...
    'yaw=selPose(3); c=cos(yaw); s=sin(yaw);\n' ...
    'f=%.6g; l=%.6g; z=%.6g+selLift;\n' ...
    'translation=[selPose(1)+f*c-l*s selPose(2)+f*s+l*c z];\n' ...
    'rotation=[0 %.6g yaw*180/pi];\n'],N,0.10+m(1),m(2),0.47+m(3),pitch));
localLine(modelName,'Poses Mux/1','Fleet Camera Pose/1'); localLine(modelName,'Lifts Mux/1','Fleet Camera Pose/2');
src = find_system('sim3dcameralib','SearchDepth',1,'FollowLinks','on','Name','Simulation 3D Camera');
camB = [modelName '/Fleet Camera']; add_block(src{1},camB,'Position',[1060 900 1220 1050]);
set_param(camB,'Priority','1','vehTag','Scene Origin','offsetFlag','on','extTmount','on','extRmount','on', ...
    'ImageSize',mat2str(C.CameraImageSize),'FocalLength',mat2str([m(5) m(5)]),'OpticalCenter',mat2str(C.CameraPrincipalPoint));
localTrySet(camB,'SampleTime',num2str(C.CameraSampleTime));
localLine(modelName,'Fleet Camera Pose/1','Fleet Camera/1'); localLine(modelName,'Fleet Camera Pose/2','Fleet Camera/2');
vw = [modelName '/Fleet Vision Viewer']; add_block('simulink/User-Defined Functions/MATLAB Function',vw,'Position',[1300 900 1470 1000]);
localSetMATLABFunction(vw,sprintf(['function ok = fcn(img,pose,lift,sel)\n' ...
    'coder.extrinsic(''fleetVisionStep'');\n' 'fleetVisionStep(img,pose,lift,sel);\n' 'ok=uint8(0);\n']));
add_block('simulink/Sinks/Terminator',[modelName '/Fleet Vision End'],'Position',[1520 925 1540 945]);
localLine(modelName,'Fleet Camera/1','Fleet Vision Viewer/1'); localLine(modelName,'Fleet Camera Pose/3','Fleet Vision Viewer/2');
localLine(modelName,'Fleet Camera Pose/4','Fleet Vision Viewer/3'); localLine(modelName,'Fleet Camera Pose/5','Fleet Vision Viewer/4'); localLine(modelName,'Fleet Vision Viewer/1','Fleet Vision End/1');

% ---------------------------------------------------------------- stop when the fleet says so (p3 end / p3 stop)
add_block('simulink/Sinks/Stop Simulation',[modelName '/Fleet Stop'],'Position',[1930 40 1980 70]);
localLine(modelName,'Fleet Carton Cmd/4','Fleet Stop/1');

% ---- self-check: report any block input/output that is not connected (they would silently read 0)
try
    bl = find_system(modelName,'LookUnderMasks','none','FollowLinks','off','Type','Block');
    for q = 1:numel(bl)
        ph = get_param(bl{q},'PortHandles'); bad = '';
        for z = 1:numel(ph.Inport),  if get_param(ph.Inport(z),'Line') < 0,  bad = [bad sprintf(' in%d',z)]; end, end %#ok<AGROW>
        for z = 1:numel(ph.Outport)      % unused outputs (e.g. Transform Get rotation / scale, Differential Drive state) get a Terminator
            if get_param(ph.Outport(z),'Line') < 0
                par = get_param(bl{q},'Parent'); tn = [par '/Term_' regexprep(get_param(bl{q},'Name'),'\W','_') sprintf('_%d',z)];
                try
                    pos = get_param(ph.Outport(z),'Position'); add_block('simulink/Sinks/Terminator',tn,'Position',[pos(1)+30 pos(2)-8 pos(1)+50 pos(2)+8]);
                    add_line(par,[get_param(bl{q},'Name') sprintf('/%d',z)],[get_param(tn,'Name') '/1']);
                catch, bad = [bad sprintf(' out%d',z)]; end %#ok<AGROW>
            end
        end
        if ~isempty(bad) && ~any(strcmp(get_param(bl{q},'BlockType'),{'Terminator','Ground'})), fprintf('[Phase3] WARNING unconnected port(s) in "%s":%s\n',bl{q},bad); end
    end
catch ME, fprintf('[Phase3] port check skipped: %s\n',ME.message);
end

set_param(modelName,'StopFcn','try, fleetVisionStep(''close''); catch, end; try, phase3(''stopped''); catch, end');
set_param(modelName,'SimulationCommand','update');
save_system(modelName,modelFile); open_system(modelName);
fprintf('[Phase3] Created %s with %d robots, %d cartons\n',modelFile,N,M);
end

% ====================================================================== one robot = subsystem 'Robot k'
function localRobot(mdl,k,C,M)
sub = sprintf('%s/Robot %d',mdl,k);
add_block('simulink/Ports & Subsystems/Subsystem',sub,'Position',[150 60+90*(k-1) 330 120+90*(k-1)]);
ln = find_system(sub,'SearchDepth',1,'FindAll','on','Type','line'); for q = 1:numel(ln), delete_line(ln(q)); end
delete_block([sub '/In1']); delete_block([sub '/Out1']);
Ck = C; Ck.HomeXY = C.P3.robots(k).home;
dt = num2str(C.SampleTime);
add_block('simulink/Sources/In1',[sub '/Pool']);
add_block('simulink/Sources/In1',[sub '/EstPose']); set_param([sub '/EstPose'],'Port','2');       % Round 12: pose the robot BELIEVES (Phase3Sense: wheel odometry + lidar AMCL)
add_block('simulink/Sources/In1',[sub '/SenseBus']); set_param([sub '/SenseBus'],'Port','3');     % lidar tracking / protective field
add_block('simulink/Sources/In1',[sub '/CamBus']); set_param([sub '/CamBus'],'Port','4');         % camera carton detection
add_block('simulink/Sinks/Out1',[sub '/Pose']); add_block('simulink/Sinks/Out1',[sub '/Pack']); add_block('simulink/Sinks/Out1',[sub '/Lift']); add_block('simulink/Sinks/Out1',[sub '/State']);
% ---- robot, controller, FSM
robot = localAdd(sub,'robotsim3dlib','Simulation 3D Robot','Robot Kinova',[1090 170 1320 400]); set_param(robot,'Priority','2');
set_param(robot,'actorInputType','RigidBodyTree Object'); set_param(robot,'RigidBodyTree','gen3');
set_param(robot,'ActorName',sprintf('Ro%dKinova',k),'inpConfEnbVal','on','inpTransEnbVal','on','inpGraspEnbVal','on');
set_param(robot,'enableRegionGraspChekBox','on','regionGraspInpChekBox','on','outConfEnbVal','on','outGraspEnbVal','on','EndEffector','ToolTip');
localTrySet(robot,'SampleTime',dt); localTrySet(robot,'initialConfig','qStow');
localTrySet(robot,'eeTargetTrans',mat2str(C.GraspOffsetTrans)); localTrySet(robot,'eeTargetRot',mat2str(C.GraspOffsetRot));
buildRobotTaskFSM(sub,Ck,[270 90 610 390]);
ctl = localAdd(sub,'simulink','MATLAB System','Navigation Arm Controller',[690 120 910 330]);
set_param(ctl,'System','WarehouseRobotControlSystemV23'); set_param(ctl,'RobotId',num2str(k)); set_param(ctl,'SimulateUsing','Interpreted execution');
cp = get_param(ctl,'PortHandles');
assert(numel(cp.Inport)==12 && numel(cp.Outport)==20,'Phase3:ControllerPorts','controller must have 12 inputs / 20 outputs');
dd = localAdd(sub,'robotmobilelib','Differential Drive Kinematic Model','Differential Drive Kinematics',[465 460 625 535]);
set_param(dd,'VehicleInputs','Vehicle Speed & Heading Angular Velocity','InitialState',sprintf('HomePose%d',k));
pd = localAdd(sub,'simulink','Unit Delay','Base Pose Delay',[660 425 730 455]); set_param(pd,'InitialCondition',sprintf('HomePose%d',k),'SampleTime',dt);
% ---- one-tick delays that break the controller <-> FSM loops
dl = {'AtGoal Delay',3,1;'ArmDone Delay',4,2;'Arm Fault Delay',5,8;'Grasped Delay',6,7;'Perception Valid Delay',16,3;'More Boxes Delay',17,9;'Scan Done Delay',18,10;'Finish Delay',19,11;'Need Home Delay',20,12};
for q = 1:size(dl,1)
    y0 = 125+45*(q-1); b = localAdd(sub,'simulink','Unit Delay',dl{q,1},[185 y0 235 y0+30]);
    set_param(b,'InitialCondition','false','SampleTime',dt);
    localLine(sub,sprintf('Navigation Arm Controller/%d',dl{q,2}),[dl{q,1} '/1']); localLine(sub,[dl{q,1} '/1'],sprintf('RobotTaskFSM/%d',dl{q,3}));
end
localLine(sub,'Grasped Delay/1','Navigation Arm Controller/8');
for q = 1:3
    zc = localAdd(sub,'simulink','Constant',sprintf('Box Coord Zero %d',q),[115 530+35*(q-1) 155 550+35*(q-1)]); set_param(zc,'Value','0','OutDataTypeStr','double');
    localLine(sub,sprintf('Box Coord Zero %d/1',q),sprintf('RobotTaskFSM/%d',q+3));
end
for q = 1:6, localLine(sub,sprintf('RobotTaskFSM/%d',q),sprintf('Navigation Arm Controller/%d',q)); end
localLine(sub,'EstPose/1','Navigation Arm Controller/7');                 % belief pose (= vehicle pose when C.Sense.Localization is 'truth')
sbd = localAdd(sub,'simulink','Unit Delay','Sense Bus Delay',[185 700 235 730]); set_param(sbd,'InitialCondition','zeros(1,8)','SampleTime',dt);
cbd = localAdd(sub,'simulink','Unit Delay','Cam Bus Delay',[185 750 235 780]); set_param(cbd,'InitialCondition','zeros(1,8)','SampleTime',dt);
localLine(sub,'SenseBus/1','Sense Bus Delay/1'); localLine(sub,'Sense Bus Delay/1','Navigation Arm Controller/10');
localLine(sub,'CamBus/1','Cam Bus Delay/1'); localLine(sub,'Cam Bus Delay/1','Navigation Arm Controller/11');
localLine(sub,'Base Pose Delay/1','Navigation Arm Controller/12');          % true vehicle pose: physics emulation only
localLine(sub,'RobotTaskFSM/1','State/1');
% ---- carton measurement for the job carton (index delayed one tick: no algebraic loop)
sel = [sub '/Idx Select']; add_block('simulink/User-Defined Functions/MATLAB Function',sel,'Position',[420 600 520 640]);
localSetMATLABFunction(sel,sprintf('function idx = fcn(c2)\nidx = c2(1);\n'));
idl = localAdd(sub,'simulink','Unit Delay','Idx Delay',[560 600 620 630]); set_param(idl,'InitialCondition','0','SampleTime',dt);
pick = [sub '/Box Meas Pick']; add_block('simulink/User-Defined Functions/MATLAB Function',pick,'Position',[680 580 800 650]);
localSetMATLABFunction(pick,sprintf(['function y = fcn(pool,idx)\n' 'y = zeros(1,9); i = round(idx);\n' 'if i>=1 && i<=%d, y(1:3) = pool(3*i-2:3*i); end\n'],M));
localLine(sub,'Navigation Arm Controller/8','Idx Select/1'); localLine(sub,'Idx Select/1','Idx Delay/1'); localLine(sub,'Idx Delay/1','Box Meas Pick/2');
localLine(sub,'Pool/1','Box Meas Pick/1'); localLine(sub,'Box Meas Pick/1','Navigation Arm Controller/9');
% ---- outputs to the top level
pk = [sub '/Robot Pack']; add_block('simulink/User-Defined Functions/MATLAB Function',pk,'Position',[900 600 1020 680]);
localSetMATLABFunction(pk,sprintf('function y = fcn(c,r,d,c2)\ny = [c r double(d) c2(1)];\n'));
localLine(sub,'Navigation Arm Controller/7','Robot Pack/1'); localLine(sub,'Navigation Arm Controller/10','Robot Pack/2');
localLine(sub,'Navigation Arm Controller/13','Robot Pack/3'); localLine(sub,'Navigation Arm Controller/8','Robot Pack/4');
localLine(sub,'Robot Pack/1','Pack/1'); localLine(sub,'Base Pose Delay/1','Pose/1');
ls = [sub '/Lift Select']; add_block('simulink/User-Defined Functions/MATLAB Function',ls,'Position',[900 700 1000 740]);
localSetMATLABFunction(ls,sprintf('function y = fcn(q)\ny = q(1);\n'));
localLine(sub,'Navigation Arm Controller/2','Lift Select/1'); localLine(sub,'Lift Select/1','Lift/1');
for q = [9 11 12 14 15]
    tm = add_block('simulink/Sinks/Terminator',sprintf('%s/Ctl Unused %d',sub,q),'Position',[1100 700+20*q 1120 715+20*q]);
    localLine(sub,sprintf('Navigation Arm Controller/%d',q),sprintf('Ctl Unused %d/1',q));
end
% ---- drive + robot actor
vd = localAdd(sub,'simulink','Demux','Velocity Split',[420 465 440 520]); set_param(vd,'Outputs','2');
localLine(sub,'Navigation Arm Controller/1','Velocity Split/1'); localLine(sub,'Velocity Split/1','Differential Drive Kinematics/1');
localLine(sub,'Velocity Split/2','Differential Drive Kinematics/2'); localLine(sub,'Differential Drive Kinematics/1','Base Pose Delay/1');
pt = localAdd(sub,'simulink','MATLAB Function','Pose To Robot Transform',[775 395 965 500]);
localSetMATLABFunction(pt,sprintf(['function [translation,rotation] = fcn(pose)\n' 'translation = [pose(1), pose(2), 0];\n' 'rotation = [pose(3), 0, 0];\n']));
localLine(sub,'Base Pose Delay/1','Pose To Robot Transform/1');
ta = localAdd(sub,'simulink','Constant','Target Actor',[920 350 1020 380]); set_param(ta,'Value','uint8(''TargetBox'')','OutDataTypeStr','uint8','VectorParams1D','on');
tt = localAdd(sub,'simulink','Data Type Conversion','Target Actor uint8',[1035 350 1070 380]); set_param(tt,'OutDataTypeStr','uint8');
gT = localAdd(sub,'simulink','Constant','Grasp Translation',[920 390 1020 420]); set_param(gT,'Value',mat2str(C.GraspRegionTranslation));
gR = localAdd(sub,'simulink','Constant','Grasp Rotation',[920 430 1020 460]); set_param(gR,'Value',mat2str(C.GraspRegionRotation));
off = localAdd(sub,'simulink','Constant','Native Grasp Off',[920 470 1020 500]); set_param(off,'Value','false','OutDataTypeStr','boolean');
localLine(sub,'Target Actor/1','Target Actor uint8/1');
localLine(sub,'Navigation Arm Controller/2','Robot Kinova/1'); localLine(sub,'Pose To Robot Transform/1','Robot Kinova/2'); localLine(sub,'Pose To Robot Transform/2','Robot Kinova/3');
localLine(sub,'Target Actor uint8/1','Robot Kinova/4'); localLine(sub,'Native Grasp Off/1','Robot Kinova/5');
localLine(sub,'Grasp Translation/1','Robot Kinova/6'); localLine(sub,'Grasp Rotation/1','Robot Kinova/7');
for q = 1:2
    add_block('simulink/Sinks/Terminator',sprintf('%s/Robot Out Unused %d',sub,q),'Position',[1360 300+30*q 1380 320+30*q]);
    localLine(sub,sprintf('Robot Kinova/%d',q),sprintf('Robot Out Unused %d/1',q));
end
end

% ====================================================================== helpers (same as the Phase 2 builder)
function block = localAdd(model,library,blockName,newName,position)
if strcmp(library,'simulink')
    switch blockName
        case 'MATLAB System', source = 'simulink/User-Defined Functions/MATLAB System';
        case 'Unit Delay', source = 'simulink/Discrete/Unit Delay';
        case 'MATLAB Function', source = 'simulink/User-Defined Functions/MATLAB Function';
        case 'Demux', source = 'simulink/Signal Routing/Demux';
        case 'Constant', source = 'simulink/Sources/Constant';
        case 'Data Type Conversion', source = 'simulink/Signal Attributes/Data Type Conversion';
        otherwise, error('Phase3:BlockNotFound','No source mapping for "%s".',blockName);
    end
else
    load_system(library); found = find_system(library,'SearchDepth',1,'FollowLinks','on','Name',blockName);
    assert(~isempty(found),'Phase3:BlockNotFound','Could not find "%s" in %s.',blockName,library); source = found{1};
end
block = [model '/' newName]; add_block(source,block,'Position',position);
end
function localLine(model,source,destination), add_line(model,source,destination,'autorouting','on'); end
function localTrySet(block,name,value), try, set_param(block,name,value); catch, end, end
function localSetMATLABFunction(block,script)
rt = sfroot; chart = rt.find('-isa','Stateflow.EMChart','Path',block); chart.Script = script;
end


% ====================================================================== Round 12: per-robot sensing rig
function C = localSenseRig(mdl,k,C,N) %#ok<INUSD>
S = C.Sense; m = helperVisionCameraMount(); yy = 1300 + 260*(k-1);
% ---- localization / tracking block
sn = sprintf('Robot %d Sense',k);
sb = localAdd(mdl,'simulink','MATLAB System',sn,[2400 yy 2560 yy+90]);
set_param(sb,'System','Phase3Sense'); set_param(sb,'RobotId',num2str(k)); set_param(sb,'SimulateUsing','Interpreted execution');
% ---- lidar (Unreal) or placeholder
lidarBlk = '';
if strcmpi(S.LidarMode,'unreal') && (strcmpi(S.Localization,'amcl') || S.Tracking)
    try
        lidarBlk = localAddLidar(mdl,k,C,yy);
    catch ME
        C.Sense.LidarMode = 'raycast'; S = C.Sense;
        fprintf('[Phase3] WARNING Unreal lidar block could not be built (%s).\n         -> the lidar is ray-cast from the map instead (C.Sense.LidarMode=''raycast''); the sensing code is otherwise the same.\n',ME.message);
    end
end
if isempty(lidarBlk)
    lz = localAdd(mdl,'simulink','Constant',sprintf('Robot %d Lidar Off',k),[2200 yy 2260 yy+30]); set_param(lz,'Value','[0 0 0]','OutDataTypeStr','double','VectorParams1D','off');
    lidarBlk = sprintf('Robot %d Lidar Off',k);
end
localLine(mdl,sprintf('Robot %d/1',k),[sn '/1']);
localLine(mdl,[lidarBlk '/1'],[sn '/2']);
localLine(mdl,[sn '/1'],sprintf('Robot %d/2',k)); localLine(mdl,[sn '/2'],sprintf('Robot %d/3',k));
% ---- camera + carton detection
camOk = false;
if strcmpi(S.Camera,'camera')
    try
        cp = localAdd(mdl,'simulink','MATLAB Function',sprintf('Robot %d Cam Pose',k),[2000 yy+120 2160 yy+190]);
        localSetMATLABFunction(cp,sprintf([ ...
            'function [translation,rotation] = fcn(pose,lift)\n' ...
            'yaw=pose(3); c=cos(yaw); s=sin(yaw);\n' ...
            'f=%.6g; l=%.6g; z=%.6g+lift;\n' ...
            'translation=[pose(1)+f*c-l*s pose(2)+f*s+l*c z];\n' ...
            'rotation=[0 %.6g yaw*180/pi];\n'],0.10+m(1),m(2),0.47+m(3),m(4)));
        src = find_system('sim3dcameralib','SearchDepth',1,'FollowLinks','on','Name','Simulation 3D Camera');
        camB = [mdl '/' sprintf('Robot %d Camera',k)]; add_block(src{1},camB,'Position',[2200 yy+110 2360 yy+210]);
        set_param(camB,'Priority','1','vehTag','Scene Origin','offsetFlag','on','extTmount','on','extRmount','on', ...
            'ImageSize',mat2str(C.CameraImageSize),'FocalLength',mat2str([m(5) m(5)]),'OpticalCenter',mat2str(C.CameraPrincipalPoint));
        localTrySet(camB,'SampleTime',num2str(S.CamPeriod));
        localSetByPrompt(camB,'^Sensor identifier',num2str(10+k));
        localLine(mdl,sprintf('Robot %d/1',k),sprintf('Robot %d Cam Pose/1',k)); localLine(mdl,sprintf('Robot %d/3',k),sprintf('Robot %d Cam Pose/2',k));
        localLine(mdl,sprintf('Robot %d Cam Pose/1',k),sprintf('Robot %d Camera/1',k)); localLine(mdl,sprintf('Robot %d Cam Pose/2',k),sprintf('Robot %d Camera/2',k));
        cd_ = localAdd(mdl,'simulink','MATLAB System',sprintf('Robot %d Cam Detect',k),[2600 yy+110 2760 yy+210]);
        set_param(cd_,'System','Phase3CamDetect'); set_param(cd_,'RobotId',num2str(k)); set_param(cd_,'SimulateUsing','Interpreted execution');
        localLine(mdl,sprintf('Robot %d Camera/1',k),sprintf('Robot %d Cam Detect/1',k));
        localLine(mdl,sprintf('Robot %d/4',k),sprintf('Robot %d Cam Detect/2',k));
        localLine(mdl,[sn '/1'],sprintf('Robot %d Cam Detect/3',k));
        localLine(mdl,sprintf('Robot %d/3',k),sprintf('Robot %d Cam Detect/4',k));
        localLine(mdl,sprintf('Robot %d Cam Detect/1',k),sprintf('Robot %d/4',k));
        camOk = true;
    catch ME
        C.Sense.Camera = 'sim'; S = C.Sense;
        fprintf('[Phase3] WARNING camera detection rig of robot %d could not be built (%s).\n         -> camera detection OFF (C.Sense.Camera=''sim'': scene readback as before).\n',k,ME.message);
        for nm_ = {sprintf('Robot %d Cam Pose',k),sprintf('Robot %d Camera',k),sprintf('Robot %d Cam Detect',k)}
            try, delete_block([mdl '/' nm_{1}]); catch, end
        end
    end
end
if ~camOk
    cz = localAdd(mdl,'simulink','Constant',sprintf('Robot %d Cam Off',k),[2600 yy+110 2660 yy+140]); set_param(cz,'Value','zeros(1,8)','OutDataTypeStr','double','VectorParams1D','off');
    localLine(mdl,sprintf('Robot %d Cam Off/1',k),sprintf('Robot %d/4',k));
    if k == 1 && ~strcmpi(S.Camera,'camera'), fprintf('[Phase3] camera detection: OFF (scene readback)\n'); end
end
end

function blk = localAddLidar(mdl,k,C,yy)
% Unreal lidar (Simulation 3D Lidar, Automated Driving Toolbox Interface for Unreal Engine Projects) mounted on the robot.  Dialog parameter names are NOT
% assumed: every setting is found by its dialog prompt, and what was / was not found is printed and written to the log folder.
S = C.Sense;
[f,tried] = p3FindBlock('Simulation 3D Lidar');
if isempty(f), error('block "Simulation 3D Lidar" not found (libraries searched: %s)',strjoin(tried,', ')); end
name = sprintf('Robot %d Lidar',k); blk = name; full = [mdl '/' name];
add_block(f,full,'Position',[2200 yy 2360 yy+100]);
rec = struct('got',{{}},'miss',{{}});
% exact dialog parameters of the R2024b block (from p3sense): sensorId, vehTag (Parent name), mountLoc, offsetFlag, tmountOffset, rmountOffset, DetectionRange,
% VerticalFOV, VerticalResolution, HorizontalFOV, HorizontalResolution, SampleTime.  The prompt is the fallback if a name differs in another release.
rec = localTryN(rec,full,'id','sensorId','^Sensor identifier',num2str(20+k));
rec = localTryN(rec,full,'parent','vehTag','^Parent name:?$',sprintf('Ro%dKinova',k));
rec = localTryN(rec,full,'mount','mountLoc','^Mounting location','Origin');
rec = localTryN(rec,full,'offset','offsetFlag','^Specify offset','on');
rec = localTryN(rec,full,'translation','tmountOffset','^Relative translation',mat2str([S.LidarOffset(1) S.LidarOffset(2) S.LidarZ]));
rec = localTryN(rec,full,'rotation','rmountOffset','^Relative rotation',mat2str([0 0 0]));
rec = localTryN(rec,full,'sample time','SampleTime','^Sample time',num2str(S.LidarPeriod));
rec = localTryN(rec,full,'range','DetectionRange','^Detection range',num2str(S.LidarMaxRange));
rec = localTryN(rec,full,'h fov','HorizontalFOV','^Horizontal field of view','360');
rec = localTryN(rec,full,'h res','HorizontalResolution','^Horizontal resolution',num2str(360/S.LidarBeams));
rec = localTryN(rec,full,'v fov','VerticalFOV','^Vertical field of view','2');
rec = localTryN(rec,full,'v res','VerticalResolution','^Vertical resolution','2');
got = rec.got; miss = rec.miss;
localTrySet(full,'Priority','3');
fprintf('[Phase3] Robot %d lidar: set {%s}; no matching dialog parameter for {%s}\n',k,strjoin(got,', '),strjoin(miss,', '));
if k == 1
    try
        P = phase3Paths(); if ~isfolder(P.diag), mkdir(P.diag); end
        fid = fopen(fullfile(P.diag,'lidar_block_parameters.txt'),'w'); dp = get_param(full,'DialogParameters'); fn = fieldnames(dp);
        for q = 1:numel(fn), fprintf(fid,'%s | %s | %s\n',fn{q},dp.(fn{q}).Prompt,localStr(get_param(full,fn{q}))); end
        fclose(fid);
    catch
    end
end
end

function rec = localTryN(rec,blk,tag,pname,rx,val)
ok = false;
try, set_param(blk,pname,val); ok = true; catch, end
if ~ok, ok = localSetByPrompt(blk,rx,val); end
if ok, rec.got{end+1} = tag; else, rec.miss{end+1} = tag; end
end

function ok = localSetByPrompt(blk,rx,val)
ok = false;
try
    dp = get_param(blk,'DialogParameters'); fn = fieldnames(dp);
    for q = 1:numel(fn)
        pr = dp.(fn{q}).Prompt;
        if ~isempty(regexpi(pr,rx,'once')), set_param(blk,fn{q},val); ok = true; return, end
    end
catch
end
end
function s = localStr(v), try, s = char(string(v)); catch, s = '?'; end, end
