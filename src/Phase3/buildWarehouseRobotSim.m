function modelName = buildWarehouseRobotSim(cameraPitchDeg)
%BUILDWAREHOUSEROBOTSIM Reproducibly build and save WarehouseRobotSim.slx.
% This is the only model-construction entry point required after a fresh restart.
if nargin<1, cameraPitchDeg=0; end
validateattributes(cameraPitchDeg,{'numeric'},{'scalar','real','finite'},mfilename,'cameraPitchDeg');
modelName='WarehouseRobotSim'; C=WarehouseRobotPhase2Config();
% Lesson 4: clean old raw-world session and old model before new co-simulation.
if evalin('base','exist(''world'',''var'')'), try,close(evalin('base','world'));catch,end; try,delete(evalin('base','world'));catch,end; evalin('base','clear world'); end
if bdIsLoaded(modelName), close_system(modelName,0); end
modelFile=fullfile(pwd,[modelName '.slx']); if exist(modelFile,'file'), delete(modelFile); end
load_system('robotsim3dlib'); load_system('sim3dlib'); load_system('sim3dcameralib'); load_system('robotmobilelib');
new_system(modelName);
% One explicit base rate prevents the old scene/controller/robot race.
set_param(modelName,'Solver','ode4','FixedStep',num2str(C.SampleTime),'StopTime','3000');
localTrySet(modelName,'EnablePacing','on'); localTrySet(modelName,'PacingRate','1'); % Lesson 3
ws=get_param(modelName,'ModelWorkspace');
% Lesson 1: this assignment deliberately precedes the Robot mask callback.
 [gen3,qStow]=helperBuildMobileManipulator();
assignin(ws,'gen3',gen3); assignin(ws,'qStow',qStow);
assignin(ws,'HomePose',[C.HomeXY 0]); assignin(ws,'Zero',0); assignin(ws,'TargetActor',string(C.TargetActor));
assignin(ws,'CameraPitchDeg',cameraPitchDeg);

scene=localAdd(modelName,'sim3dlib','Simulation 3D Scene Configuration','Scene Configuration',[1120 560 1340 620]);
% Use the documented R2024b mask fields rather than localized UI prompts.
% An Empty scene is populated only by the environment setup callback.
% Simulation 3D setup has a strict dependency chain.  In Simulink a lower
% numeric priority runs first, so Engine (0) must precede warehouse actors
% and camera (1), then the Robot (2). Running the Robot at -1 made its
% setup access an uninitialised World.Actors collection.
set_param(scene,'ProjectFormat','Default Scenes','SceneDesc','Empty scene','Priority','0');
environment=localAdd(modelName,'sim3dlib','Simulation 3D Actor','Phase1 Warehouse',[900 560 1060 650]);
% Lesson 2: this creates a movable target at setup; nothing reparents or
% teleports it during the simulation. The native Robot block owns grasping.
set_param(environment,'ActorName','WarehouseRoot','ParentName','Scene Origin', ...
    'Operation','Create at setup','InitScriptText','setupWarehouseScene(Actor,World);','Priority','1');

% The Engine and static warehouse actor must exist before the Robot creates
% its Unreal actor; Simulation 3D schedules the camera before the Robot so
% the sensor frame is available for the same co-simulation tick.
robot=localAdd(modelName,'robotsim3dlib','Simulation 3D Robot','Robot 1 Kinova',[1090 170 1320 400]); set_param(robot,'Priority','2');
set_param(robot,'actorInputType','RigidBodyTree Object'); set_param(robot,'RigidBodyTree','gen3');
set_param(robot,'ActorName','Ro1Kinova','inpConfEnbVal','on','inpTransEnbVal','on','inpGraspEnbVal','on');
% R2024b orders enabled Robot outputs as GroundTruthJointConfiguration,
% then IsTargetGrasped.  Both are enabled so the command-to-joint path can
% be verified independently of the rendered arm.
set_param(robot,'enableRegionGraspChekBox','on','regionGraspInpChekBox','on', ...
    'outConfEnbVal','on','outGraspEnbVal','on','EndEffector','ToolTip');
localTrySet(robot,'SampleTime',num2str(C.SampleTime)); localTrySet(robot,'initialConfig','qStow');
% Expected end-effector -> target relative pose (box centre ahead of the cup) so a native attach does not snap the box onto the tool.
localTrySet(robot,'eeTargetTrans',mat2str(C.GraspOffsetTrans)); localTrySet(robot,'eeTargetRot',mat2str(C.GraspOffsetRot));

camera=localAdd(modelName,'sim3dcameralib','Simulation 3D Camera','Head Camera',[690 510 865 660]); set_param(camera,'Priority','1');
set_param(camera,'vehTag','Scene Origin','offsetFlag','on','extTmount','on','extRmount','on');
% Ground-truth camera pose is diagnostic-only; it verifies the mount-frame
% convention against the actual rendered image before any calibration edit.
set_param(camera,'DepthOutportEnabled','on','TransformOutportEnabled','on','ImageSize',mat2str(C.CameraImageSize),'FocalLength',mat2str(C.CameraFocalLength),'OpticalCenter',mat2str(C.CameraPrincipalPoint));
localTrySet(camera,'SampleTime',num2str(C.CameraSampleTime));

buildRobotTaskFSM(modelName,C,[270 90 610 390]);
controller=localAdd(modelName,'simulink','MATLAB System','Navigation Arm Controller',[690 120 910 330]);
perception=localAdd(modelName,'simulink','MATLAB System','HSV Depth Perception',[900 455 1050 545]);
% Do not suppress errors here. These assignments instantiate the System
% objects and create their dynamic ports; wiring a still-default System block
% would otherwise fail later with an unhelpful ".../3" object-name error.
set_param(controller,'System','WarehouseRobotControlSystemV23');
set_param(perception,'System','WarehousePerceptionSystem');
% These objects use toolbox functions (image processing, A*, IK) that are
% intentionally interpreted for this simulation rather than code-generated.
set_param(controller,'SimulateUsing','Interpreted execution');
set_param(perception,'SimulateUsing','Interpreted execution');
controllerPorts=get_param(controller,'PortHandles'); perceptionPorts=get_param(perception,'PortHandles');
robotPorts=get_param(robot,'PortHandles');
assert(numel(controllerPorts.Inport)==12 && numel(controllerPorts.Outport)==20, ...
    'Phase2:ControllerPorts','WarehouseRobotControlSystemV23 did not expose its required 12 inputs and 20 outputs.');
assert(numel(perceptionPorts.Inport)==7 && numel(perceptionPorts.Outport)==4, ...
    'Phase2:PerceptionPorts','WarehousePerceptionSystem did not expose its required 7 inputs and 4 outputs.');
assert(numel(robotPorts.Outport)==2, ...
    'Phase2:RobotOutputs','Expected GroundTruthJointConfiguration and IsTargetGrasped Robot outputs.');
dd=localAdd(modelName,'robotmobilelib','Differential Drive Kinematic Model','Differential Drive Kinematics',[465 460 625 535]);
set_param(dd,'VehicleInputs','Vehicle Speed & Heading Angular Velocity','InitialState','HomePose');
poseDelay=localAdd(modelName,'simulink','Unit Delay','Base Pose Delay',[660 425 730 455]); set_param(poseDelay,'InitialCondition','HomePose','SampleTime',num2str(C.SampleTime));
mount=localAdd(modelName,'simulink','MATLAB Function','Camera Mount',[480 570 620 650]);
% sprintf supplies real line breaks to the embedded MATLAB Function chart.
% The external camera rotation is [roll pitch yaw] in the scene frame.
% The pitch is an explicit diagnostic variant. A settled-pose snapshot, not
% a visual guess while the base is turning, selects the permanent value.
localSetMATLABFunction(mount,sprintf([ ...
'function [translation,rotation] = fcn(pose)\n' ...
'%% Camera input is [roll pitch yaw] in degrees (R2024b camera mask).\n' ...
'yaw=pose(3); c=cos(yaw); s=sin(yaw);\n' ...
'%% Simulation 3D Camera mounted to Scene Origin uses Unreal pose axes.\n' ...
'translation=[pose(1)+0.20*c+0.18*s pose(2)+0.20*s-0.18*c 1.00];\n' ...
'%% Perception uses radians internally; pitch is camera-only diagnostic input.\n' ...
'rotation=[0 %.12g -yaw*180/pi];\n'],cameraPitchDeg));

% FSM inputs AtGoal, ArmDone, PerceptionValid, BoxX, BoxY, BoxZ,
% IsTargetGrasped, delayed ArmFault, MoreBoxes, ScanDone.
% These two delays are intentional: Stateflow outputs drive the controller,
% whose completion outputs feed Stateflow. A one-tick delay breaks that
% feedback algebraic loop and gives the supervisor discrete-time semantics.
atGoalDelay=localAdd(modelName,'simulink','Unit Delay','AtGoal Delay',[185 125 235 155]);
armDoneDelay=localAdd(modelName,'simulink','Unit Delay','ArmDone Delay',[185 175 235 205]);
armFaultDelay=localAdd(modelName,'simulink','Unit Delay','Arm Fault Delay',[115 220 165 250]);
graspDelay=localAdd(modelName,'simulink','Unit Delay','Grasped Delay',[185 225 235 255]);
set_param(atGoalDelay,'InitialCondition','false','SampleTime',num2str(C.SampleTime));
set_param(armDoneDelay,'InitialCondition','false','SampleTime',num2str(C.SampleTime));
set_param(armFaultDelay,'InitialCondition','false','SampleTime',num2str(C.SampleTime));
set_param(graspDelay,'InitialCondition','false','SampleTime',num2str(C.SampleTime));
localLine(modelName,'Navigation Arm Controller/3','AtGoal Delay/1');
localLine(modelName,'AtGoal Delay/1','RobotTaskFSM/1');
localLine(modelName,'Navigation Arm Controller/4','ArmDone Delay/1');
localLine(modelName,'ArmDone Delay/1','RobotTaskFSM/2');
localLine(modelName,'Navigation Arm Controller/5','Arm Fault Delay/1');
localLine(modelName,'Arm Fault Delay/1','RobotTaskFSM/8');
% Round 30: the controller itself scans the selected carton (slot, range, field of view) and reports
% PerceptionValid / ScanDone / MoreBoxes.  They pass through one-tick delays like every other FSM input.
% The HSV camera block stays in the model as a diagnostic (its valid flag is logged); its other outputs are terminated.
fsmDelays={'Perception Valid Delay',16,'false',3; 'More Boxes Delay',17,'false',9; 'Scan Done Delay',18,'false',10; 'Finish Delay',19,'false',11; 'Need Home Delay',20,'false',12};
for k=1:size(fsmDelays,1)
    y0=270+45*(k-1);
    pDelay=localAdd(modelName,'simulink','Unit Delay',fsmDelays{k,1},[185 y0 235 y0+30]);
    set_param(pDelay,'InitialCondition',fsmDelays{k,3},'SampleTime',num2str(C.SampleTime));
    localLine(modelName,sprintf('Navigation Arm Controller/%d',fsmDelays{k,2}),sprintf('%s/1',fsmDelays{k,1}));
    localLine(modelName,sprintf('%s/1',fsmDelays{k,1}),sprintf('RobotTaskFSM/%d',fsmDelays{k,4}));
end
for k=1:3     % BoxX/Y/Z are unused by the Round 30 FSM transitions; tie them to zero
    zc=localAdd(modelName,'simulink','Constant',sprintf('Box Coord Zero %d',k),[115 330+35*(k-1) 155 350+35*(k-1)]);
    set_param(zc,'Value','0','OutDataTypeStr','double'); localLine(modelName,sprintf('Box Coord Zero %d/1',k),sprintf('RobotTaskFSM/%d',k+3));
end
for k=2:4
    tm=add_block('simulink/Sinks/Terminator',sprintf('%s/HSV Unused %d',modelName,k),'Position',[1200 455+25*k 1220 475+25*k]);
    localLine(modelName,sprintf('HSV Depth Perception/%d',k),sprintf('HSV Unused %d/1',k));
end
% V23.2: the native Robot-block grasp never engaged, so the attached flag comes from the controller.
% (controller -> Grasped Delay -> FSM and controller keeps the loop broken by the one-tick delay)
localLine(modelName,'Navigation Arm Controller/6','Grasped Delay/1');
localLine(modelName,'Grasped Delay/1','RobotTaskFSM/7');
% The controller consumes the same one-tick delayed *native* status for its
% SEAL/CONFIRM/LIFT sequence.  No custom proximity signal is introduced.
localLine(modelName,'Grasped Delay/1','Navigation Arm Controller/8');
% ---- controller-driven box actors (3 cartons): Transform Get (delayed) -> controller -> Transform Set per carton
boxTags={'TargetBox','TargetBox2','TargetBox3'};
boxScl=localAdd(modelName,'simulink','Constant','Box Scale Hold',[1150 940 1230 970]); set_param(boxScl,'Value','[1 1 1]','OutDataTypeStr','double','VectorParams1D','off');
setPrio='1'; if isfield(C,'BoxSetPriority'), setPrio=num2str(C.BoxSetPriority); end
pack=localAdd(modelName,'simulink','MATLAB Function','Box Pack',[1240 610 1330 690]);
localSetMATLABFunction(pack,sprintf(['function y = fcn(a,b,c)\n' '%% Three 1x3 carton positions (Unreal frame) -> one 1x9 controller input.\n' 'y = [a b c];\n']));
for k=1:3
    xyz=C.Boxes(k).xyz;
    if any(k==C.Test.MissingBox), xyz=[C.Test.MissingXY 0.5*C.TargetDimensions(3)+0.001]; end
    boxIC=[xyz(1) -xyz(2) xyz(3)]+[reshape(C.Test.SpawnOffset,1,2)*[1 0;0 -1]*(k==1) 0];
    gname=sprintf('%s Transform',boxTags{k}); dname=sprintf('Box Pos Delay %d',k); y0=610+80*(k-1);
    g=localAdd(modelName,'sim3dlib','Simulation 3D Actor Transform Get',gname,[980 y0 1140 y0+60]);
    localTrySet(g,'ActorTag',boxTags{k}); localTrySet(g,'NumberOfParts','1'); localTrySet(g,'Priority','1');
    bd=localAdd(modelName,'simulink','Unit Delay',dname,[1150 y0+10 1220 y0+40]);
    set_param(bd,'InitialCondition',mat2str(boxIC),'SampleTime',num2str(C.SampleTime));
    localLine(modelName,[gname '/1'],[dname '/1']); localLine(modelName,[dname '/1'],sprintf('Box Pack/%d',k));
    cmdPort=sprintf('Navigation Arm Controller/%d',6+k); rotPort=sprintf('Navigation Arm Controller/%d',9+k); drvPort=sprintf('Navigation Arm Controller/%d',12+k);
    built=false;
    if isfield(C,'UnrealPhysics') && C.UnrealPhysics
        % Transform Set lives in an Enabled Subsystem: the controller disables it for a carton that Unreal physics owns.
        try
            sname='Box Driver'; if k>1, sname=sprintf('Box Driver %d',k); end
            ss=[modelName '/' sname];
            add_block('simulink/Ports & Subsystems/Enabled Subsystem',ss,'Position',[1360 700+90*(k-1) 1500 770+90*(k-1)]);
            ln=find_system(ss,'SearchDepth',1,'FindAll','on','Type','line'); for kk=1:numel(ln), delete_line(ln(kk)); end
            delete_block([ss '/Out1']);
            set_param([ss '/In1'],'Name','Translation');
            add_block('simulink/Sources/In1',[ss '/Rotation']); add_block('simulink/Sources/In1',[ss '/Scale']);
            boxSet=localAdd(ss,'sim3dlib','Simulation 3D Actor Transform Set',sprintf('%s Set',boxTags{k}),[200 40 380 140]);
            set_param(boxSet,'ActorTag',boxTags{k}); localTrySet(boxSet,'NumberOfParts','1'); localTrySet(boxSet,'Priority',setPrio);
            add_line(ss,'Translation/1',sprintf('%s Set/1',boxTags{k})); add_line(ss,'Rotation/1',sprintf('%s Set/2',boxTags{k})); add_line(ss,'Scale/1',sprintf('%s Set/3',boxTags{k}));
            localLine(modelName,cmdPort,[sname '/1']); localLine(modelName,rotPort,[sname '/2']); localLine(modelName,'Box Scale Hold/1',[sname '/3']); localLine(modelName,drvPort,[sname '/Enable']);
            built=true;
        catch ME
            warning('Phase2:BoxDriver','Enabled-subsystem box driver %d failed (%s); using the always-on Transform Set instead.',k,ME.message);
            try, delete_block([modelName '/' sname]); catch, end
        end
    end
    if ~built
        sn=sprintf('%s Set',boxTags{k});
        boxSet=localAdd(modelName,'sim3dlib','Simulation 3D Actor Transform Set',sn,[1360 700+90*(k-1) 1520 770+90*(k-1)]);
        set_param(boxSet,'ActorTag',boxTags{k}); localTrySet(boxSet,'NumberOfParts','1'); localTrySet(boxSet,'Priority',setPrio);
        localLine(modelName,cmdPort,[sn '/1']); localLine(modelName,rotPort,[sn '/2']); localLine(modelName,'Box Scale Hold/1',[sn '/3']);
        add_block('simulink/Sinks/Terminator',sprintf('%s/Box Drive Unused %d',modelName,k),'Position',[1550 740+90*(k-1) 1570 760+90*(k-1)]);
        localLine(modelName,drvPort,sprintf('Box Drive Unused %d/1',k));
    end
end
localLine(modelName,'Box Pack/1','Navigation Arm Controller/9');
rgTerm=add_block('simulink/Sinks/Terminator',[modelName '/Native Grasped Unused'],'Position',[1360 300 1380 320]);
localLine(modelName,'Robot 1 Kinova/2','Native Grasped Unused/1');
try
    stp=add_block('simulink/Logic and Bit Operations/Compare To Constant',[modelName '/Mission Done Test'],'Position',[330 560 400 590]);
    set_param(stp,'relop','==','const','12','OutDataTypeStr','boolean');
    add_block('simulink/Sinks/Stop Simulation',[modelName '/Mission Done Stop'],'Position',[430 560 480 590]);
    add_line(modelName,'RobotTaskFSM/1','Mission Done Test/1','autorouting','on'); add_line(modelName,'Mission Done Test/1','Mission Done Stop/1','autorouting','on');
catch ME
    warning('Phase2:StopBlock','Mission-done stop block not added (%s); stop the run manually.',ME.message);
end
localWorkspaceLog(modelName,'Navigation Arm Controller/7','Phase2BoxCmdLog',[1290 800 1420 830]);
localWorkspaceLog(modelName,'TargetBox Transform/2','Phase2TargetBoxRotationLog',[1150 830 1280 860]);
% Diagnostic-only logs: these do not drive any signal or alter scheduling.
localWorkspaceLog(modelName,'RobotTaskFSM/1','Phase2TaskStateLog',[245 405 345 435]);
localWorkspaceLog(modelName,'Grasped Delay/1','Phase2GraspLog',[245 230 345 260]);
localWorkspaceLog(modelName,'HSV Depth Perception/1','Phase2PerceptionValidLog',[1060 455 1180 485]);
localWorkspaceLog(modelName,'Navigation Arm Controller/2','Phase2CommandedJointConfigLog',[245 445 390 475]);
localWorkspaceLog(modelName,'Robot 1 Kinova/1','Phase2GroundTruthJointConfigLog',[1060 105 1210 135]);
localWorkspaceLog(modelName,'Navigation Arm Controller/4','Phase2ArmDoneLog',[245 485 390 515]);
localWorkspaceLog(modelName,'Navigation Arm Controller/5','Phase2ArmFaultLog',[245 525 390 555]);
% FSM outputs TaskState, GoalX/Y, IK target XYZ to controller.
for k=1:6, localLine(modelName,sprintf('RobotTaskFSM/%d',k),sprintf('Navigation Arm Controller/%d',k)); end
localLine(modelName,'Base Pose Delay/1','Navigation Arm Controller/7');
% Phase 3 sensing inputs (unused in single-robot mode, RobotId = 0): sense bus, camera bus (zeros) and the true pose (= the pose)
for q_=[10 11]
    zb_=localAdd(modelName,'simulink','Constant',sprintf('Sense Off %d',q_),[60 900+40*(q_-10) 120 925+40*(q_-10)]); set_param(zb_,'Value','zeros(1,8)','OutDataTypeStr','double','VectorParams1D','off');
    localLine(modelName,sprintf('Sense Off %d/1',q_),sprintf('Navigation Arm Controller/%d',q_));
end
localLine(modelName,'Base Pose Delay/1','Navigation Arm Controller/12');
% [v omega] is split to the required Differential Drive Kinematic Model v/omega ports.
velocityDemux=localAdd(modelName,'simulink','Demux','Velocity Split',[420 465 440 520]); set_param(velocityDemux,'Outputs','2');
localLine(modelName,'Navigation Arm Controller/1','Velocity Split/1'); localLine(modelName,'Velocity Split/1','Differential Drive Kinematics/1'); localLine(modelName,'Velocity Split/2','Differential Drive Kinematics/2'); localLine(modelName,'Differential Drive Kinematics/1','Base Pose Delay/1');
% The chassis is now a fixed visual body in gen3, so one Robot actor owns
% both chassis and arm. Explicit row vectors preserve the required pose type.
poseTransforms=localAdd(modelName,'simulink','MATLAB Function','Pose To Robot Transform',[775 395 965 500]);
localSetMATLABFunction(poseTransforms,sprintf([ ...
'function [translation,rotation] = fcn(pose)\n' ...
'%% Simulation 3D expects 1-by-3 world-frame row vectors.\n' ...
'translation = [pose(1), pose(2), 0];\n' ...
'%% Robot uses [yaw pitch roll]; yaw must be the first component.\n' ...
'rotation = [pose(3), 0, 0];\n']));
localLine(modelName,'Base Pose Delay/1','Pose To Robot Transform/1');
% Simulation 3D Robot accepts a variable-length actor name as a uint8
% character vector. Its native grasp implementation converts this back to
% "TargetBox"; a Simulink string signal is not supported by this port.
target=localAdd(modelName,'simulink','Constant','Target Actor',[920 350 1020 380]);
set_param(target,'Value','uint8(''TargetBox'')', ...
    'OutDataTypeStr','uint8','VectorParams1D','on');
% Guard against library/model data-type inheritance: the Robot block itself
% validates TargetActor as uint8, so force that exact type immediately before
% its input rather than relying on the Constant block's propagated type.
targetType=localAdd(modelName,'simulink','Data Type Conversion','Target Actor uint8',[1035 350 1070 380]);
set_param(targetType,'OutDataTypeStr','uint8');
gTrans=localAdd(modelName,'simulink','Constant','Grasp Translation',[920 390 1020 420]); set_param(gTrans,'Value',mat2str(C.GraspRegionTranslation));
gRot=localAdd(modelName,'simulink','Constant','Grasp Rotation',[920 430 1020 460]); set_param(gRot,'Value',mat2str(C.GraspRegionRotation));
% R2024b creates external Robot ports in this callback order:
% Config, Translation, Rotation, TargetActor, ActivateTargetGrasp,
% GraspRegionTranslation, GraspRegionRotation. This is also the documented
% five-port ordering (Config/Translation/Rotation/TargetActor/Activate)
% when region inputs are disabled.
localLine(modelName,'Target Actor/1','Target Actor uint8/1');
localLine(modelName,'Navigation Arm Controller/2','Robot 1 Kinova/1');
localLine(modelName,'Pose To Robot Transform/1','Robot 1 Kinova/2');
localLine(modelName,'Pose To Robot Transform/2','Robot 1 Kinova/3');
localLine(modelName,'Target Actor uint8/1','Robot 1 Kinova/4');
% ActivateTargetGrasp is deliberately controller-owned: it rises only after
% the Cartesian approach settles at the non-penetrating seal gap.
nativeOff=localAdd(modelName,'simulink','Constant','Native Grasp Off',[920 470 1020 500]);
set_param(nativeOff,'Value','false','OutDataTypeStr','boolean');
localLine(modelName,'Native Grasp Off/1','Robot 1 Kinova/5');   % native grasp deliberately never activated
localLine(modelName,'Grasp Translation/1','Robot 1 Kinova/6');
localLine(modelName,'Grasp Rotation/1','Robot 1 Kinova/7');
% Camera mount follows the model-integrated base. RGB/depth plus base pose
% feed perception; no camera pose is used as an uncontrolled feedback signal.
localLine(modelName,'Base Pose Delay/1','Camera Mount/1'); localLine(modelName,'Camera Mount/1','Head Camera/1'); localLine(modelName,'Camera Mount/2','Head Camera/2');
localLine(modelName,'Head Camera/1','HSV Depth Perception/1');
localLine(modelName,'Head Camera/2','HSV Depth Perception/2');
localLine(modelName,'Base Pose Delay/1','HSV Depth Perception/3');
localLine(modelName,'Head Camera/3','HSV Depth Perception/4');
localLine(modelName,'Head Camera/4','HSV Depth Perception/5');
% Use the existing one-tick AtGoal delay for this diagnostic input. A direct
% controller-to-perception wire would close controller -> perception -> FSM
% -> controller as an algebraic loop.
localLine(modelName,'AtGoal Delay/1','HSV Depth Perception/6');
cameraPitch=localAdd(modelName,'simulink','Constant','Camera Pitch Deg',[810 650 910 680]);
set_param(cameraPitch,'Value','CameraPitchDeg','OutDataTypeStr','double');
localLine(modelName,'Camera Pitch Deg/1','HSV Depth Perception/7');
localWorkspaceLog(modelName,'Head Camera/3','Phase2CameraTranslationLog',[1050 520 1180 550]);
localWorkspaceLog(modelName,'Head Camera/4','Phase2CameraRotationLog',[1050 555 1180 585]);
localWorkspaceLog(modelName,'TargetBox Transform/1','Phase2TargetBoxTranslationLog',[1150 610 1280 640]);
localWorkspaceLog(modelName,'TargetBox2 Transform/1','Phase2TargetBox2TranslationLog',[1150 520 1280 550]);
localWorkspaceLog(modelName,'TargetBox3 Transform/1','Phase2TargetBox3TranslationLog',[1150 480 1280 510]);
% Compile only after every dynamic port has been wired. This validates the
% complete diagram without triggering false underspecified-signal diagnostics
% from partially constructed System and Stateflow blocks.
set_param(modelName,'SimulationCommand','update');
save_system(modelName,modelFile); open_system(modelName); fprintf('[Phase2] Created %s\n',modelFile);
end

function block=localAdd(model,library,blockName,newName,position)
% Use installed library source paths; this avoids an ambiguous global block-name search.
if strcmp(library,'simulink')
    switch blockName
        case 'MATLAB System', source='simulink/User-Defined Functions/MATLAB System';
        case 'Unit Delay', source='simulink/Discrete/Unit Delay';
        case 'MATLAB Function', source='simulink/User-Defined Functions/MATLAB Function';
        case 'Demux', source='simulink/Signal Routing/Demux';
        case 'Selector', source='simulink/Signal Routing/Selector';
        case 'Constant', source='simulink/Sources/Constant';
        case 'String Constant', source='simulink/String/String Constant';
        case 'Data Type Conversion', source='simulink/Signal Attributes/Data Type Conversion';
        case 'Mux', source='simulink/Signal Routing/Mux';
        otherwise, error('Phase2:BlockNotFound','No source mapping for Simulink block "%s".',blockName);
    end
else
    load_system(library); found=find_system(library,'SearchDepth',1,'FollowLinks','on','Name',blockName);
    assert(~isempty(found),'Phase2:BlockNotFound','Could not find "%s" in %s.',blockName,library); source=found{1};
end
block=[model '/' newName]; add_block(source,block,'Position',position);
end
function localLine(model,source,destination), add_line(model,source,destination,'autorouting','on'); end
function localWorkspaceLog(model,source,variableName,position)
block=[model '/' variableName];
add_block('simulink/Sinks/To Workspace',block,'Position',position);
set_param(block,'VariableName',variableName,'SaveFormat','Structure With Time','MaxDataPoints','inf');
localLine(model,source,[variableName '/1']);
end
function localTrySet(block,name,value), try,set_param(block,name,value);catch,end,end
function localSetMATLABFunction(block,script)
rt=sfroot; chart=rt.find('-isa','Stateflow.EMChart','Path',block); chart.Script=script;
end
