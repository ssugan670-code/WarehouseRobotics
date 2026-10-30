function ok = addRobotVisionView(mdl, varargin)
%ADDROBOTVISIONVIEW  Give the robot ONE forward-looking camera (replacing the old unused head camera) and an optional live window.
%
%   addRobotVisionView(mdl,'Mode','view')      live "Robot Vision" window            (default)
%   addRobotVisionView(mdl,'Mode','record')    live window + RobotVision_<time>.mp4
%   addRobotVisionView(mdl,'Mode','none')      no camera window (the old head camera is still retired)
%   'YawSign',-1    use only if the picture turns the wrong way when the robot turns (default +1)
%   'InvertPitch',true   use only if the camera looks UP instead of down
%
% What it does to the built model (the model file on disk is NOT saved with these changes; every run rebuilds it):
%   1. Verifies the old "Head Camera" pipeline (Head Camera, Camera Mount, HSV Depth Perception, Camera Pitch Deg,
%      HSV Unused 2-4, two camera To-Workspace logs) is a pure diagnostic: nothing in it reaches the controller,
%      the state machine or the robot. Then removes it - one camera on the robot is enough.
%   2. Adds "Robot Camera": rigid on the lift carriage (moves with the lift, never pans), mount from
%      helperVisionCameraMount - the same numbers that place the visible camera body in the 3-D scene.
%      It reads Base Pose Delay/1 and Navigation Arm Controller/2 (read-only) through a Unit Delay, so it creates
%      no data dependency on the controller.
%   3. Adds the viewer block (Mode view/record).
% Returns true when the model compiled in the requested state. On false the caller should rebuild the model
% (buildWarehouseRobotSim) - nothing about the robot logic has been touched either way.
ok=false; stage='start';
p=inputParser;
addParameter(p,'Mode','view');
addParameter(p,'YawSign',1);
addParameter(p,'InvertPitch',false);
addParameter(p,'Retire',true);   % false = leave the old (unused) head camera in place
parse(p,varargin{:});
mode=lower(char(p.Results.Mode)); yawSign=sign(p.Results.YawSign); if yawSign==0, yawSign=1; end
assert(any(strcmp(mode,{'view','record','none'})),'Mode must be view, record or none.');

retire={'Head Camera','Camera Mount','HSV Depth Perception','Camera Pitch Deg','HSV Unused 2','HSV Unused 3','HSV Unused 4', ...
        'Phase2CameraTranslationLog','Phase2CameraRotationLog','Phase2PerceptionValidLog'};
mine={'Robot Camera','Robot Camera Mount','Robot Camera Lift Delay','Robot Vision Viewer','Robot Vision Viewer End'};

try
    stage='check old camera';
    % ---- 1. prove the old camera pipeline is diagnostic-only, then retire it --------------------------------
    present=retire(cellfun(@(n) ~isempty(find_system(mdl,'SearchDepth',1,'Name',n)),retire));
    if ~p.Results.Retire, present={}; end
    for k=1:numel(present)
        b=[mdl '/' present{k}]; lh=get_param(b,'LineHandles');
        hs=lh.Outport(:).'; hs=hs(hs>0);
        for i=1:numel(hs)
            dst=get_param(hs(i),'DstBlockHandle'); dst=dst(:).'; dst=dst(dst>0);
            for j=1:numel(dst)
                nm=get_param(dst(j),'Name');
                assert(any(strcmp(nm,retire)),'Phase2:Vision','"%s" feeds "%s" - not a pure diagnostic, old camera kept.',present{k},nm);
            end
        end
    end
    stage='retire old camera';
    for k=1:numel(present), localRemoveBlock(mdl,present{k}); end
    localRemoveBlock(mdl,mine);

    % ---- 2. the one robot camera ---------------------------------------------------------------------------
    if ~strcmp(mode,'none')
        C=WarehouseRobotPhase2Config();
        m=helperVisionCameraMount(); stage='camera blocks'; pitch=m(4); if p.Results.InvertPitch, pitch=-pitch; end
        load_system('sim3dcameralib');
        src=find_system('sim3dcameralib','SearchDepth',1,'FollowLinks','on','Name','Simulation 3D Camera');
        assert(~isempty(src),'Simulation 3D Camera block not found.');
        y0=760;
        camB=[mdl '/' mine{1}]; add_block(src{1},camB,'Position',[900 y0 1060 y0+150]);
        set_param(camB,'Priority','1','vehTag','Scene Origin','offsetFlag','on','extTmount','on','extRmount','on', ...
            'ImageSize',mat2str(C.CameraImageSize),'FocalLength',mat2str([m(5) m(5)]),'OpticalCenter',mat2str(C.CameraPrincipalPoint));
        try, set_param(camB,'SampleTime',num2str(C.CameraSampleTime)); catch, end

        dl=[mdl '/' mine{3}]; add_block('simulink/Discrete/Unit Delay',dl,'Position',[460 y0+90 520 y0+120]);
        set_param(dl,'InitialCondition','zeros(8,1)','SampleTime',num2str(C.SampleTime));

        % camera position = base pose + carriage (x=0.10, z=0.47+lift) + mount offset; heading = robot heading.
        % The carriage is rigid on the chassis, so the camera yaw is exactly the robot yaw (never a pan).
        mt=[mdl '/' mine{2}]; add_block('simulink/User-Defined Functions/MATLAB Function',mt,'Position',[620 y0 790 y0+90]);
        code=sprintf([ ...
            'function [translation,rotation] = fcn(pose,q)\n' ...
            '%% Right-handed world, yaw counter-clockwise: same heading the Robot block uses (pose(3)).\n' ...
            'yaw=pose(3); c=cos(yaw); s=sin(yaw);\n' ...
            'f=%.6g; l=%.6g; z=%.6g+q(1);\n' ...
            'translation=[pose(1)+f*c-l*s pose(2)+f*s+l*c z];\n' ...
            'rotation=[0 %.6g %.6g*yaw*180/pi];\n'],0.10+m(1),m(2),0.47+m(3),pitch,yawSign);
        rt=sfroot; ch=rt.find('-isa','Stateflow.EMChart','Path',mt); ch.Script=code;

        add_line(mdl,'Navigation Arm Controller/2',[mine{3} '/1'],'autorouting','on');   % read-only tap (8x1 commanded config)
        add_line(mdl,[mine{3} '/1'],[mine{2} '/2'],'autorouting','on');
        add_line(mdl,'Base Pose Delay/1',[mine{2} '/1'],'autorouting','on');             % read-only tap
        add_line(mdl,[mine{2} '/1'],[mine{1} '/1'],'autorouting','on');
        add_line(mdl,[mine{2} '/2'],[mine{1} '/2'],'autorouting','on');

        % ---- 3. viewer ---------------------------------------------------------------------------------------
        stage='viewer'; rec='false'; if strcmp(mode,'record'), rec='true'; end
        vw=[mdl '/' mine{4}]; add_block('simulink/User-Defined Functions/MATLAB Function',vw,'Position',[1120 y0 1290 y0+80]);
        code2=sprintf([ ...
            'function ok = fcn(img,pose,q)\n' ...
            'coder.extrinsic(''robotVisionViewer'');\n' ...
            'robotVisionViewer(img,pose,q,%s);\n' ...
            'ok=uint8(0);\n'],rec);
        ch2=rt.find('-isa','Stateflow.EMChart','Path',vw); ch2.Script=code2;
        add_block('simulink/Sinks/Terminator',[mdl '/' mine{5}],'Position',[1340 y0+15 1360 y0+35]);
        add_line(mdl,[mine{1} '/1'],[mine{4} '/1'],'autorouting','on');
        add_line(mdl,'Base Pose Delay/1',[mine{4} '/2'],'autorouting','on');
        add_line(mdl,[mine{3} '/1'],[mine{4} '/3'],'autorouting','on');
        add_line(mdl,[mine{4} '/1'],[mine{5} '/1'],'autorouting','on');
    end

    stage='compile'; set_param(mdl,'SimulationCommand','update');     % full compile check of the final diagram
    ok=true;
    clear robotVisionViewer                           % fresh window / recorder for this run
    if p.Results.Retire, oldTxt='old head camera retired'; else, oldTxt='old head camera kept'; end
    fprintf('[VISION] %s; robot camera: %s.\n',oldTxt,mode);
catch err
    where=''; try, where=sprintf(' [%s:%d]',err.stack(1).name,err.stack(1).line); catch, end
    warning('Phase2:Vision','Camera setup failed at stage "%s"%s: %s',stage,where,err.message);
    try, for cc=1:numel(err.cause), fprintf('   cause: %s\n',err.cause{cc}.message); end, catch, end
    ok=false;
end
end

function localRemoveBlock(mdl,names)
if ischar(names), names={names}; end
for k=1:numel(names)
    try
        if ~isempty(find_system(mdl,'SearchDepth',1,'Name',names{k}))
            b=[mdl '/' names{k}]; lh=get_param(b,'LineHandles');
            ln=[lh.Inport(:); lh.Outport(:)]; ln=ln(ln>0);
            for i=1:numel(ln), try, delete_line(ln(i)); catch, end, end
            delete_block(b);
        end
    catch
    end
end
end
