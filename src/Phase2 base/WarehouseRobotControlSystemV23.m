classdef WarehouseRobotControlSystemV23 < matlab.System
%WAREHOUSEROBOTCONTROLSYSTEMV23  Closed-loop multi-carton pick-and-place on a lift-mounted Gen3 (Round 30)
%
% Round 30 changes
%   * 3 cartons on 3 shelves / 3 levels.  The mission queue comes from the base-workspace variable
%     Phase2MissionQueue ([2 3 1] = that order, 'auto' = nearest first).  A carton that is not in its
%     slot when the robot has scanned it is reported absent and skipped (no endless loop).
%   * Arm = prismatic lift column + Gen3 (config is 8x1 = [lift; q1..q7]).
%   * After the lift the hand is levelled (tool axis and tool X horizontal) and stays level while carrying.
%   * Navigation: rotate in place, then drive straight; larger switch radius, lookahead-free corner slowing,
%     arrival latch (no back-and-forth), wider obstacle inflation while carrying.
%   * Reverse distance grows with the swing radius of the held carton, so the turn happens clear of the rack.
%   * Whole-arm (dense link samples + fingers + held carton) collision checks for approach, level, carry,
%     place and stow; stow is Cartesian (back, up) then joint space, all checked.
%   * Contact emulation: any carton touched by the chassis, an arm link, a finger or the held carton is pushed
%     horizontally (capped step), stopped by static geometry, and falls under gravity if unsupported.
%
% State ids: 0 IDLE, 1 NAV_TO_STAGING, 10 DOCK_SHELF, 2 ARM_EXTEND, 3 GRASP, 4 ARM_RETRACT, 5 NAV_TO_TABLE,
%            6 ARM_PLACE, 8 RELEASE, 9 ARM_STOW, 7 RETURN_HOME, 11 FAULT_HOLD, 12 MISSION_DONE
properties (Nontunable)
    SampleTime=0.02
end
properties (Access=private)
    C
    Robot
    IKRobot
    IK
    IKPin
    EEName
    StowConfig
    JointLimits
    ShoulderLocal
    LinkNames
    Obstacles
    CurrentConfig
    Trajectory
    TrajectoryIndex
    TrajB
    QPre
    PreLocal
    HeldOffset
    HeldAxisLocal
    RLevelLocal
    Waypoints
    WaypointIndex
    PlannedGoal
    PreviousState
    Plan
    Phase
    PhaseStartTime
    VacuumStartTime
    ArmFaultLatched
    DebugTime
    DebugNextNavPrintTime
    DockStableTicks
    ReverseStart
    ReverseDist
    LastGrasped
    BoxAttached
    BoxLast
    BoxSeenW
    CmdHist
    AttachBoxZ
    NextAttachPrint
    Placed
    BoxYaw
    LastBaseYaw
    BoxVz
    ContactPrint
    PhysHandoff
    PhysDone
    PhysT0
    PhysRelease
    SeenHist
    PhysPrint
    % ---- Round 30 mission / multi-box state
    Sel
    Queue
    BoxPos
    BoxYawV
    BoxVzV
    SeenW
    OwnPhys
    Moved
    BoxDone
    BoxAbsent
    MoreBoxes
    ScanTicks
    NavMode
    NavLatch
    ArmCache
    PushPrint
    LiftTarget
    StowTrajB
    StowReverseStart
    DockShift
    % ---- Round 31
    IKWide
    IKRobotWide
    IKPinWide
    CarryLift
    CarryWide
    CarryQ
    CarryS
    Pending
    AutoMode
    Finish
    CmdSeq
    SkipRungs
    StatusTick
    Dest
    StationCount
    BoxStation
end
methods (Access=protected)
    function setupImpl(obj)
        obj.C=localDefaults(WarehouseRobotPhase2Config());
        [obj.Robot,qs]=helperBuildMobileManipulator();
        obj.StowConfig=reshape(double(qs),8,1);
        obj.EEName='ToolTip';
        obj.JointLimits=localJointBounds(obj.Robot);
        obj.IKRobot=localNarrowBranch(obj.Robot,obj.StowConfig,obj.C.BranchWindow);
        obj.IKPin=NaN; obj.IK=[]; obj.IKWide=false; obj.IKPinWide=false;
        obj.IKRobotWide=localNarrowBranch(obj.Robot,obj.StowConfig,obj.C.BranchWindowWide);
        obj.CarryLift=0; obj.CarryWide=false; obj.CarryQ=[]; obj.CarryS=[]; obj.Finish=false; obj.CmdSeq=0; obj.StatusTick=0; obj.SkipRungs=0;
        try, obj.SkipRungs=evalin('base','Phase2TestSkipLadder'); catch, end
        names={}; k0=0;
        for k=1:numel(obj.Robot.Bodies)
            if ~strcmp(obj.Robot.Bodies{k}.Joint.Type,'fixed')
                k0=k0+1;
                if k0==1, names{end+1}=obj.Robot.Bodies{k}.Name; end %#ok<AGROW>  % lift carriage = start of the arm
                if k0==2, shoulderName=obj.Robot.Bodies{k}.Name; end
                if k0>=2, names{end+1}=obj.Robot.Bodies{k}.Name; end %#ok<AGROW>
            end
        end
        names{end+1}='EndEffector_Link';
        obj.LinkNames=names;
        T=getTransform(obj.Robot,obj.StowConfig,shoulderName); obj.ShoulderLocal=T(1:3,4).';
        obj.Obstacles=localObstacleBoxes(obj.C);
        obj.CurrentConfig=obj.StowConfig; obj.Trajectory=obj.StowConfig; obj.TrajectoryIndex=1; obj.TrajB=[];
        obj.QPre=obj.StowConfig; obj.PreLocal=[0 0 0]; obj.HeldOffset=[]; obj.HeldAxisLocal=[1 0 0]; obj.RLevelLocal=[];
        obj.Waypoints=zeros(0,2); obj.WaypointIndex=1; obj.PlannedGoal=[NaN NaN]; obj.PreviousState=uint8(255);
        obj.Phase='IDLE'; obj.PhaseStartTime=0; obj.VacuumStartTime=0;
        obj.ArmFaultLatched=false; obj.DebugTime=0; obj.DebugNextNavPrintTime=0; obj.DockStableTicks=0;
        obj.ReverseStart=[0 0]; obj.ReverseDist=obj.C.ReverseDistance; obj.LastGrasped=false;
        obj.BoxAttached=false; obj.BoxLast=[NaN NaN NaN]; obj.BoxSeenW=[NaN NaN NaN]; obj.CmdHist=nan(6,3);
        obj.AttachBoxZ=0; obj.NextAttachPrint=0; obj.Placed=false; obj.BoxYaw=0; obj.LastBaseYaw=0; obj.BoxVz=0;
        obj.ContactPrint=0; obj.PhysHandoff=false; obj.PhysDone=false; obj.PhysT0=0; obj.PhysRelease=[NaN NaN NaN];
        obj.SeenHist=nan(26,3); obj.PhysPrint=0;
        obj.Plan=struct('box',nan(1,3),'face',nan(1,3),'contact',nan(1,3),'pre',nan(1,3),'touch',nan(1,3),'axis',nan(1,3),'nh',nan(1,3),'phi',nan);
        % ---- cartons
        nb=numel(obj.C.Boxes); obj.BoxPos=zeros(nb,3); obj.BoxYawV=zeros(nb,1); obj.BoxVzV=zeros(nb,1);
        for k=1:nb
            p=obj.C.Boxes(k).xyz(:).';
            if any(k==obj.C.Test.MissingBox), p=[obj.C.Test.MissingXY 0.5*obj.C.TargetDimensions(3)+0.001]; end
            obj.BoxPos(k,:)=p;
        end
        obj.SeenW=nan(nb,3); obj.OwnPhys=false(nb,1); obj.Moved=false(nb,1); obj.BoxDone=false(nb,1); obj.BoxAbsent=false(nb,1);
        obj.Sel=0; obj.MoreBoxes=true; obj.ScanTicks=0; obj.NavMode='ROTATE'; obj.NavLatch=false; obj.ArmCache=struct('q',[],'P',[]); obj.PushPrint=0; obj.LiftTarget=0; obj.StowTrajB=[]; obj.StowReverseStart=[0 0]; obj.DockShift=0;
        % The stowed hand sticks out in front of the chassis; keep it clear of the rack face when docking.
        reach=max(obj.localFingerPts(obj.StowConfig,[0 0 0])*[1;0;0]);
        dockDist=obj.C.BaseRadius+obj.C.ChassisFrontClearance; obj.DockShift=max(0,obj.C.StowRackGap-(dockDist-reach));
        fprintf('[STOWCHK] stowed hand reaches %.3f m ahead of the base axis; dock distance %.3f m -> gap to rack %.3f m, dock moved back %.3f m\n',reach,dockDist,dockDist-reach+obj.DockShift,obj.DockShift);
        q=[];
        try, q=evalin('base','Phase2MissionQueue'); catch, end
        obj.AutoMode=false; obj.Pending=zeros(1,0);
        if ischar(q) || isstring(q)
            if strcmpi(char(q),'auto'), obj.AutoMode=true; obj.Pending=1:nb; end
        elseif ~isempty(q)
            q=reshape(double(q),1,[]); obj.Pending=q(q>=1 & q<=nb);
        end
        ns=numel(obj.C.Stations); obj.StationCount=zeros(1,ns); obj.BoxStation=ones(1,nb); obj.Dest=[];
        try
            sv=reshape(double(evalin('base','Phase2MissionStations')),1,[]);
            if isscalar(sv), sv=repmat(sv,1,nb); end
            if numel(sv)==nb && all(sv>=1 & sv<=ns & sv==round(sv)), obj.BoxStation=sv; end
        catch
        end
        for k=1:nb, fprintf('[STATIONS] box %d -> %s\n',k,obj.C.Stations(obj.BoxStation(k)).name); end
        obj.Finish=~isempty(obj.Pending);   % order given up front: do it, go home, stop
        try, evalin('base','Phase2MissionCmd=struct(''seq'',0,''box'',0,''finish'',false,''station'',0);'); catch, end
        obj.localPublish('START');
        if obj.AutoMode, fprintf('[MISSION] start queue = auto (nearest first)\n'); else, fprintf('[MISSION] start queue = %s (more boxes can be added live with pickBox(k); endMission ends)\n',mat2str(obj.Pending)); end
    end
    function [velocity,config,atGoal,armDone,armFault,vacuum,c1,c2,c3,r1,r2,r3,d1,d2,d3,percValid,moreBoxes,scanDone,finish,needHome]=stepImpl(obj,state,~,~,~,~,~,basePose,isTargetGrasped,boxMeas)
        state=uint8(state); basePose=reshape(double(basePose),1,3); grasped=logical(isTargetGrasped);
        bm=reshape(double(boxMeas),3,[]).';          % one row per carton, Unreal frame (y negated)
        for k=1:size(bm,1), obj.SeenW(k,:)=[bm(k,1) -bm(k,2) bm(k,3)]; end
        if obj.Sel>0, obj.BoxSeenW=obj.SeenW(obj.Sel,:); else, obj.BoxSeenW=[NaN NaN NaN]; end
        obj.DebugTime=obj.DebugTime+obj.SampleTime; velocity=[0 0]; atGoal=false; armDone=false; armFault=false;
        obj.localPollCommands(state,basePose);
        if state~=obj.PreviousState
            fprintf('[STATE] state=%d start=%.2f s box=%d\n',state,obj.DebugTime,obj.Sel);
            obj.localEnter(state,basePose);
        end
        if grasped~=obj.LastGrasped
            fprintf('[GRASPSIG] t=%.2f attachedFlag=%d phase=%s\n',obj.DebugTime,grasped,obj.Phase); obj.LastGrasped=grasped;
        end
        if state==0 && obj.Sel==0 && ~isempty(obj.localCandidates()), obj.localSelectNext(basePose); end
        if (obj.Sel>0 && any(state==uint8([1 5 10]))) || state==7
            goal=obj.localGoal(state);
            [velocity,atGoal]=obj.localNavigate(state,goal,basePose);
        end
        percValid=false; scanDone=false;
        if state==1 && obj.Sel>0
            if atGoal, obj.ScanTicks=obj.ScanTicks+1; else, obj.ScanTicks=0; end
            scanDone=obj.ScanTicks>=round(obj.C.ScanDwell/obj.SampleTime);
            percValid=obj.localDetect(basePose);
            if scanDone && obj.ScanTicks==round(obj.C.ScanDwell/obj.SampleTime)
                fprintf('[SCAN] t=%.2f box %d %s (seen=[%.3f %.3f %.3f] slot=[%.3f %.3f %.3f])\n',obj.DebugTime,obj.Sel,localTern(percValid,'FOUND','NOT FOUND'),obj.BoxSeenW,obj.C.Boxes(obj.Sel).xyz);
            end
        elseif state==1
            percValid=false;
        end
        switch state
            case 10
                if atGoal, obj.DockStableTicks=obj.DockStableTicks+1; else, obj.DockStableTicks=0; end
                if strcmp(obj.Phase,'DOCK.WAIT') && obj.DockStableTicks>=15
                    if obj.LiftTarget>0.002
                        obj.Trajectory=obj.localJointPath(obj.CurrentConfig,[obj.LiftTarget;obj.CurrentConfig(2:end)],max(1.0,1.2*obj.LiftTarget/obj.C.LiftSpeed)); obj.TrajectoryIndex=1;
                        fprintf('[LIFT] raising arm column to %.2f m for box %d (level %d)\n',obj.LiftTarget,obj.Sel,obj.C.Boxes(obj.Sel).level);
                        obj.localPhase('DOCK.LIFT');
                    else
                        armDone=true;
                    end
                elseif strcmp(obj.Phase,'DOCK.LIFT')
                    obj.localAdvance(); armDone=atGoal && obj.localTrajDone();
                end
            case 2
                if strcmp(obj.Phase,'EXTEND.PLAN') && ~obj.ArmFaultLatched, obj.localPlanApproach(obj.BoxSeenW,basePose); end
                if strcmp(obj.Phase,'ARM_EXTEND'), obj.localAdvance(); armDone=obj.localTrajDone(); end
            case 3
                armDone=obj.localGrasp(basePose);
            case 4
                [velocity,armDone]=obj.localRetract(basePose);
            case 6
                if strcmp(obj.Phase,'PLACE.PLAN') && ~obj.ArmFaultLatched, obj.localPlanPlace(obj.Dest.slotXYZ,basePose); end
                if strcmp(obj.Phase,'ARM_PLACE'), obj.localAdvance(); armDone=obj.localTrajDone(); end
            case 8
                obj.BoxAttached=false; obj.HeldOffset=[];
            case 9
                if strcmp(obj.Phase,'STOW.PLAN'), obj.localPlanStow(basePose); end
                [velocity,armDone]=obj.localStow(basePose);
            case 11
                armFault=true;
            case 12
                % MISSION_DONE: base and arm hold still
        end
        if obj.ArmFaultLatched, armFault=true; end
        [c1,c2,c3,r1,r2,r3,d1,d2,d3]=obj.localBoxUpdate(state,basePose);
        config=obj.CurrentConfig; vacuum=logical(obj.BoxAttached); moreBoxes=~isempty(obj.localCandidates()) || (obj.Sel>0 && ~obj.BoxDone(obj.Sel) && ~obj.BoxAbsent(obj.Sel)); finish=logical(obj.Finish); needHome=norm(basePose(1:2)-obj.C.HomeXY)>0.5; obj.MoreBoxes=moreBoxes; obj.PreviousState=state;
        if obj.DebugTime>=obj.StatusTick, obj.localPublish(''); end
    end
    function n=getNumInputsImpl(~), n=9; end
    function n=getNumOutputsImpl(~), n=20; end
    function [a,b,c,d,e,f,g,h,i]=getInputSizeImpl(~), a=[1 1];b=[1 1];c=[1 1];d=[1 1];e=[1 1];f=[1 1];g=[1 3];h=[1 1];i=[1 9];end
    function [a,b,c,d,e,f,g,h,i]=getInputDataTypeImpl(~), a='uint8';b='double';c='double';d='double';e='double';f='double';g='double';h='boolean';i='double';end
    function [a,b,c,d,e,f,g,h,i]=isInputComplexImpl(~), a=false;b=false;c=false;d=false;e=false;f=false;g=false;h=false;i=false;end
    function [a,b,c,d,e,f,g,h,i]=isInputFixedSizeImpl(~), a=true;b=true;c=true;d=true;e=true;f=true;g=true;h=true;i=true;end
    function varargout=getOutputSizeImpl(~)
        s={[1 2],[8 1],[1 1],[1 1],[1 1],[1 1],[1 3],[1 3],[1 3],[1 3],[1 3],[1 3],[1 1],[1 1],[1 1],[1 1],[1 1],[1 1],[1 1],[1 1]};
        varargout=s;
    end
    function varargout=getOutputDataTypeImpl(~)
        s={'double','double','boolean','boolean','boolean','boolean','double','double','double','double','double','double','boolean','boolean','boolean','boolean','boolean','boolean','boolean','boolean'};
        varargout=s;
    end
    function varargout=isOutputComplexImpl(~), varargout=repmat({false},1,20); end
    function varargout=isOutputFixedSizeImpl(~), varargout=repmat({true},1,20); end
end
methods (Access=private)
    % ------------------------------------------------------------ mission logic
    function localPollCommands(obj,state,pose) %#ok<INUSD>
        % Live operator commands: base variable Phase2MissionCmd = struct(seq,box,finish) written by pickBox / endMission.
        if mod(round(obj.DebugTime/obj.SampleTime),10)~=0, return, end
        try, c=evalin('base','Phase2MissionCmd'); catch, return, end
        if ~isstruct(c) || ~isfield(c,'seq') || c.seq<=obj.CmdSeq, return, end
        obj.CmdSeq=c.seq;
        if c.finish
            obj.Finish=true;
            fprintf('[COMMAND] t=%.2f END received: finish the queued boxes %s, return home and stop\n',obj.DebugTime,mat2str(obj.Pending));
        else
            k=c.box;
            if k<1 || k>numel(obj.C.Boxes) || k~=round(k)
                fprintf('[COMMAND] t=%.2f box %g does not exist (valid: 1..%d)\n',obj.DebugTime,k,numel(obj.C.Boxes));
            elseif obj.BoxDone(k)
                fprintf('[COMMAND] t=%.2f box %d is already on the table - ignored\n',obj.DebugTime,k);
            elseif obj.BoxAbsent(k)
                fprintf('[COMMAND] t=%.2f box %d was reported absent at its slot - ignored\n',obj.DebugTime,k);
            elseif k==obj.Sel && any(obj.PreviousState==uint8([1 10 2 3 4 5 6 8 9]))
                fprintf('[COMMAND] t=%.2f box %d is the box being handled now - ignored\n',obj.DebugTime,k);
            elseif any(obj.Pending==k)
                fprintf('[COMMAND] t=%.2f box %d is already queued\n',obj.DebugTime,k);
            else
                stn=obj.BoxStation(k); if isfield(c,'station') && c.station>=1 && c.station<=numel(obj.C.Stations), stn=c.station; end
                obj.BoxStation(k)=stn; obj.Pending(end+1)=k; obj.Finish=false;
                fprintf('[COMMAND] t=%.2f NEW box %d accepted (state %d), deliver to %s -> queue %s\n',obj.DebugTime,k,obj.PreviousState,obj.C.Stations(stn).name,mat2str(obj.Pending));
            end
        end
    end
    function localPublish(obj,tag)
        obj.StatusTick=obj.DebugTime+0.5;
        % goal / station are read-only extras for the robot-vision window (a failure here must never reach the controller)
        goal=[]; try, goal=obj.localGoal(double(obj.PreviousState)); catch, end
        stn=0; try, if obj.Sel>0, stn=obj.BoxStation(obj.Sel); end, catch, end
        st=struct('time',obj.DebugTime,'state',double(obj.PreviousState),'sel',obj.Sel,'done',find(obj.BoxDone).','absent',find(obj.BoxAbsent).', ...
            'pending',obj.Pending,'finish',obj.Finish,'tag',tag,'stationsUsed',obj.StationCount,'goal',goal,'station',stn);
        try, assignin('base','Phase2MissionStatus',st); catch, end
    end
    function goal=localGoal(obj,state)
        if state==7 || obj.Sel==0, goal=reshape(obj.C.HomeXY,1,2); return, end
        b=obj.C.Boxes(obj.Sel);
        switch state
            case 1, goal=b.staging;
            case 10, goal=b.dock+[obj.DockShift 0];
            case 5, goal=obj.Dest.approach;
            otherwise, goal=obj.C.HomeXY;
        end
        goal=reshape(goal,1,2);
    end
    function r=localCandidates(obj)
        r=obj.Pending(~obj.BoxDone(obj.Pending) & ~obj.BoxAbsent(obj.Pending));
    end
    function localSelectNext(obj,pose)
        rem=obj.localCandidates(); nxt=0;
        if ~isempty(rem)
            if obj.AutoMode
                d=arrayfun(@(k) norm(obj.C.Boxes(k).staging-pose(1:2)),rem); [~,i]=min(d); nxt=rem(i);
            else
                nxt=rem(1);
            end
            obj.Pending(obj.Pending==nxt)=[];
            stn=obj.BoxStation(nxt); j=min(3,obj.StationCount(stn)+1); S=obj.C.Stations(stn);
            obj.Dest=struct('station',stn,'slot',j,'name',S.name,'yaw',S.yaw,'slotXYZ',S.slotXYZ(j,:),'approach',S.approach(j,:));
        end
        obj.Sel=nxt; obj.MoreBoxes=~isempty(obj.localCandidates());
        if nxt>0
            obj.BoxLast=obj.BoxPos(nxt,:); obj.BoxYaw=obj.BoxYawV(nxt); obj.BoxVz=obj.BoxVzV(nxt);
            obj.BoxAttached=false; obj.HeldOffset=[]; obj.RLevelLocal=[]; obj.TrajB=[]; obj.Placed=false; obj.PhysHandoff=false; obj.PhysDone=false;
            obj.LiftTarget=obj.C.Boxes(nxt).lift; obj.ScanTicks=0; obj.NavLatch=false;
            b=obj.C.Boxes(nxt);
            fprintf('[MISSION] t=%.2f next = box %d (shelf %d, level %d, lift %.2f m) -> %s slot %d; done=%s absent=%s\n',obj.DebugTime,nxt,b.shelf,b.level,b.lift,obj.Dest.name,obj.Dest.slot,mat2str(find(obj.BoxDone).'),mat2str(find(obj.BoxAbsent).'));
        else
            fprintf('[MISSION] t=%.2f no boxes left to pick; done=%s absent=%s\n',obj.DebugTime,mat2str(find(obj.BoxDone).'),mat2str(find(obj.BoxAbsent).'));
        end
    end
    function ok=localDetect(obj,pose)
        % Scan result for the selected carton: it must be at its slot (readback), inside sensor range and inside the camera field of view.
        s=obj.BoxSeenW; slot=obj.C.Boxes(obj.Sel).xyz; ok=false;
        if ~all(isfinite(s)), return, end
        rng=norm(s(1:2)-pose(1:2)); brg=abs(localWrap(atan2(s(2)-pose(2),s(1)-pose(1))-pose(3)));
        ok = norm(s(1:2)-slot(1:2))<0.35 && abs(s(3)-slot(3))<0.20 && rng<=obj.C.Perception.SimRange && brg<=deg2rad(obj.C.DetectFovHalfDeg);
    end
    % ------------------------------------------------------- box actor command
    function [c1,c2,c3,r1,r2,r3,d1,d2,d3]=localBoxUpdate(obj,state,pose)
        dyaw=localWrap(pose(3)-obj.LastBaseYaw); obj.LastBaseYaw=pose(3); sel=obj.Sel;
        if sel>0
            if obj.BoxAttached && ~isempty(obj.HeldOffset)
                if ~strcmp(obj.Phase,'RETRACT.LEVEL')     % while the hand levels, the carton stays where it is (clamped, base standing still)
                    tip=obj.localTipWorld(obj.CurrentConfig,pose);
                    obj.BoxLast=localB2W(localW2B(tip,pose)+obj.HeldOffset,pose);
                end
                obj.BoxYaw=obj.BoxYaw+dyaw; obj.BoxVz=0;
                [obj.BoxLast,pen]=obj.localResolveContact(obj.BoxLast,obj.BoxYaw);
                if pen>0.002 && obj.DebugTime>=obj.ContactPrint
                    fprintf('[CONTACT] t=%.2f held box pushed out of static geometry by %.3f m (clamp slips)\n',obj.DebugTime,pen); obj.ContactPrint=obj.DebugTime+1;
                end
            elseif ~obj.BoxAttached && obj.Placed && ~obj.C.UnrealPhysics
                [~,hz]=obj.localBoxHalf(obj.BoxYaw); sup=obj.localSupportZ(obj.BoxLast,obj.BoxYaw); restZ=sup+hz+0.0005;
                if obj.BoxLast(3)>restZ+1e-6 || obj.BoxVz~=0
                    obj.BoxLast(3)=obj.localFall(obj.BoxLast(3),restZ);
                    if obj.BoxLast(3)<=restZ && obj.BoxVz==0
                        obj.Placed=false;
                        fprintf('[PLACED] t=%.2f box %d SETTLED at [%.3f %.3f %.3f] bottomAboveSupport=%.4f yaw=%.1fdeg\n',obj.DebugTime,sel,obj.BoxLast,obj.BoxLast(3)-hz-sup,rad2deg(obj.BoxYaw));
                    end
                end
            end
            obj.CmdHist=[obj.BoxLast;obj.CmdHist(1:end-1,:)];
            if obj.BoxAttached && obj.DebugTime>=obj.NextAttachPrint
                tip=obj.localTipWorld(obj.CurrentConfig,pose); err=min(sqrt(sum((obj.CmdHist-obj.BoxSeenW).^2,2)));
                fprintf('[ATTACH] t=%.2f box=%d cmd=[%.3f %.3f %.3f] seen=[%.3f %.3f %.3f] tip=[%.3f %.3f %.3f] followErr=%.3f yaw=%.1fdeg\n', ...
                    obj.DebugTime,sel,obj.BoxLast,obj.BoxSeenW,tip,err,rad2deg(obj.BoxYaw));
                obj.NextAttachPrint=obj.DebugTime+2;
            end
            if state==8 && obj.DebugTime>=obj.NextAttachPrint
                fprintf('[PLACED] t=%.2f box %d=[%.3f %.3f %.3f] seen=[%.3f %.3f %.3f] bottomAboveTableSurface=%.4f\n',obj.DebugTime,sel,obj.BoxLast,obj.BoxSeenW,obj.BoxLast(3)-0.5*obj.C.TargetDimensions(3)-obj.C.TableSurfaceZ);
                obj.NextAttachPrint=obj.DebugTime+0.5;
            end
            % physics hand-over watch for the carton that was just released
            if obj.PhysHandoff
                [~,hz]=obj.localBoxHalf(obj.BoxYaw); sup=obj.localSupportZ(obj.PhysRelease,obj.BoxYaw); tt=obj.DebugTime-obj.PhysT0;
                obj.SeenHist=[obj.BoxSeenW;obj.SeenHist(1:end-1,:)]; rel=obj.PhysRelease; seen=obj.BoxSeenW;
                spd=norm(obj.SeenHist(1,:)-obj.SeenHist(end,:))/0.5;
                bad=tt>0.3 && (seen(3)<sup+hz-0.03 || norm(seen(1:2)-rel(1:2))>0.5 || seen(3)>rel(3)+0.15);
                if obj.DebugTime>=obj.PhysPrint && ~obj.PhysDone
                    fprintf('[PHYS] t=%.2f box %d seen=[%.3f %.3f %.3f] speed=%.3f m/s supportZ=%.3f\n',obj.DebugTime,sel,seen,spd,sup); obj.PhysPrint=obj.DebugTime+1;
                end
                if bad
                    fprintf('[PHYS] FAIL t=%.2f box %d left the valid region (seen z=%.3f, need >= %.3f): Transform Set takes over and rests it on the support\n',obj.DebugTime,sel,seen(3),sup+hz-0.03);
                    obj.PhysHandoff=false; obj.OwnPhys(sel)=false; obj.BoxLast=[rel(1:2) sup+hz+0.0005];
                elseif ~obj.PhysDone && tt>1.0 && isfinite(spd) && spd<0.01
                    obj.PhysDone=true; obj.BoxLast=seen; fprintf('[PLACED] t=%.2f box %d SETTLED by Unreal physics at [%.3f %.3f %.3f] bottomAboveSupport=%.4f yaw=%.1fdeg\n',obj.DebugTime,sel,seen,seen(3)-hz-sup,rad2deg(obj.BoxYaw));
                elseif ~obj.PhysDone && tt>8
                    obj.PhysDone=true; obj.BoxLast=seen; fprintf('[PHYS] t=%.2f box %d still moving after 8 s; leaving it to physics\n',obj.DebugTime,sel);
                end
            end
            obj.BoxPos(sel,:)=obj.BoxLast; obj.BoxYawV(sel)=obj.BoxYaw; obj.BoxVzV(sel)=obj.BoxVz;
            if obj.PhysHandoff, obj.OwnPhys(sel)=true; end
        end
        for k=1:size(obj.BoxPos,1)      % cartons owned by Unreal physics: the read-back is the truth
            if k~=sel && obj.OwnPhys(k) && all(isfinite(obj.SeenW(k,:))) && any(obj.SeenW(k,:)~=0), obj.BoxPos(k,:)=obj.SeenW(k,:); end
        end
        obj.localContactEmulation(state,pose);
        if sel>0 && ~obj.BoxAttached, obj.BoxLast=obj.BoxPos(sel,:); end
        drive=~obj.OwnPhys;
        C=cell(1,3); R=cell(1,3);
        for k=1:3
            C{k}=[obj.BoxPos(k,1) -obj.BoxPos(k,2) obj.BoxPos(k,3)];     % Unreal frame (y negated)
            R{k}=[0 0 obj.C.BoxYawSign*obj.BoxYawV(k)];
        end
        c1=C{1}; c2=C{2}; c3=C{3}; r1=R{1}; r2=R{2}; r3=R{3}; d1=drive(1); d2=drive(2); d3=drive(3);
    end
    function z=localFall(obj,z0,restZ)
        vz=obj.BoxVz-9.81*obj.SampleTime; z=z0+vz*obj.SampleTime;
        if z<=restZ, z=restZ; vz=-0.15*vz; if abs(vz)<0.08, vz=0; end, end
        obj.BoxVz=vz;
    end
    % ------------------------------------------------------- contact emulation (B)
    function localContactEmulation(obj,state,pose)
        % Any carton that is not under the controller's own planned manipulation is a free body: the chassis, arm links,
        % fingers and the held carton push it sideways (capped step, stopped by static geometry); it falls if unsupported.
        nb=size(obj.BoxPos,1); sel=obj.Sel; C=obj.C;
        if any(state==uint8([12 11])), return, end
        planned = sel>0 && (obj.BoxAttached || any(state==uint8([1 10 2 3])));
        active=false(nb,1);
        for k=1:nb
            if k==sel && planned, continue, end
            if k==sel && obj.PhysHandoff, continue, end
            if hypot(obj.BoxPos(k,1)-pose(1),obj.BoxPos(k,2)-pose(2))<2.4, active(k)=true; end
        end
        if any(active)
            pts=obj.localPusherPoints(pose);
            for k=find(active).'
                [hxy,hz]=obj.localBoxHalf(obj.BoxYawV(k)); ctr=obj.BoxPos(k,:); moved=false;
                for it=1:3
                    bb=[ctr(1)-hxy(1) ctr(1)+hxy(1) ctr(2)-hxy(2) ctr(2)+hxy(2) ctr(3)-hz ctr(3)+hz];
                    [push,who]=localPushVector(pts,bb,ctr);
                    if all(push==0), break, end
                    step=max(-0.02,min(0.02,push)); ctr(1:2)=ctr(1:2)+step; moved=true;
                    if obj.DebugTime>=obj.PushPrint
                        fprintf('[CONTACT] t=%.2f box %d pushed by %s: dx=[%.3f %.3f]\n',obj.DebugTime,k,who,step); obj.PushPrint=obj.DebugTime+0.5;
                    end
                end
                if obj.BoxAttached && sel>0 && k~=sel
                    held=obj.BoxPos(sel,:); [hh,hhz]=obj.localBoxHalf(obj.BoxYawV(sel));
                    ob=[held(1)-hh(1) held(1)+hh(1) held(2)-hh(2) held(2)+hh(2) held(3)-hhz held(3)+hhz];
                    bb=[ctr(1)-hxy(1) ctr(1)+hxy(1) ctr(2)-hxy(2) ctr(2)+hxy(2) ctr(3)-hz ctr(3)+hz];
                    ox=min(bb(2)-ob(1),ob(2)-bb(1)); oy=min(bb(4)-ob(3),ob(4)-bb(3)); oz=min(bb(6)-ob(5),ob(6)-bb(5));
                    if ox>0 && oy>0 && oz>0
                        if ox<oy, ctr(1)=ctr(1)+sign(ctr(1)-held(1)+1e-9)*min(ox,0.02); else, ctr(2)=ctr(2)+sign(ctr(2)-held(2)+1e-9)*min(oy,0.02); end
                        moved=true; if obj.DebugTime>=obj.PushPrint, fprintf('[CONTACT] t=%.2f held box hit box %d\n',obj.DebugTime,k); obj.PushPrint=obj.DebugTime+0.5; end
                    end
                end
                if moved
                    [ctr,~]=obj.localResolveContact(ctr,obj.BoxYawV(k));
                    if obj.OwnPhys(k)
                        fprintf('[CONTACT] t=%.2f box %d touched: controller takes the carton from Unreal physics\n',obj.DebugTime,k);
                        obj.OwnPhys(k)=false; ctr(3)=obj.SeenW(k,3); if k==sel, obj.PhysDone=true; obj.PhysHandoff=false; end
                    end
                    obj.Moved(k)=true;
                end
                if obj.Moved(k) && ~obj.OwnPhys(k)
                    sup=obj.localSupportZ(ctr,obj.BoxYawV(k)); restZ=sup+hz+0.0005;
                    if ctr(3)>restZ+1e-6 || obj.BoxVzV(k)~=0
                        vz=obj.BoxVzV(k)-9.81*obj.SampleTime; z=ctr(3)+vz*obj.SampleTime;
                        if z<=restZ, z=restZ; vz=-0.15*vz; if abs(vz)<0.08, vz=0; end, end
                        ctr(3)=z; obj.BoxVzV(k)=vz;
                    end
                end
                obj.BoxPos(k,:)=ctr;
            end
        end
    end
    function P=localPusherPoints(obj,pose)
        % [x y z r] rows in world frame: chassis ring, arm link samples, finger pads.
        R=obj.C.BaseRadius; a=linspace(0,2*pi,13); a(end)=[];
        ring=[]; for z=[0.12 0.30 0.44], ring=[ring; pose(1)+R*cos(a(:)) pose(2)+R*sin(a(:)) z*ones(numel(a),1) zeros(numel(a),1)]; end %#ok<AGROW>
        q=obj.CurrentConfig;
        if isempty(obj.ArmCache.q) || any(abs(obj.ArmCache.q-q)>1e-9)
            Parm=obj.localArmSamples(q,[0 0 0]); Pf=obj.localFingerPts(q,[0 0 0]);
            obj.ArmCache=struct('q',q,'loc',[Parm;Pf],'nA',size(Parm,1));
        end
        L=obj.ArmCache.loc; W=zeros(size(L)); for k=1:size(L,1), W(k,:)=localB2W(L(k,:),pose); end
        rad=[obj.C.LinkRadius*ones(obj.ArmCache.nA,1); 0.005*ones(size(L,1)-obj.ArmCache.nA,1)];
        P=[ring; W rad];
    end
    % ------------------------------------------------------------ state entry
    function localEnter(obj,state,pose)
        obj.ArmFaultLatched=false; obj.PhaseStartTime=obj.DebugTime;
        switch state
            case 0
                if obj.PreviousState==1 && obj.Sel>0
                    obj.BoxAbsent(obj.Sel)=true;
                    fprintf('[MISSION] t=%.2f box %d is ABSENT at its slot after scanning - skipped, not retried\n',obj.DebugTime,obj.Sel);
                end
                obj.localSelectNext(pose); obj.localPhase('IDLE');
            case 1
                obj.NavLatch=false; obj.ScanTicks=0;
            case 10
                obj.DockStableTicks=0; obj.localPhase('DOCK.WAIT');
            case 2
                obj.localPhase('EXTEND.PLAN');
            case 3
                if ~isempty(obj.TrajB), obj.Trajectory=obj.TrajB; obj.TrajectoryIndex=1; end
                obj.BoxAttached=false; obj.localPhase('GRASP.APPROACH');
            case 4
                obj.ReverseStart=pose(1:2); obj.ReverseDist=obj.localReverseDistance(pose); obj.localPhase('RETRACT.REVERSE');
            case 6
                obj.localPhase('PLACE.PLAN');
            case 8
                obj.BoxAttached=false; obj.Placed=~obj.C.UnrealPhysics; obj.localPhase('RELEASE');
                if obj.C.UnrealPhysics, obj.PhysHandoff=true; obj.PhysDone=false; obj.PhysT0=obj.DebugTime; obj.PhysRelease=obj.BoxLast; obj.SeenHist=nan(26,3); end
                fprintf('[RELEASE] t=%.2f box %d clamp opens at [%.3f %.3f %.3f] tableTopZ=%.3f\n',obj.DebugTime,obj.Sel,obj.BoxLast,obj.C.TableTopZ);
            case 9
                if obj.Sel>0, obj.BoxDone(obj.Sel)=true; if ~isempty(obj.Dest), obj.StationCount(obj.Dest.station)=obj.StationCount(obj.Dest.station)+1; fprintf('[MISSION] box %d delivered to %s slot %d\n',obj.Sel,obj.Dest.name,obj.Dest.slot); end, end
                obj.localPhase('STOW.PLAN');
            case 7
                obj.NavLatch=false;
            case 11
                obj.localPhase('FAULT_HOLD');
            case 12
                obj.localPhase('MISSION_DONE'); fprintf('[MISSION] t=%.2f finished: placed=%s absent=%s; robot parked at home\n',obj.DebugTime,mat2str(find(obj.BoxDone).'),mat2str(find(obj.BoxAbsent).'));
        end
        if state==5 || state==7, obj.NavLatch=false; end
    end
    function d=localReverseDistance(obj,pose)
        % Back off far enough that the held carton, swung about the base axis, clears the rack face by TurnMargin.
        C=obj.C; d=C.ReverseDistance;
        try
            c=localW2B(obj.BoxLast,pose); swing=hypot(c(1),c(2))+0.5*hypot(C.TargetDimensions(1),C.TargetDimensions(2));
            faceDist=pose(1)-C.Boxes(obj.Sel).faceX;
            d=min(1.3,max(C.ReverseDistance,swing+C.TurnMargin-faceDist));
            fprintf('[REVERSE] swing radius=%.2f m, base-to-rack=%.2f m -> back off %.2f m before turning\n',swing,faceDist,d);
        catch
        end
    end
    % ------------------------------------------------------------- navigation
    function [velocity,atGoal]=localNavigate(obj,state,goal,pose)
        C=obj.C; velocity=[0 0]; atGoal=false; carrying=obj.BoxAttached;
        if state==10
            if state~=obj.PreviousState || any(~isfinite(obj.PlannedGoal)) || norm(goal-obj.PlannedGoal)>1e-9
                obj.Waypoints=[pose(1:2);goal]; obj.WaypointIndex=2; obj.PlannedGoal=goal; obj.NavMode='ROTATE'; obj.NavLatch=false;
            end
        elseif any(~isfinite(obj.PlannedGoal)) || norm(goal-obj.PlannedGoal)>1e-9 || state~=obj.PreviousState
            obj.Waypoints=obj.localPlanPath(pose(1:2),goal,carrying,state); obj.WaypointIndex=min(2,size(obj.Waypoints,1)); obj.PlannedGoal=goal; obj.NavMode='ROTATE'; obj.NavLatch=false;
        end
        N=size(obj.Waypoints,1); rs=C.NavSwitchRadius; if carrying, rs=C.NavSwitchRadiusCarry; end
        while obj.WaypointIndex<N
            r=rs; if state==5 && obj.WaypointIndex==N-1, r=C.TableGateRadius; end      % gate point: switch only when really there
            if norm(obj.Waypoints(obj.WaypointIndex,:)-pose(1:2))<r, obj.WaypointIndex=obj.WaypointIndex+1; else, break, end
        end
        final=obj.WaypointIndex==N; target=obj.Waypoints(obj.WaypointIndex,:); if final, target=goal; end
        gd=norm(goal-pose(1:2)); tol=.20; yawGoal=NaN; yawTol=.12;
        switch state
            case 10, tol=.02; yawGoal=C.ShelfFacingYaw; yawTol=.06;
            case 1, yawGoal=C.ShelfFacingYaw;
            case 5, tol=.04; yawGoal=obj.Dest.yaw; yawTol=.06;
            case 7, tol=.15;
        end
        if ~obj.NavLatch && final && gd<tol, obj.NavLatch=true; elseif obj.NavLatch && gd>tol+0.15, obj.NavLatch=false; obj.NavMode='ROTATE'; end
        v=0; w=0; e=0; mode=obj.NavMode;
        if obj.NavLatch
            mode='ALIGN';
            if isnan(yawGoal), atGoal=true; else
                e=localWrap(yawGoal-pose(3)); w=max(-.6,min(.6,1.6*e)); if abs(e)>yawTol*0.5 && abs(w)<0.12, w=0.12*sign(e); end
                atGoal=abs(e)<yawTol; if atGoal, w=0; end
            end
        else
            dist=norm(target-pose(1:2)); e=localWrap(atan2(target(2)-pose(2),target(1)-pose(1))-pose(3));
            if final && dist<0.05, e=0; end
            if strcmp(obj.NavMode,'ROTATE')
                if abs(e)<C.NavRotateExit, obj.NavMode='DRIVE'; end
            elseif abs(e)>C.NavRotateEnter
                obj.NavMode='ROTATE';
            end
            mode=obj.NavMode;
            if strcmp(mode,'ROTATE')
                wm=C.NavTurnRate; w=max(-wm,min(wm,1.6*e)); if abs(w)<0.15, w=0.15*sign(e); end
            else
                vmax=C.NavSpeed; if carrying, vmax=C.NavSpeedCarry; end
                v=min(vmax,1.0*gd);
                if ~final && dist<1.2 && obj.WaypointIndex<N      % slow smoothly before a corner instead of braking late
                    nxt=obj.Waypoints(obj.WaypointIndex+1,:); a1=atan2(target(2)-pose(2),target(1)-pose(1)); a2=atan2(nxt(2)-target(2),nxt(1)-target(1));
                    psi=abs(localWrap(a2-a1)); v=v*(1-0.6*min(1,psi/(pi/2))*min(1,(1.2-dist)/0.6));
                end
                v=max(v,0.04)*max(0,cos(e));
                if final && any(state==uint8([5 10])), v=min(v,max(0,dot(goal-pose(1:2),[cos(pose(3)) sin(pose(3))])/obj.SampleTime)); end
                w=max(-C.NavTurnRate,min(C.NavTurnRate,1.8*e));
            end
        end
        velocity=[v w];
        if obj.DebugTime>=obj.DebugNextNavPrintTime && any(state==uint8([1 5 7 10]))
            fprintf('[NAV] t=%.2f state=%d mode=%s goalDist=%.3f headingErr=%.1fdeg wp=%d/%d atGoal=%d v=%.2f w=%.2f\n',obj.DebugTime,state,mode,gd,rad2deg(e),obj.WaypointIndex,N,atGoal,v,w); obj.DebugNextNavPrintTime=obj.DebugTime+2;
        end
    end
    function wp=localPlanPath(obj,start,goal,carrying,state)
        % Table approach: route to a gate point in line with the final approach, turn there (far from the table and from
        % cartons already placed), then drive straight in.  Turning at the table would swing the held carton through its neighbours.
        if state==5
            gate=goal-obj.C.TableApproachRun*[cos(obj.Dest.yaw) sin(obj.Dest.yaw)];
            wp1=obj.localPlanPathCore(start,gate,carrying); wp=localSimplifyPath([wp1;goal]);
            fprintf('[NAV] table approach via gate point [%.2f %.2f], %d waypoints\n',gate,size(wp,1));
        else
            wp=obj.localPlanPathCore(start,goal,carrying); wp=localSimplifyPath(wp);
        end
    end
    function wp=localPlanPathCore(obj,start,goal,carrying)
        wp=[]; WHp=obj.C.WH;
        if carrying && isfield(WHp,'navMapCarry')
            try
                WHc=WHp; WHc.navMap=WHp.navMapCarry; wp=helperPlanPathAStar(WHc,start,goal);
                if size(wp,1)<2, wp=[]; else, fprintf('[NAV] carry path planned on the widened map (%d points)\n',size(wp,1)); end
            catch
                wp=[];
            end
        end
        if isempty(wp)
            wp=helperPlanPathAStar(WHp,start,goal);
            if carrying, fprintf('[NAV] widened map had no route; using the normal map\n'); end
        end
    end
    % ------------------------------------------------------- approach planning
    function localPlanApproach(obj,box,pose)
        if ~all(isfinite(box)), obj.localFault('nonfiniteTarget'); return, end
        obj.localPinLift(obj.CurrentConfig(1));
        p0=obj.localGeometry(box,pose,[]); phi0=rad2deg(p0.phi);
        tries=unique(min(obj.C.TiltMaxDeg+10,max(8,phi0+[0 5 10 -4 15])),'stable');
        tip0=obj.localTipWorld(obj.CurrentConfig,pose); ax0=obj.localAxisWorld(obj.CurrentConfig,pose);
        reason='collisionPlan'; best=[]; bestMargin=-inf; dyn=obj.localDynBoxes(obj.Sel,pose);
        for k=1:numel(tries)
            p=obj.localGeometry(box,pose,tries(k));
            [Q,S,seg,ok,why]=obj.localChain(obj.CurrentConfig,[tip0;p.pre;p.touch],[ax0;p.axis;p.axis],pose,[]);
            if ~ok, reason='IK'; fprintf('[IKCHK] phi=%.1f FAIL %s\n',tries(k),why); continue, end
            [margin,minSep,what]=obj.localCheck(Q,pose,p.box,dyn);
            fprintf('[COLLCHK] samples=%d minSep=%.4f margin=%.4f worst=%s phiUsed=%.1f\n',size(Q,2),minSep,margin,what,tries(k));
            if margin>=0
                obj.localAcceptApproach(p,Q,S,seg,pose); return
            elseif margin>bestMargin
                bestMargin=margin; best=struct('p',p,'Q',Q,'S',S,'seg',seg);
            end
        end
        if ~isempty(best) && bestMargin>-0.02
            fprintf('[COLLCHK] WARNING no plan met the full clearance; using best plan (margin=%.4f m)\n',bestMargin);
            obj.localAcceptApproach(best.p,best.Q,best.S,best.seg,pose); return
        end
        obj.localFault(reason);
    end
    function localAcceptApproach(obj,p,Q,S,seg,pose)
        i1=seg(2);
        obj.Plan=p; obj.PreLocal=localW2B(p.pre,pose); obj.QPre=Q(:,i1);
        trajA=obj.localRetime(Q(:,1:i1),S(1:i1),obj.C.SpeedApproach,3.0);
        obj.TrajB=obj.localRetime(Q(:,i1:end),S(i1:end)-S(i1),obj.C.SpeedTouch,3.0);
        obj.Trajectory=trajA; obj.TrajectoryIndex=1;
        step=max(abs(diff(Q,1,2)),[],2);
        fprintf('[IKCHK] samples=%d maxPerStepDq=%.4f jointTravel=[%s]\n',size(Q,2),max(step),num2str(sum(abs(diff(Q,1,2)),2).','%.2f '));
        fprintf('[PLAN] phi=%.2f a=[%.4f %.4f %.4f] pFace=[%.4f %.4f %.4f] pContact=[%.4f %.4f %.4f] pPre=[%.4f %.4f %.4f] pTouch=[%.4f %.4f %.4f] toolTipOffset=%.3f sealGapNormal=%.3f\n', ...
            rad2deg(p.phi),p.axis,p.face,p.contact,p.pre,p.touch,obj.C.ToolTipOffset,obj.C.SealGap);
        obj.localPhase('ARM_EXTEND');
    end
    function p=localGeometry(obj,box,pose,phiDeg)
        C=obj.C; nh=[box(1)-pose(1) box(2)-pose(2) 0]; nh=nh/max(norm(nh),eps);
        dims=C.TargetDimensions(:).';
        half=0.5*(abs(nh(1))*dims(1)+abs(nh(2))*dims(2));
        face=[box(1:2)-half*nh(1:2) box(3)];
        contact=[face(1:2) box(3)+C.ContactHeight];
        if isempty(phiDeg)
            sh=localB2W(obj.ShoulderLocal,pose); sh(3)=sh(3)+obj.CurrentConfig(1); dH=norm(contact(1:2)-sh(1:2));
            phi=atan2(sh(3)-contact(3),max(dH,eps))+deg2rad(C.TiltBiasDeg);
            phi=min(deg2rad(C.TiltMaxDeg),max(deg2rad(C.TiltMinDeg),phi));
        else
            phi=deg2rad(phiDeg);
        end
        axis=[cos(phi)*nh(1:2) -sin(phi)];
        p=struct('box',box,'face',face,'contact',contact,'pre',contact-C.PregraspDistance*axis, ...
            'touch',contact-(C.SealGap/cos(phi))*axis,'axis',axis,'nh',nh,'phi',phi);
    end
    % ----------------------------------------------------------------- grasp
    function done=localGrasp(obj,pose)
        done=false; tip=obj.localTipWorld(obj.CurrentConfig,pose);
        if any(strcmp(obj.Phase,{'GRASP.APPROACH','GRASP.SEAL','GRASP.CONFIRM'}))
            gap=dot(obj.Plan.face-tip,obj.Plan.nh);
            if gap<-0.002
                fprintf('[PENETRATION] t=%.2f signedGap=%.5f\n',obj.DebugTime,gap); obj.localFault('penetration'); return
            end
        end
        switch obj.Phase
            case 'GRASP.APPROACH'
                obj.localAdvance();
                if obj.localTrajDone()
                    obj.VacuumStartTime=obj.DebugTime; obj.localPhase('GRASP.SEAL');
                    fprintf('[GRASPFSM] tipNormalGap=%.4f m (plan %.3f) tip=[%.3f %.3f %.3f] boxCentre=[%.3f %.3f %.3f]\n', ...
                        dot(obj.Plan.face-tip,obj.Plan.nh),obj.C.SealGap,tip,obj.Plan.box);
                end
            case 'GRASP.SEAL'
                if obj.DebugTime-obj.PhaseStartTime>=obj.C.SealHold, obj.localPhase('GRASP.CONFIRM'); end
            case 'GRASP.CONFIRM'
                gapN=dot(obj.Plan.face-tip,obj.Plan.nh); boxErr=norm(obj.BoxSeenW-obj.Plan.box);
                if gapN<=obj.C.SealGap+obj.C.AttachGapSlack && gapN>=-0.002 && boxErr<=obj.C.AttachBoxTol
                    ax=obj.localAxisWorld(obj.CurrentConfig,pose);
                    obj.HeldOffset=localW2B(obj.BoxSeenW,pose)-localW2B(tip,pose);
                    obj.HeldAxisLocal=(localRz(pose(3)).'*ax(:)).';
                    obj.BoxLast=obj.BoxSeenW; obj.BoxYaw=0; obj.LastBaseYaw=pose(3); obj.BoxAttached=true; obj.AttachBoxZ=obj.BoxSeenW(3); obj.NextAttachPrint=obj.DebugTime;
                    fprintf('[GRASPFSM] clamp closed, box %d attached t=%.2f gap=%.4f boxVsPlan=%.4f heldOffset(base frame)=[%.3f %.3f %.3f]\n',obj.Sel,obj.DebugTime,gapN,boxErr,obj.HeldOffset);
                    [Q,S,~,ok,why]=obj.localChain(obj.CurrentConfig,[tip;tip+[0 0 obj.C.LiftDistance]],[ax;ax],pose,[]);
                    if ~ok, fprintf('[IKCHK] lift FAIL %s\n',why); obj.localFault('liftPlan'); return, end
                    obj.Trajectory=obj.localRetime(Q,S,obj.C.SpeedLift,1.5); obj.TrajectoryIndex=1; obj.localPhase('GRASP.LIFT');
                elseif obj.DebugTime-obj.PhaseStartTime>=obj.C.GraspConfirmTimeout
                    fprintf('[GRASPCHK] NO ATTACH. tipGap=%.4f (need <=%.3f) boxVsPlan=%.4f (need <=%.3f) box=[%.3f %.3f %.3f] tip=[%.3f %.3f %.3f]\n', ...
                        gapN,obj.C.SealGap+obj.C.AttachGapSlack,boxErr,obj.C.AttachBoxTol,obj.BoxSeenW,tip);
                    obj.localFault('noAttach');
                end
            case 'GRASP.LIFT'
                obj.localAdvance();
                if obj.localTrajDone()
                    rise=obj.BoxSeenW(3)-obj.AttachBoxZ;
                    fprintf('[LIFTCHK] t=%.2f boxZ rise=%.3f m (need >=%.3f) seen=[%.3f %.3f %.3f] cmd=[%.3f %.3f %.3f]\n',obj.DebugTime,rise,obj.C.LiftRiseMin,obj.BoxSeenW,obj.BoxLast);
                    if rise<obj.C.LiftRiseMin && obj.C.BoxFollowFault
                        fprintf('[BOXSET] The Transform Set block did not move box %d (readback unchanged). Check ActorTag/Priority.\n',obj.Sel);
                        obj.localFault('boxNotFollowing'); return
                    end
                    obj.localPhase('GRASP.DONE'); done=true;
                end
            case 'GRASP.DONE'
                done=true;
        end
    end
    % --------------------------------------------------------------- retract
    function [velocity,done]=localRetract(obj,pose)
        velocity=[0 0]; done=false;
        switch obj.Phase
            case 'RETRACT.REVERSE'
                d=norm(pose(1:2)-obj.ReverseStart); remain=obj.ReverseDist-d;
                if remain<=0.005
                    obj.localPlanLevel(pose);
                else
                    v=min([obj.C.ReverseSpeed,0.05+1.0*d,max(0.05,1.2*remain)]); velocity=[-v 0];
                end
            case 'RETRACT.LEVEL'
                obj.localAdvance();
                if obj.localTrajDone(), obj.localFinishLevel(pose); obj.localPlanLower(pose); end
            case 'RETRACT.LOWER'
                obj.localAdvance();
                if obj.localTrajDone(), obj.localPlanCarry(pose); end
            case 'RETRACT.CARRY'
                obj.localAdvance(); done=obj.localTrajDone();
        end
    end
    function localPlanLevel(obj,pose)
        % Hand straight: tool axis and tool X horizontal, carton stays put while the clamp rotates around it.
        C=obj.C; obj.RLevelLocal=[]; obj.IKWide=false; obj.localPinLift(obj.CurrentConfig(1));
        tip0=obj.localTipWorld(obj.CurrentConfig,pose); T0=getTransform(obj.Robot,obj.CurrentConfig,obj.EEName); Rcur=localRz(pose(3))*T0(1:3,1:3);
        zh=[Rcur(1:2,3).' 0]; zh=zh/norm(zh); xh=[Rcur(1:2,1).' 0]; xh=xh-dot(xh,zh)*zh;
        if norm(xh)<1e-6, xh=cross([0 0 1],zh); end
        xh=xh/norm(xh); yh=cross(zh,xh); Rlev=[xh(:) yh(:) zh(:)];
        [hxy,~]=obj.localBoxHalf(obj.BoxYaw); half=abs(zh(1))*hxy(1)+abs(zh(2))*hxy(2);
        tipL=obj.BoxLast-(half+C.SealGap)*zh+[0 0 C.ContactHeight];
        Rs=cat(3,Rcur,Rlev);
        for wide=[false true]
            obj.IKWide=wide; obj.localPinLift(obj.CurrentConfig(1));
            [Q,S,~,ok,why]=obj.localChain(obj.CurrentConfig,[tip0;tipL],[],pose,Rs);
            if ok, break, end
            fprintf('[LEVEL] window %s failed: %s\n',localTern(wide,'wide','normal'),why);
        end
        if ok
            dyn=obj.localDynBoxes(obj.Sel,pose); [margin,minSep,what]=obj.localCheck(Q,pose,obj.BoxLast,dyn);
            fprintf('[COLLCHK] level samples=%d minSep=%.4f margin=%.4f worst=%s\n',size(Q,2),minSep,margin,what);
            if margin<-0.03, ok=false; why=sprintf('collision margin %.3f',margin); end
        end
        if ok
            obj.Trajectory=obj.localRetime(Q,S,C.SpeedLevel,C.LevelMinTime); obj.TrajectoryIndex=1;
            obj.RLevelLocal=localRz(pose(3)).'*Rlev;
            fprintf('[LEVEL] hand levelling planned: tilt %.1f deg -> 0, tip moves %.3f m\n',acosd(max(-1,min(1,Rcur(:,3).'*zh(:)))),S(end));
            obj.localPhase('RETRACT.LEVEL');
        else
            fprintf('[LEVEL] FAIL %s -> carrying with the grasp tilt (not levelled)\n',why);
            obj.localPlanLower(pose);
        end
    end
    function localFinishLevel(obj,pose)
        tip=obj.localTipWorld(obj.CurrentConfig,pose);
        obj.HeldOffset=localW2B(obj.BoxLast,pose)-localW2B(tip,pose);
        ax=obj.localAxisWorld(obj.CurrentConfig,pose); obj.HeldAxisLocal=(localRz(pose(3)).'*ax(:)).';
        fprintf('[LEVEL] done. heldOffset(base frame)=[%.3f %.3f %.3f] toolAxisZ=%.3f\n',obj.HeldOffset,ax(3));
    end
    function localPlanLower(obj,pose)
        % Choose the column height for carrying/placing.  The carton must arrive ABOVE the table top, so the column is set
        % to table height (boxZ - 0.53) - the same arm geometry as the validated level-1 grasp.  A ladder of column
        % offsets and IK windows is tried; the carry is checked (whole arm + held carton) before anything moves.
        % If every rung fails the robot FAULTS and holds - it never drives with the carton below table height.
        C=obj.C; b=C.Boxes(obj.Sel); q=obj.CurrentConfig;
        hBase=max(0,min(C.LiftMax,obj.Dest.slotXYZ(3)+C.PlaceHover-C.LiftBaseBoxZ));
        rung=0; found=false;
        for dh=[0 0.08 -0.08 0.16 -0.16]
            hc=max(0,min(C.LiftMax,hBase+dh));
            for wide=[false true]
                rung=rung+1;
                if rung<=obj.SkipRungs, fprintf('[CARRYCHK] rung %d (lift %.2f, %s) skipped by test hook\n',rung,hc,localTern(wide,'wide','normal')); continue, end
                [Q,S,ok,why,mode]=obj.localTryCarry(pose,hc,wide);
                fprintf('[CARRYCHK] rung %d lift=%.2f window=%s -> %s\n',rung,hc,localTern(wide,'wide','normal'),localTern(ok,['OK (' mode ')'],why));
                if ok, found=true; break, end
            end
            if found, break, end
        end
        if ~found
            fprintf('[IKCHK] carry above the table is not reachable on any rung -> FAULT (robot will not drive with the carton below table height)\n');
            obj.localFault('carryIK'); return
        end
        obj.CarryLift=hc; obj.CarryWide=wide; obj.CarryQ=Q; obj.CarryS=S;
        fprintf('[LIFT] carry column height %.2f m (table-height carry) for box %d\n',hc,obj.Sel);
        if abs(q(1)-hc)>0.003
            obj.Trajectory=obj.localJointPath(q,[hc;q(2:end)],max(1.0,1.2*abs(q(1)-hc)/C.LiftSpeed)); obj.TrajectoryIndex=1;
            obj.localPhase('RETRACT.LOWER');
        else
            obj.localPlanCarry(pose);
        end
    end
    function [Q,S,ok,why,mode]=localTryCarry(obj,pose,hc,wide)
        b=obj.C.Boxes(obj.Sel); nominal=[obj.Dest.approach obj.Dest.yaw];
        obj.IKWide=wide; obj.localPinLift(hc);
        seed=[hc;obj.CurrentConfig(2:end)];
        tipL=localW2B(obj.Dest.slotXYZ,nominal)-obj.HeldOffset+[0 0 obj.C.PlaceHover];
        carryW=localB2W(tipL,pose); axW=(localRz(pose(3))*obj.HeldAxisLocal(:)).';
        tip0=obj.localTipWorld(seed,pose); ax0=obj.localAxisWorld(seed,pose);
        ok=false; why=''; Q=[]; S=[]; mode='';
        dyn=obj.localDynBoxes(obj.Sel,pose);
        for m=1:2
            if m==1
                if isempty(obj.RLevelLocal), continue, end
                Rw=localRz(pose(3))*obj.RLevelLocal; T0=getTransform(obj.Robot,seed,obj.EEName); R0=localRz(pose(3))*T0(1:3,1:3);
                [Q,S,~,ok,why]=obj.localChain(seed,[tip0;carryW],[],pose,cat(3,R0,Rw)); mode='level hand';
            else
                [Q,S,~,ok,why]=obj.localChain(seed,[tip0;carryW],[ax0;axW],pose,[]); mode='axis only';
            end
            if ok
                [margin,~,what]=obj.localCheck(Q,pose,[],dyn);
                hs=obj.localHeldSep(Q,pose,dyn);
                if margin<-0.03 || hs<0.005, ok=false; why=sprintf('%s: collision margin %.3f heldSep %.3f (%s)',mode,margin,hs,what); end
            else
                why=[mode ': ' why];
            end
            if ok, return, end
        end
    end
    function localPlanCarry(obj,~)
        obj.Trajectory=obj.localRetime(obj.CarryQ,obj.CarryS,obj.C.SpeedCarry,3.0); obj.TrajectoryIndex=1;
        fprintf('[PLAN] carry planned at lift %.2f m: %d samples, path %.3f m\n',obj.CarryLift,size(obj.CarryQ,2),obj.CarryS(end));
        obj.localPhase('RETRACT.CARRY');
    end
    % ----------------------------------------------------------------- place
    function localPlanPlace(obj,boxTarget,pose)
        if ~all(isfinite(boxTarget)) || isempty(obj.HeldOffset), obj.localFault('placeTarget'); return, end
        R=localRz(pose(3));
        tipPlaceL=localW2B(boxTarget,pose)-obj.HeldOffset;
        tipPlaceW=localB2W(tipPlaceL,pose); hoverW=localB2W(tipPlaceL+[0 0 obj.C.PlaceHover],pose);
        axW=(R*obj.HeldAxisLocal(:)).';
        tip0=obj.localTipWorld(obj.CurrentConfig,pose); ax0=obj.localAxisWorld(obj.CurrentConfig,pose);
        ok=false; why=''; Q=[]; S=[]; seg=[];
        for wide=[obj.CarryWide ~obj.CarryWide]
            obj.IKWide=wide; obj.localPinLift(obj.CarryLift);
            if ~isempty(obj.RLevelLocal)
                Rw=R*obj.RLevelLocal; T0=getTransform(obj.Robot,obj.CurrentConfig,obj.EEName); R0=R*T0(1:3,1:3);
                [Q,S,seg,ok,why]=obj.localChain(obj.CurrentConfig,[tip0;hoverW;tipPlaceW],[],pose,cat(3,R0,Rw,Rw));
            end
            if ~ok
                if ~isempty(obj.RLevelLocal), fprintf('[IKCHK] level place FAIL (%s) %s -> axis-only place\n',localTern(wide,'wide','normal'),why); end
                [Q,S,seg,ok,why]=obj.localChain(obj.CurrentConfig,[tip0;hoverW;tipPlaceW],[ax0;axW;axW],pose,[]);
            end
            if ok, break, end
        end
        if ~ok, fprintf('[IKCHK] place FAIL %s\n',why); obj.localFault('placeIK'); return, end
        dyn=obj.localDynBoxes(obj.Sel,pose);
        [margin,minSep,what]=obj.localCheck(Q,pose,[],dyn);
        fprintf('[COLLCHK] place samples=%d minSep=%.4f margin=%.4f worst=%s\n',size(Q,2),minSep,margin,what);
        hs=obj.localHeldSep(Q,pose,dyn);
        fprintf('[COLLCHK] held box vs world/other boxes minSep=%.4f (need >= 0.010)\n',hs);
        if hs<0.010, obj.localFault('heldBoxCollision'); return, end
        i1=seg(2);
        A=obj.localRetime(Q(:,1:i1),S(1:i1),obj.C.SpeedPlace,3.0); B=obj.localRetime(Q(:,i1:end),S(i1:end)-S(i1),obj.C.SpeedDescend,2.0);
        obj.Trajectory=[A B(:,2:end)]; obj.TrajectoryIndex=1;
        fprintf('[PLAN] place boxTarget=[%.3f %.3f %.3f] tipPlace=[%.3f %.3f %.3f] hover=[%.3f %.3f %.3f]\n',boxTarget,tipPlaceW,hoverW);
        obj.localPhase('ARM_PLACE');
    end
    function localPlanStow(obj,pose)
        % Release -> (1) Cartesian retreat: back along the tool axis (fingers slide out of the carton), then up
        %            (2) base backs away from the table  (3) joint-space fold to the stow pose.
        % The stowed hand reaches ~0.63 m ahead of the base axis and the carton sits at the table edge, so folding in place
        % would drive the hand into the carton: the base backs off first.  Every candidate is checked for the whole arm.
        C=obj.C; obj.IKWide=true; obj.localPinLift(obj.CurrentConfig(1)); dyn=obj.localDynBoxes(obj.Sel,pose);
        poseAfter=[pose(1:2)-C.StowBackoff*[cos(pose(3)) sin(pose(3))] pose(3)];
        tip0=obj.localTipWorld(obj.CurrentConfig,pose); ax0=obj.localAxisWorld(obj.CurrentConfig,pose);
        zh=[ax0(1:2) 0]; if norm(zh)>1e-6, zh=zh/norm(zh); else, zh=ax0; end
        best=[]; bestM=-inf;
        for up=C.StowUp
            for back=C.StowBack
                retW=tip0-back*zh; topW=retW+[0 0 up];
                [Q,S,~,ok,why]=obj.localChain(obj.CurrentConfig,[tip0;retW;topW],[ax0;ax0;ax0],pose,[]);
                if ~ok, fprintf('[COLLCHK] stow back=%.2f up=%.2f IK fail: %s\n',back,up,why); continue, end
                A=obj.localRetime(Q,S,C.SpeedLift*2,2.0); Bj=obj.localJointPath(A(:,end),obj.StowConfig,C.ArmMoveTime);
                [mA,sepA,whatA]=obj.localCheck(A,pose,obj.BoxLast,dyn,0.04);   %#ok<ASGLU>
                [mB,sepB,whatB]=obj.localCheck(Bj,poseAfter,obj.BoxLast,dyn);
                m=min(mA,mB);
                fprintf('[COLLCHK] stow back=%.2f up=%.2f retreat margin=%.4f (%s) fold margin=%.4f (%s)\n',back,up,mA,whatA,mB,whatB);
                if m>bestM, bestM=m; best=struct('A',A,'B',Bj); end
                if m>=0, break, end
            end
            if bestM>=0, break, end
        end
        if isempty(best)
            fprintf('[IKCHK] Cartesian retreat failed everywhere -> back off, then direct joint-space stow\n');
            best=struct('A',obj.CurrentConfig,'B',obj.localJointPath(obj.CurrentConfig,obj.StowConfig,C.ArmMoveTime));
        elseif bestM<0
            fprintf('[COLLCHK] WARNING stow path margin %.4f m (best of all candidates)\n',bestM);
        end
        obj.Trajectory=best.A; obj.TrajectoryIndex=1; obj.StowTrajB=best.B; obj.localPhase('STOW.RETREAT');
    end
    function [velocity,done]=localStow(obj,pose)
        velocity=[0 0]; done=false;
        switch obj.Phase
            case 'STOW.RETREAT'
                obj.localAdvance();
                if obj.localTrajDone(), obj.StowReverseStart=pose(1:2); obj.localPhase('STOW.REVERSE'); end
            case 'STOW.REVERSE'
                d=norm(pose(1:2)-obj.StowReverseStart); remain=obj.C.StowBackoff-d;
                if remain<=0.005
                    obj.Trajectory=obj.StowTrajB; obj.TrajectoryIndex=1; obj.localPhase('STOW.FOLD');
                else
                    v=min([obj.C.ReverseSpeed,0.05+1.0*d,max(0.05,1.2*remain)]); velocity=[-v 0];
                end
            case 'STOW.FOLD'
                obj.localAdvance(); done=obj.localTrajDone();
        end
    end
    % ------------------------------------------------- Cartesian IK machinery
    function localPinLift(obj,h)
        % The lift joint is a plan-time constant: pin its limits so the IK solver only moves the seven arm joints.
        % The pin window is clamped INTO the real joint range ([0 LiftMax]); at h=0 a window of [-1e-4 1e-4] let the solver
        % return q(1)=-1e-4 which the limit test rejected (level-1 carton: every IK sample failed).
        if ~isempty(obj.IK) && isequal(obj.IKPin,h) && isequal(obj.IKPinWide,obj.IKWide), return, end
        w=warning('off','all'); cl=onCleanup(@() warning(w)); %#ok<NASGU>
        if obj.IKWide, t=copy(obj.IKRobotWide); else, t=copy(obj.IKRobot); end
        lo=obj.JointLimits(1,1); hi=obj.JointLimits(1,2); h=max(lo,min(hi,h));
        for k=1:numel(t.Bodies)
            j=t.Bodies{k}.Joint;
            if ~strcmp(j.Type,'fixed')
                j.PositionLimits=[max(lo,h-1e-4) min(hi,h+1e-4)]; replaceJoint(t,t.Bodies{k}.Name,j); break
            end
        end
        obj.IK=inverseKinematics('RigidBodyTree',t);
        obj.IK.SolverParameters.AllowRandomRestart=false;
        try, obj.IK.SolverParameters.MaxIterations=300; catch, end
        try, obj.IK.SolverParameters.MaxTime=0.5; catch, end
        obj.IKPin=h; obj.IKPinWide=obj.IKWide;
    end
    function [Q,S,seg,ok,why]=localChain(obj,seed,tips,axes,pose,Rs)
        ds=obj.C.IKSampleSpacing; q=seed(:); Q=q; S=0; seg=1; ok=true; why='ok'; prevTip=tips(1,:); R=localRz(pose(3));
        full=~isempty(Rs);
        for i=1:size(tips,1)-1
            a=tips(i,:); b=tips(i+1,:); n=max(1,ceil(norm(b-a)/ds));
            if full, n=max(n,ceil(localRotAngle(Rs(:,:,i),Rs(:,:,i+1))/0.04)); end
            for k=1:n
                u=k/n; tipW=a+u*(b-a);
                if full
                    Rw=localRotInterp(Rs(:,:,i),Rs(:,:,i+1),u); Rb=R.'*Rw; axL=Rb(:,3).';
                    [qn,good,ep,ea]=obj.localIKSample(q,localW2B(tipW,pose),axL,Rb);
                else
                    axW=(1-u)*axes(i,:)+u*axes(i+1,:); axW=axW/norm(axW);
                    [qn,good,ep,ea]=obj.localIKSample(q,localW2B(tipW,pose),(R.'*axW(:)).',[]);
                end
                dq=max(abs(qn-q));
                if ~good || dq>obj.C.ArmMaxStep
                    ok=false; why=sprintf('segment %d sample %d/%d posErr=%.4f aim=%.1fdeg dq=%.3f good=%d',i,k,n,ep,ea,dq,good); return
                end
                q=qn; Q(:,end+1)=q; S(end+1)=S(end)+norm(tipW-prevTip)+1e-6; prevTip=tipW; %#ok<AGROW>
            end
            seg(end+1)=numel(S); %#ok<AGROW>
        end
    end
    function [q,good,ep,ea]=localIKSample(obj,seed,tipL,axL,Rb)
        Tp=getTransform(obj.Robot,seed,obj.EEName); Rp=Tp(1:3,1:3);
        if isempty(Rb)
            Rd=localAlign(Rp(:,3),axL(:))*Rp; wo=obj.C.IKOrientWeight;
        else
            Rd=Rb; wo=obj.C.IKOrientWeightFull;
        end
        T=[Rd tipL(:); 0 0 0 1];
        q=seed(:); ep=inf; ea=inf; good=false;
        try
            [qs,~]=obj.IK(obj.EEName,T,[wo wo wo 1 1 1],seed(:));
            q=reshape(double(qs),8,1);
        catch ME
            fprintf('[IKCHK] solver exception: %s\n',ME.message); return
        end
        Tq=getTransform(obj.Robot,q,obj.EEName);
        ep=norm(Tq(1:3,4)-tipL(:));
        if isempty(Rb)
            ea=acosd(max(-1,min(1,(Tq(1:3,3).'*axL(:))/norm(axL))));
            tolA=obj.C.IKAimTolDeg;
        else
            ea=rad2deg(localRotAngle(Tq(1:3,1:3),Rd)); tolA=obj.C.IKRotTolDeg;
        end
        good=all(isfinite(q)) && ep<=obj.C.IKPosTol && ea<=tolA && obj.localLimits(q);
    end
    function tf=localLimits(obj,q)
        m=[0;0;1;0;1;0;1;0]*deg2rad(3); lim=obj.JointLimits;
        qq=q; qq(1)=max(lim(1,1),min(lim(1,2),qq(1)));   % lift joint: clamp (pinned, checked below)
        tf=all(qq>=lim(:,1)+m & qq<=lim(:,2)-m) && all(abs(q([2 4 6 8])-obj.StowConfig([2 4 6 8]))<=localTern(obj.IKWide,obj.C.BranchWindowWide,obj.C.BranchWindow)+0.05) && abs(q(1)-obj.IKPin)<3e-3;
    end
    function p=localTipWorld(obj,q,pose)
        T=getTransform(obj.Robot,q,obj.EEName); p=localB2W(T(1:3,4).',pose);
    end
    function a=localAxisWorld(obj,q,pose)
        T=getTransform(obj.Robot,q,obj.EEName); a=(localRz(pose(3))*T(1:3,3)).';
    end
    % ------------------------------------------------ analytic clearance check
    function B=localDynBoxes(obj,exclude,pose)
        % AABBs of every other carton (placed ones on the table, ones still on racks) as obstacles for planning.
        B=zeros(0,6);
        for k=1:size(obj.BoxPos,1)
            if k==exclude, continue, end
            c=obj.BoxPos(k,:); if hypot(c(1)-pose(1),c(2)-pose(2))>5, continue, end
            [hxy,hz]=obj.localBoxHalf(obj.BoxYawV(k)); B(end+1,:)=[c(1)-hxy(1) c(1)+hxy(1) c(2)-hxy(2) c(2)+hxy(2) c(3)-hz c(3)+hz]; %#ok<AGROW>
        end
    end
    function [margin,minSep,what]=localCheck(obj,Q,pose,boxCentre,dyn,boxSlack)
        if nargin<6, boxSlack=0; end   % >0 for sliding contact with the carton (fingers straddle it while retreating)
        C=obj.C; margin=inf; minSep=inf; what='none';
        near=obj.Obstacles;
        if ~isempty(near)
            cx=(near(:,1)+near(:,2))/2; cy=(near(:,3)+near(:,4))/2; near=near(hypot(cx-pose(1),cy-pose(2))<8,:);
        end
        if nargin>=5 && ~isempty(dyn), near=[near;dyn]; end
        boxB=[];
        if ~isempty(boxCentre)
            [hxy0,hz0]=obj.localBoxHalf(obj.BoxYaw); hb=[hxy0 hz0]; boxB=[boxCentre(1)-hb(1) boxCentre(1)+hb(1) boxCentre(2)-hb(2) boxCentre(2)+hb(2) boxCentre(3)-hb(3) boxCentre(3)+hb(3)];
        end
        idx=unique([1:C.CheckStride:size(Q,2) size(Q,2)]); linkMin=C.LinkRadius+C.ClearLinkSurface;
        for k=idx
            [pts,tool]=obj.localArmPointsSplit(Q(:,k),pose); fing=obj.localFingerPts(Q(:,k),pose);
            cand=cell(0,3);
            if ~isempty(boxB)
                cand(end+1,:)={min(localAabbDist(fing,boxB)),C.ClearFingerBox-boxSlack,'finger/box'}; %#ok<AGROW>
                cand(end+1,:)={min(localAabbDist(tool,boxB)),C.ClearToolBox-boxSlack,'tool/box'}; %#ok<AGROW>
                cand(end+1,:)={min(localAabbDist(pts,boxB)),linkMin-boxSlack,'link/box'}; %#ok<AGROW>
            end
            if ~isempty(near)
                cand(end+1,:)={min(min(localAabbDist(fing,near))),C.ClearFingerWorld,'finger/world'}; %#ok<AGROW>
                cand(end+1,:)={min(min(localAabbDist(tool,near))),C.ClearToolWorld,'tool/world'}; %#ok<AGROW>
                cand(end+1,:)={min(min(localAabbDist(pts,near))),linkMin,'link/world'}; %#ok<AGROW>
            end
            for j=1:size(cand,1)
                d=cand{j,1}; m=d-cand{j,2};
                if d<minSep, minSep=d; end
                if m<margin, margin=m; what=sprintf('%s@sample%d',cand{j,3},k); end
            end
        end
    end
    function [hxy,hz]=localBoxHalf(obj,yaw)
        c=cos(yaw); sn=sin(yaw); dm=obj.C.TargetDimensions(:).';
        hxy=[abs(c)*dm(1)/2+abs(sn)*dm(2)/2, abs(sn)*dm(1)/2+abs(c)*dm(2)/2]; hz=dm(3)/2;
    end
    function [ctr,pen]=localResolveContact(obj,ctr,yaw)
        % Minimum-translation push-out of the box AABB from every static obstacle AABB (decks, panels, table).
        pen=0; N=obj.Obstacles; if isempty(N), return, end
        [hxy,hz]=obj.localBoxHalf(yaw);
        for it=1:4
            bb=[ctr(1)-hxy(1) ctr(1)+hxy(1) ctr(2)-hxy(2) ctr(2)+hxy(2) ctr(3)-hz ctr(3)+hz];
            ox=min(bb(2)-N(:,1),N(:,2)-bb(1)); oy=min(bb(4)-N(:,3),N(:,4)-bb(3)); oz=min(bb(6)-N(:,5),N(:,6)-bb(5));
            hit=find(ox>1e-4 & oy>1e-4 & oz>1e-4,1); if isempty(hit), return, end
            [m,ax]=min([ox(hit) oy(hit) oz(hit)]);
            nc=[(N(hit,1)+N(hit,2))/2 (N(hit,3)+N(hit,4))/2 (N(hit,5)+N(hit,6))/2];
            sgn=sign(ctr(ax)-nc(ax)); if sgn==0, sgn=1; end
            ctr(ax)=ctr(ax)+sgn*m; pen=max(pen,m);
        end
    end
    function z=localSupportZ(obj,ctr,yaw)
        % Highest static top surface under the box footprint that is at or below the box bottom.
        z=0; N=obj.Obstacles; if isempty(N), return, end
        [hxy,hz]=obj.localBoxHalf(yaw); bottom=ctr(3)-hz;
        ox=min(ctr(1)+hxy(1)-N(:,1),N(:,2)-(ctr(1)-hxy(1))); oy=min(ctr(2)+hxy(2)-N(:,3),N(:,4)-(ctr(2)-hxy(2)));
        ok=ox>0 & oy>0 & N(:,6)<=bottom+0.005; if any(ok), z=max(N(ok,6)); end
    end
    function d=localHeldSep(obj,Q,pose,dyn)
        % Minimum distance between the (yaw-rotated) held box and shelves/decks/table/other cartons over a chain.
        d=inf; near=obj.Obstacles; if isempty(near) || isempty(obj.HeldOffset), return, end
        cx=(near(:,1)+near(:,2))/2; cy=(near(:,3)+near(:,4))/2; near=near(hypot(cx-pose(1),cy-pose(2))<8,:);
        if nargin>=4 && ~isempty(dyn), near=[near;dyn]; end
        [hxy,hz]=obj.localBoxHalf(obj.BoxYaw); hx=hxy(1); hy=hxy(2);
        for k=1:size(Q,2)
            tip=obj.localTipWorld(Q(:,k),pose); ctr=localB2W(localW2B(tip,pose)+obj.HeldOffset,pose);
            bb=[ctr(1)-hx ctr(1)+hx ctr(2)-hy ctr(2)+hy ctr(3)-hz ctr(3)+hz];
            for j=1:size(near,1)
                n=near(j,:); gx=max([n(1)-bb(2) bb(1)-n(2) 0]); gy=max([n(3)-bb(4) bb(3)-n(4) 0]); gz=max([n(5)-bb(6) bb(5)-n(6) 0]);
                dd=sqrt(gx^2+gy^2+gz^2); if dd==0, dd=-min([bb(2)-n(1) n(2)-bb(1) bb(4)-n(3) n(4)-bb(3) bb(6)-n(5) n(6)-bb(5)]); end
                if dd<d, d=dd; end
            end
        end
    end
    function P=localFingerPts(obj,q,pose)
        % Corner/face sample points of the palm and the two finger pads (ToolTip frame -> world).
        C=obj.C; T=getTransform(obj.Robot,q,obj.EEName); xi=C.FingerCenterX-C.FingerHalfT; xo=C.FingerCenterX+C.FingerHalfT;
        L=[];
        for sx=[-1 1]
            for yy=[-C.FingerHalfH 0 C.FingerHalfH]
                for zz=[0 C.FingerLen/2 C.FingerLen]
                    L(end+1,:)=[sx*xi yy zz]; %#ok<AGROW>
                    L(end+1,:)=[sx*xo yy zz]; %#ok<AGROW>
                end
            end
            for yy=[-C.FingerHalfH C.FingerHalfH], L(end+1,:)=[sx*xo yy -0.03]; end %#ok<AGROW>
        end
        W=(T(1:3,1:3)*L.'+T(1:3,4)).'; P=zeros(size(W));
        for k=1:size(W,1), P(k,:)=localB2W(W(k,:),pose); end
    end
    function P=localArmSamples(obj,q,pose)
        % Dense centre-line samples (every ~5 cm) along every arm link, lift carriage to tool tip.
        n=numel(obj.LinkNames); O=zeros(n,3);
        for k=1:n
            T=getTransform(obj.Robot,q,obj.LinkNames{k}); O(k,:)=localB2W(T(1:3,4).',pose);
        end
        O=[O;obj.localTipWorld(q,pose)]; P=zeros(0,3);
        for k=1:size(O,1)-1
            L=norm(O(k+1,:)-O(k,:)); m=max(1,ceil(L/0.05)); s=(0:m-1).'/m; P=[P; O(k,:)+s.*(O(k+1,:)-O(k,:))]; %#ok<AGROW>
        end
        P=[P;O(end,:)];
    end
    function [pts,tool]=localArmPointsSplit(obj,q,pose)
        P=obj.localArmSamples(q,pose); pts=P(1:end-2,:);   % links up to the bracelet; the hand (last 0.10 m) is covered by the tool and finger checks
        tip=P(end,:); a=P(end-2,:); s=linspace(0,1,6).'; tool=a+s.*(tip-a);
    end
    % ------------------------------------------------------------- time scaling
    function Q=localRetime(obj,Qk,Sk,speed,minTime)
        Sk=Sk(:).'; L=Sk(end); T=max(minTime,1.9*L/max(speed,1e-3)); n=max(2,round(T/obj.SampleTime)+1);
        t=linspace(0,1,n); u=t.^3.*(10-15*t+6*t.^2);
        Sk=Sk+(0:numel(Sk)-1)*1e-9;
        Q=interp1(Sk.',Qk.',(u*L).','linear').'; Q(:,1)=Qk(:,1); Q(:,end)=Qk(:,end);
    end
    function Q=localJointPath(obj,a,b,T)
        n=max(2,round(T/obj.SampleTime)+1); t=linspace(0,1,n); u=t.^3.*(10-15*t+6*t.^2);
        Q=a(:)+(b(:)-a(:))*u;
    end
    function tf=localTrajDone(obj), tf=obj.TrajectoryIndex>size(obj.Trajectory,2); end
    function localAdvance(obj)
        if ~obj.ArmFaultLatched && obj.TrajectoryIndex<=size(obj.Trajectory,2)
            obj.CurrentConfig=obj.Trajectory(:,obj.TrajectoryIndex); obj.TrajectoryIndex=obj.TrajectoryIndex+1;
        end
    end
    function localPhase(obj,s), obj.Phase=s; obj.PhaseStartTime=obj.DebugTime; fprintf('[GRASPFSM] t=%.2f phase=%s\n',obj.DebugTime,s); end
    function localFault(obj,s)
        obj.ArmFaultLatched=true; fprintf('[FAULT] t=%.2f box=%d reason=%s\n',obj.DebugTime,obj.Sel,s); obj.localPhase('FAULT_HOLD');
    end
end
end

function C=localDefaults(C)
d=struct('ContactHeight',0.05,'TiltBiasDeg',10,'TiltMinDeg',12,'TiltMaxDeg',25,'IKPosTol',0.006,'IKAimTolDeg',12, ...
    'IKSampleSpacing',0.015,'IKOrientWeight',0.25,'IKOrientWeightFull',1.0,'IKRotTolDeg',6,'BranchWindow',0.50,'BranchWindowWide',1.30,'SpeedApproach',0.12,'SpeedTouch',0.06,'SpeedLift',0.04, ...
    'SpeedCarry',0.10,'SpeedPlace',0.10,'SpeedDescend',0.04,'SpeedLevel',0.08,'LevelMinTime',2.0,'CarryRise',0.12,'ReverseDistance',0.65,'ReverseSpeed',0.15,'TurnMargin',0.12, ...
    'PlaceHover',0.10,'RetreatDistance',0.15,'SealHold',0.6,'GraspConfirmTimeout',1.5,'ClearToolBox',0.006,'ClearLinkBox',0.04, ...
    'ClearToolWorld',0.010,'ClearLinkWorld',0.04,'FingerCenterX',0.272,'FingerHalfT',0.01,'FingerLen',0.18,'FingerHalfH',0.06, ...
    'ClearFingerBox',0.004,'ClearFingerWorld',0.02,'BoxFollowFault',true,'BoxYawSign',-1,'UnrealPhysics',true,'LiftRiseMin',0.04,'AttachGapSlack',0.012,'AttachBoxTol',0.06, ...
    'LinkRadius',0.05,'ClearLinkSurface',0.01,'CheckStride',3,'StowUp',[0.20 0.30 0.40],'StowBack',[0.15 0.25],'ScanDwell',1.0,'DetectFovHalfDeg',35, ...
    'NavSwitchRadius',0.45,'NavSwitchRadiusCarry',0.60,'NavRotateEnter',0.60,'NavRotateExit',0.10,'LiftSpeed',0.25,'StowBackoff',0.60,'NavSpeed',0.60,'NavSpeedCarry',0.45,'NavTurnRate',1.1,'TableApproachRun',1.6,'TableGateRadius',0.15,'StowRackGap',0.10);
if ~isfield(C,'TableSurfaceZ'), C.TableSurfaceZ=C.TableTopZ+0.05; end   % table slab is centred at TableTopZ and is 0.10 m thick
f=fieldnames(d);
for k=1:numel(f), if ~isfield(C,f{k}), C.(f{k})=d.(f{k}); end, end
C.ArmMaxStep=max(C.ArmMaxStep,0.30);   % 1.5 cm Cartesian samples move a joint ~0.05-0.1 rad; a branch flip moves >1 rad
C.SealGap=max(C.SealGap,0.012);        % normal gap tool tip -> box face
end
function r2=localNarrowBranch(robot,qStow,window)
% Window-limit the continuous joints (arm joints 1,3,5,7 = tree joints 2,4,6,8) on a private copy so the IK solver cannot leave the stow branch.
r2=robot;
try
    t=copy(robot); n=0;
    for k=1:numel(t.Bodies)
        j=t.Bodies{k}.Joint;
        if ~strcmp(j.Type,'fixed')
            n=n+1;
            if any(n==[2 4 6 8])
                j.PositionLimits=[qStow(n)-window qStow(n)+window];
                replaceJoint(t,t.Bodies{k}.Name,j);
            end
        end
    end
    r2=t; lim=localJointBounds(r2);
    fprintf('[IKBRANCH] window limits applied: q1=[%.2f %.2f] q3=[%.2f %.2f] q5=[%.2f %.2f] q7=[%.2f %.2f]\n',lim(2,:),lim(4,:),lim(6,:),lim(8,:));
catch ME
    fprintf('[IKBRANCH] narrowing unavailable (%s); drift guard still active\n',ME.message);
end
end
function B=localObstacleBoxes(C)
B=zeros(0,6);
try
    for k=1:size(C.WH.shelves,1)
        d=C.WH.shelves(k,:); g=helperShelfBays(d);
        for z=1:3, B(end+1,:)=localAabb([d(1) d(2) g.deckTop(z)-g.deckT/2],[d(4) d(3) g.deckT]); end %#ok<AGROW>
        for y=g.panelY, B(end+1,:)=localAabb([d(1) y g.H/2],[d(4) g.panelT g.H]); end %#ok<AGROW>
    end
    for k=1:size(C.WH.tables,1)
        t=C.WH.tables(k,:); B(end+1,:)=localAabb([t(1) t(2) C.TableSurfaceZ/2],[t(3) t(4) C.TableSurfaceZ]); %#ok<AGROW>
    end
catch ME
    fprintf('[COLLCHK] obstacle list unavailable (%s)\n',ME.message);
end
end
function b=localAabb(c,dim)
b=[c(1)-dim(1)/2 c(1)+dim(1)/2 c(2)-dim(2)/2 c(2)+dim(2)/2 c(3)-dim(3)/2 c(3)+dim(3)/2];
end
function d=localAabbDist(P,B)
% Signed distance from points P (m-by-3) to axis-aligned boxes B (n-by-6) -> m-by-n, negative inside.
cx=(B(:,1)+B(:,2))/2; cy=(B(:,3)+B(:,4))/2; cz=(B(:,5)+B(:,6))/2;
hx=(B(:,2)-B(:,1))/2; hy=(B(:,4)-B(:,3))/2; hz=(B(:,6)-B(:,5))/2;
qx=abs(P(:,1)-cx.')-hx.'; qy=abs(P(:,2)-cy.')-hy.'; qz=abs(P(:,3)-cz.')-hz.';
d=sqrt(max(qx,0).^2+max(qy,0).^2+max(qz,0).^2)+min(max(max(qx,qy),qz),0);
end
function [push,who]=localPushVector(pts,bb,ctr)
% Deepest penetration of any pusher point (x y z r) into the carton AABB bb -> horizontal displacement for the carton.
push=[0 0]; who='';
if isempty(pts), return, end
lo=bb([1 3 5])-pts(:,4); hi=bb([2 4 6])+pts(:,4);
inside=pts(:,1)>lo(:,1) & pts(:,1)<hi(:,1) & pts(:,2)>lo(:,2) & pts(:,2)<hi(:,2) & pts(:,3)>lo(:,3) & pts(:,3)<hi(:,3);
% a pusher that is above the top face or below the bottom face is not pushing sideways
inside=inside & pts(:,3)<bb(6)-0.01 & pts(:,3)>bb(5)+0.01;
idx=find(inside); if isempty(idx), return, end
bestPen=0;
for i=idx.'
    px=min(pts(i,1)-lo(i,1),hi(i,1)-pts(i,1)); py=min(pts(i,2)-lo(i,2),hi(i,2)-pts(i,2));
    if px<py, pen=px; dirv=[sign(ctr(1)-pts(i,1)+1e-9) 0]; else, pen=py; dirv=[0 sign(ctr(2)-pts(i,2)+1e-9)]; end
    if pen>bestPen, bestPen=pen; push=dirv*pen; who=sprintf('pusher point [%.2f %.2f %.2f]',pts(i,1:3)); end
end
end
function wp=localSimplifyPath(wp)
% Drop nearly collinear interior waypoints so the follower is not asked to make tiny heading changes.
if size(wp,1)<3, return, end
keep=true(size(wp,1),1);
last=1;
for i=2:size(wp,1)-1
    a=wp(i,:)-wp(last,:); b=wp(i+1,:)-wp(i,:);
    ang=abs(atan2(a(1)*b(2)-a(2)*b(1),a(1)*b(1)+a(2)*b(2)));
    if ang<deg2rad(12), keep(i)=false; else, last=i; end
end
wp=wp(keep,:);
end
function s=localTern(c,a,b), if c, s=a; else, s=b; end, end
function R=localAlign(a,b)
% Minimal rotation taking unit vector a onto unit vector b.
a=a/norm(a); b=b/norm(b); v=cross(a,b); s=norm(v); c=dot(a,b);
if s<1e-9
    R=eye(3);
else
    K=[0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0]; R=eye(3)+K+K*K*((1-c)/s^2);
end
end
function ang=localRotAngle(Ra,Rb)
d=Ra.'*Rb; ang=acos(max(-1,min(1,(trace(d)-1)/2)));
end
function R=localRotInterp(Ra,Rb,u)
d=Ra.'*Rb; ang=acos(max(-1,min(1,(trace(d)-1)/2)));
if ang<1e-9, R=Ra; return, end
ax=[d(3,2)-d(2,3); d(1,3)-d(3,1); d(2,1)-d(1,2)]/(2*sin(ang));
K=[0 -ax(3) ax(2); ax(3) 0 -ax(1); -ax(2) ax(1) 0]; th=u*ang;
R=Ra*(eye(3)+sin(th)*K+(1-cos(th))*(K*K));
end
function R=localRz(yaw), R=[cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1]; end
function w=localB2W(l,pose), w=(localRz(pose(3))*l(:)).'+[pose(1) pose(2) 0]; end
function l=localW2B(w,pose), l=(localRz(pose(3)).'*(w(:)-[pose(1);pose(2);0])).'; end
function l=localJointBounds(robot)
l=zeros(8,2); n=0;
for k=1:numel(robot.Bodies)
    j=robot.Bodies{k}.Joint;
    if ~strcmp(j.Type,'fixed'), n=n+1; l(n,:)=j.PositionLimits; end
end
end
function a=localWrap(a), a=atan2(sin(a),cos(a)); end
