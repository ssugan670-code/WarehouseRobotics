classdef WarehousePerceptionSystem < matlab.System
%WAREHOUSEPERCEPTIONSYSTEM Stable RGB-D target estimate for the supervisor.
% The camera is rigidly mounted to the base, so the only live pose input is
% the delayed ground-truth base pose. This avoids a camera-output feedback loop.
properties (Nontunable)
    FramesRequired = 3
end
properties (Access=private)
    C
    History
    DebugTime
    DebugNextPrintTime
    LastPoint
    LastValidPoint
    SnapshotSaved
    SettleCount
end
methods (Access=protected)
    function setupImpl(obj)
        obj.C=WarehouseRobotPhase2Config();
        obj.FramesRequired=obj.C.Perception.FramesRequired;
        obj.History=zeros(0,3);
        obj.DebugTime=0; obj.DebugNextPrintTime=0; obj.LastPoint=[NaN NaN NaN];
        obj.LastValidPoint=[NaN NaN NaN];
        obj.SnapshotSaved=false; obj.SettleCount=0;
    end
    function [valid,x,y,z]=stepImpl(obj,rgb,depth,basePose,cameraTranslation,cameraRotation,atGoal,cameraPitchDeg)
        basePose=reshape(double(basePose),1,3);
        cameraTranslation=reshape(double(cameraTranslation),1,3);
        cameraRotation=reshape(double(cameraRotation),1,3);
        % These two diagnostic inputs retain fixed ports in the generated
        % model. Snapshot timing below deliberately does not use AtGoal.
        atGoal=logical(atGoal); %#ok<NASGU>
        cameraPitchDeg=double(cameraPitchDeg);
        % The camera chart and perception now consume this same signal.
        % Do not read the neutral config default for a live Simulink frame.
        activeMount=obj.C.CameraMount;
        activeMount.PitchDown=deg2rad(cameraPitchDeg);
        % Always execute the camera detector so its diagnostic measurements
        % remain available when the selected detector is simulated.
        [cameraPoint,~,cameraDetected,details]=perceptionBlobDetect(rgb,depth, ...
            struct('FocalLength',obj.C.CameraFocalLength, ...
                   'PrincipalPoint',obj.C.CameraPrincipalPoint), ...
            basePose,activeMount);
        simTarget=localSpawnedTarget(obj.C);
        [simInFrame,simRange]=localTargetInCamera(simTarget,basePose,activeMount, ...
            obj.C.CameraFocalLength,obj.C.CameraPrincipalPoint,obj.C.CameraImageSize);
        switch lower(char(obj.C.Perception.Mode))
            case 'camera'
                p=cameraPoint;
                detected=cameraDetected;
            case 'sim'
                % This scene/detector module reports the spawned target pose
                % when it is in the camera frustum and range. As in the
                % MathWorks scene-attribute examples, it intentionally does
                % not model visual occlusion; RGB-D HSV continues in the
                % background for later comparison.
                detected=simInFrame && simRange<=obj.C.Perception.SimRange;
                p=simTarget+obj.C.Perception.SimNoise*randn(1,3);
            otherwise
                error('Phase2:PerceptionMode','Unknown C.Perception.Mode "%s".',obj.C.Perception.Mode);
        end
        if detected
            obj.LastPoint=p;
            obj.History=[obj.History; p];
            if size(obj.History,1)>obj.FramesRequired
                obj.History=obj.History(end-obj.FramesRequired+1:end,:);
            end
        else
            obj.History=zeros(0,3);
        end
        valid=size(obj.History,1)>=obj.FramesRequired;
        if valid
            p=median(obj.History,1);
            obj.LastValidPoint=p;
        elseif all(isfinite(obj.LastValidPoint))
            % A target may leave the near-field camera frustum while the
            % supervisor transitions from docking to arm motion. Preserve
            % the last filtered measurement for numeric consumers; the
            % separate valid output still truthfully reports this dropout.
            p=obj.LastValidPoint;
        else
            % Before any complete filtered observation, no target estimate
            % exists and NaN correctly communicates that condition.
            p=[NaN NaN NaN];
        end
        x=p(1); y=p(2); z=p(3);
        obj.DebugTime=obj.DebugTime+obj.C.CameraSampleTime;
        % Guaranteed self-contained snapshot: this relies only on the
        % base-pose input and a sustained shelf-facing yaw.  It cannot be
        % skipped because AtGoal, Stateflow, or perception is false.
        headingError=atan2(sin(basePose(3)-obj.C.ShelfFacingYaw), ...
            cos(basePose(3)-obj.C.ShelfFacingYaw));
        if abs(headingError)<0.15
            obj.SettleCount=obj.SettleCount+1;
        else
            obj.SettleCount=0;
        end
        pitchTag=localPitchTag(cameraPitchDeg);
        if obj.C.Debug.Enabled && ~obj.SnapshotSaved && obj.SettleCount>=50
            if strcmp(pitchTag,'pitch0')
                rgbFile='phase2_snap2_rgb.png';
            else
                rgbFile=['phase2_snap2_rgb_' pitchTag '.png'];
            end
            imwrite(rgb,fullfile(obj.C.Debug.SnapshotDir,rgbFile));
            rgbValues=reshape(double(rgb),[],3);
            rgbMean=mean(rgbValues,1); rgbStd=std(rgbValues,0,1);
            hsv=rgb2hsv(im2double(rgb));
            c=cos(basePose(3)); s=sin(basePose(3));
            cmdTranslation=[basePose(1)+0.20*c+0.18*s, ...
                basePose(2)+0.20*s-0.18*c,1.00];
            % This exactly mirrors Camera Mount: rotation=[0 pitch -yaw].
            cmdRotation=[0 cameraPitchDeg -basePose(3)*180/pi];
            % Forward-project the debug-only true target by exactly
            % inverting perceptionBlobDetect's Rz*mount*Ry*P chain.  This
            % is intentionally separate from the renderer diagnostics: it
            % tells whether the perception model itself expects the target
            % to be in its 640-by-480 image.
            boxWorld=reshape(double(simTarget),3,1);
            baseTranslation=[basePose(1);basePose(2);0];
            yaw=basePose(3);
            Rz=[cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
            pit=activeMount.PitchDown;
            Ry=[cos(pit) 0 sin(pit); 0 1 0; -sin(pit) 0 cos(pit)];
            P=[0 0 1; -1 0 0; 0 -1 0];
            pin=P.'*Ry.'*(Rz.'*(boxWorld-baseTranslation)-activeMount.Translation(:));
            focal=obj.C.CameraFocalLength; principal=obj.C.CameraPrincipalPoint;
            u=principal(1)+pin(1)*focal(1)/pin(3);
            v=principal(2)+pin(2)*focal(2)/pin(3);
            inFrame=pin(3)>0 && u>=1 && u<=640 && v>=1 && v<=480;
            worldDelta=boxWorld-cmdTranslation(:);
            fprintf(['[SNAP2] basePose=[%.3f %.3f %.1fdeg] cmdTranslation=[%.3f %.3f %.3f] ' ...
                'cmdRotation=[%.1f %.1f %.1fdeg] settleCount=%d\n'], ...
                basePose(1),basePose(2),basePose(3)*180/pi,cmdTranslation,cmdRotation,obj.SettleCount);
            reportedRotationDeg=rad2deg(cameraRotation);
            fprintf(['[CAMPOSE] reportedTranslation=[%.3f %.3f %.3f] commandTranslation=[%.3f %.3f %.3f] ' ...
                'translationError=%.3f reportedRotationRaw=[%.4f %.4f %.4f] ' ...
                'reportedRotationDeg=[%.1f %.1f %.1f] commandRotationDeg=[%.1f %.1f %.1f]\n'], ...
                cameraTranslation,cmdTranslation,norm(cameraTranslation-cmdTranslation), ...
                cameraRotation,reportedRotationDeg,cmdRotation);
            fprintf(['[PROJ] boxWorld=[%.3f %.3f %.3f] camToBox=[%.3f %.3f %.3f] ' ...
                'perceptionPin=[%.3f %.3f %.3f] pixel=[%.1f %.1f] inFrame=%d ' ...
                'modelPitch=%.1f commandPitch=%.1f\n'], ...
                boxWorld,worldDelta,pin,u,v,inFrame,rad2deg(pit),cameraPitchDeg);
            occluders=localSightlineCandidates(obj.C,cmdTranslation,boxWorld);
            if isempty(occluders)
                fprintf('[OCCLUDER] no shelf actor is within 0.30 m of the target sightline.\n');
            else
                for k=1:numel(occluders)
                    fprintf(['[OCCLUDER] %s name=%s centre=[%.3f %.3f %.3f] ' ...
                        'dims=[%.3f %.3f %.3f] rayT=%.3f\n'], ...
                        occluders(k).kind,occluders(k).name,occluders(k).centre, ...
                        occluders(k).dims,occluders(k).rayT);
                end
            end
            hue=hsv(:,:,1);
            blueCount=nnz(hue>=0.55 & hue<=0.70);
            magentaCount=nnz(hue>=0.82 & hue<=0.91);
            otherCount=numel(hue)-blueCount-magentaCount;
            fprintf(['[SNAP2STAT] mean=[%.1f %.1f %.1f] std=[%.1f %.1f %.1f] ' ...
                'hueRange=[%.3f %.3f] sat05=%d val04=%d blue=%d magenta=%d other=%d file=%s\n'], ...
                rgbMean,rgbStd,min(hsv(:,:,1),[],'all'),max(hsv(:,:,1),[],'all'), ...
                nnz(hsv(:,:,2)>0.5),nnz(hsv(:,:,3)>0.4),blueCount,magentaCount,otherCount,rgbFile);
            fprintf('[SNAP2] saved %s\n',fullfile(obj.C.Debug.SnapshotDir,rgbFile));
            obj.SnapshotSaved=true;
        end
        if obj.C.Debug.Enabled && obj.DebugTime>=obj.DebugNextPrintTime
            aspect=details.boundingBox(3)/max(details.boundingBox(4),eps);
            fprintf(['[PERC] t=%.2f mode=%s detected=%d cameraDetected=%d simInFrame=%d simRange=%.3f ' ...
                'historyLen=%d valid=%d lastPoint=[%.3f %.3f %.3f] blobs=%d gate=%s area=%.1f ' ...
                'aspect=%.3f depth=%.3f\n'],obj.DebugTime,obj.C.Perception.Mode,detected,cameraDetected,simInFrame,simRange, ...
                size(obj.History,1),valid,obj.LastPoint,details.blobCount, ...
                details.gate,details.area,aspect,details.depth);
            obj.DebugNextPrintTime=obj.DebugTime+5.0;
        end
    end
    function n=getNumInputsImpl(~), n=7; end
    function [a,b,c,d,e,f,g]=getInputSizeImpl(~), a=[480 640 3];b=[480 640];c=[1 3];d=[1 3];e=[1 3];f=[1 1];g=[1 1];end
    function [a,b,c,d,e,f,g]=getInputDataTypeImpl(~), a='uint8';b='single';c='double';d='double';e='double';f='boolean';g='double';end
    function [a,b,c,d,e,f,g]=isInputComplexImpl(~), a=false;b=false;c=false;d=false;e=false;f=false;g=false;end
    function [a,b,c,d,e,f,g]=isInputFixedSizeImpl(~), a=true;b=true;c=true;d=true;e=true;f=true;g=true;end
    function [a,b,c,d]=getOutputSizeImpl(~), a=[1 1];b=[1 1];c=[1 1];d=[1 1];end
    function [a,b,c,d]=getOutputDataTypeImpl(~), a='boolean';b='double';c='double';d='double';end
    function [a,b,c,d]=isOutputComplexImpl(~), a=false;b=false;c=false;d=false;end
    function [a,b,c,d]=isOutputFixedSizeImpl(~), a=true;b=true;c=true;d=true;end
end
end

function tag=localPitchTag(pitchDeg)
if abs(pitchDeg)<1e-9
    tag='pitch0';
elseif pitchDeg<0
    tag=sprintf('pitchNeg%d',round(abs(pitchDeg)));
else
    tag=sprintf('pitch%d',round(pitchDeg));
end
end

function target=localSpawnedTarget(C)
target=reshape(double(C.TargetXYZ),1,3)+[reshape(double(C.Test.SpawnOffset),1,2) 0];
end

function [inFrame,range]=localTargetInCamera(target,basePose,mount,focal,principal,imageSize)
% Invert the same Rz*mount*Ry*P sequence used by perceptionBlobDetect.
baseTranslation=[basePose(1);basePose(2);0]; yaw=basePose(3);
Rz=[cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
pit=mount.PitchDown;
Ry=[cos(pit) 0 sin(pit); 0 1 0; -sin(pit) 0 cos(pit)];
P=[0 0 1; -1 0 0; 0 -1 0];
pin=P.'*Ry.'*(Rz.'*(target(:)-baseTranslation)-mount.Translation(:));
u=principal(1)+pin(1)*focal(1)/pin(3);
v=principal(2)+pin(2)*focal(2)/pin(3);
inFrame=pin(3)>0 && u>=1 && u<=imageSize(2) && v>=1 && v<=imageSize(1);
cameraWorld=baseTranslation+Rz*mount.Translation(:);
range=norm(target(:)-cameraWorld);
end

function candidates=localSightlineCandidates(C,cameraWorld,targetWorld)
% Report static shelf actors which block, or are close enough to block, the
% camera-to-target ray.  Filler geometry is regenerated with the same seed
% and loop order used by buildShelf; this is diagnostic only.
sid=C.Target.shelf;
d=C.WH.shelves(sid,:); G=helperShelfBays(d);
cx=d(1); cy=d(2); widX=d(4);
cameraWorld=reshape(cameraWorld,3,1); targetWorld=reshape(targetWorld,3,1);
ray=targetWorld-cameraWorld;
all=struct('name',{},'kind',{},'centre',{},'dims',{});
for i=1:numel(G.panelY)
    all(end+1)=localActor(sprintf('S%dPanel%d',sid,i),'intersects', ...
        [cx G.panelY(i) G.H/2],[widX G.panelT G.H]); %#ok<AGROW>
end
for level=1:3
    all(end+1)=localActor(sprintf('S%dDeck%d',sid,level),'intersects', ...
        [cx cy G.deckTop(level)-G.deckT/2],[widX-.02 d(3) G.deckT]); %#ok<AGROW>
end

targetSlot=C.Target.slot;
targetSlot=helperTargetSlot(G,targetSlot);
savedRng=rng; rng(100+sid);
for side=[-1 1]
    for level=1:3
        for slot=1:numel(G.slotY)
            isTarget=side==1 && C.Target.level==level && slot==targetSlot;
            if ~G.slotClear(slot) || isTarget || rand>=C.ShelfFill, continue; end
            bx=.40+.15*rand; by=.50+.18*rand; bh=.28+.16*rand;
            inset=G.rowInset+.15*rand;
            xc=cx+side*(widX/2-inset-bx/2);
            yc=G.slotY(slot)+.06*(rand-.5);
            all(end+1)=localActor(sprintf('S%dB%d_%d_%d',sid,side>0,level,slot), ...
                'intersects',[xc yc G.deckTop(level)+bh/2],[bx by bh]); %#ok<AGROW>
        end
    end
end
rng(savedRng);

candidates=struct('name',{},'kind',{},'centre',{},'dims',{},'rayT',{});
for i=1:numel(all)
    [hit,tHit]=localRayAabb(cameraWorld,ray,all(i).centre,all(i).dims);
    [near,tNear]=localRayAabb(cameraWorld,ray,all(i).centre,all(i).dims+0.60);
    if ~near, continue; end
    item=all(i); item.rayT=ternary(hit,tHit,tNear);
    if ~hit, item.kind='near'; end
    candidates(end+1)=item; %#ok<AGROW>
end
end

function item=localActor(name,kind,centre,dims)
item=struct('name',name,'kind',kind,'centre',centre,'dims',dims);
end

function [hit,tHit]=localRayAabb(origin,ray,centre,dims)
lo=centre(:)-dims(:)/2; hi=centre(:)+dims(:)/2;
tMin=0; tMax=1; hit=true;
for axis=1:3
    if abs(ray(axis))<eps
        if origin(axis)<lo(axis) || origin(axis)>hi(axis), hit=false; break; end
    else
        t1=(lo(axis)-origin(axis))/ray(axis); t2=(hi(axis)-origin(axis))/ray(axis);
        tMin=max(tMin,min(t1,t2)); tMax=min(tMax,max(t1,t2));
        if tMin>tMax, hit=false; break; end
    end
end
tHit=tMin;
end

function value=ternary(condition,whenTrue,whenFalse)
if condition, value=whenTrue; else, value=whenFalse; end
end
