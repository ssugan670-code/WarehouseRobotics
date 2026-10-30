classdef Phase3CamDetect < matlab.System
%PHASE3CAMDETECT  Camera-based carton detection of ONE robot (Simulink MATLAB System block 'Cam Detect', one per robot).
%   The robot's own camera (Simulation 3D Camera on the lift carriage, picture 480x640) is the sensor that finds the carton:
%   while the robot stands at the shelf / stand in the scan state, p3CamLocate looks for the carton in the picture, around the place the warehouse
%   database says it should be, and measures where it really is (the readback of the Unreal actor is NOT used here).
%   inputs : 1 img (480x640x3 uint8)   2 state (FSM state; scanning = 1)   3 estPose [x y yaw] (robot's own estimate)   4 lift [m]
%   output : camBus = [valid x y z yaw score seq done]   (world frame, from the robot's own pose estimate)
%            valid = carton measured (>= CamMinFound frames agree)   done = scan finished (found or not)   seq = scan counter
%   The measurement is made in the ROBOT frame; the world position is only expressed with the pose estimate, so a localization error does not
%   disturb the grasp geometry (the arm plans in the robot frame again).
properties (Nontunable)
    RobotId = 1
end
properties (Access = private)
    C
    S
    Fleet
    name
    seq
    frames
    rels
    scores
    nFound
    done
    valid
    out
    lastPose
    stillN
    lastState
    saved
    dbg
    lastLift
    liftN
end
methods
    function obj = Phase3CamDetect(varargin)
        setProperties(obj,nargin,varargin{:});
    end
end
methods (Access = protected)
    function setupImpl(obj)
        obj.C = evalin('base','Phase3Cfg'); obj.S = obj.C.Sense; obj.Fleet = evalin('base','Phase3Fleet'); obj.name = obj.Fleet.robots(obj.RobotId).name;
        obj.seq = 0; obj.localReset(); obj.lastPose = [NaN NaN NaN]; obj.lastState = uint8(255); obj.saved = 0; obj.dbg = 0; obj.lastLift = 0; obj.liftN = 0;
        if obj.RobotId == 1, try, p3CamCVT('status'); catch ME, fprintf('[CAM] Computer Vision Toolbox layer failed to start (%s) - plain camera code used\n',ME.message); end, end      % prints once which toolbox functions really run
    end
    function out = stepImpl(obj,img,state,estPose,lift)
        state = uint8(state); pose = reshape(double(estPose),1,3);
        if state ~= 1
            if obj.lastState == 1 && obj.seq > 0, end
            obj.localReset(); obj.lastState = state; obj.lastPose = pose; out = obj.out; return
        end
        if obj.lastState ~= 1, obj.seq = obj.seq + 1; obj.localReset(); end
        obj.lastState = state;
        still = ~any(isnan(obj.lastPose)) && hypot(pose(1)-obj.lastPose(1),pose(2)-obj.lastPose(2)) < 0.02 && abs(atan2(sin(pose(3)-obj.lastPose(3)),cos(pose(3)-obj.lastPose(3)))) < 0.02;
        obj.lastPose = pose; if still, obj.stillN = obj.stillN + 1; else, obj.stillN = 0; end
        if abs(double(lift) - obj.lastLift) < 0.002, obj.liftN = obj.liftN + 1; else, obj.liftN = 0; end
        obj.lastLift = double(lift);
        if ~obj.done && obj.stillN >= 2 && obj.frames < obj.S.CamFrames
            job = []; try, job = obj.Fleet.robots(obj.RobotId).job; catch, end
            ready = ~isempty(job);
            if ready, ready = abs(double(lift) - p3ScanLift(job.src.xyz(3))) < 0.03 && obj.liftN >= 3; end      % the carriage must be at the height the camera needs for this level and stand still
            if ready
                try
                    obj.localFrame(uint8(img),pose,double(lift),job);
                catch ME
                    obj.done = true; obj.valid = false; obj.setOut(pose,[NaN NaN NaN],0,0);
                    fprintf('[CAM] %s: camera detection failed (%s) - controller falls back as configured\n',obj.name,ME.message);
                end
            end
        end
        out = obj.out;
    end
    function sts = getSampleTimeImpl(obj)
        sts = createSampleTime(obj,'Type','Discrete','SampleTime',evalin('base','Phase3Cfg.Sense.CamPeriod'));
    end
    function n = getNumInputsImpl(~), n = 4; end
    function n = getNumOutputsImpl(~), n = 1; end
    function o = getOutputSizeImpl(~), o = [1 8]; end
    function o = getOutputDataTypeImpl(~), o = 'double'; end
    function o = isOutputComplexImpl(~), o = false; end
    function o = isOutputFixedSizeImpl(~), o = true; end
end
methods (Access = private)
    function localReset(obj)
        obj.frames = 0; obj.rels = zeros(0,3); obj.scores = zeros(0,1); obj.nFound = 0; obj.done = false; obj.valid = false; obj.stillN = 0;
        obj.out = [0 0 0 0 0 0 obj.seq 0];
    end
    function setOut(obj,pose,xyz,yaw,score)
        obj.out = [double(obj.valid) xyz yaw score obj.seq double(obj.done)];
    end
    function localFrame(obj,img,pose,lift,job)
        xyz = job.src.xyz(:).'; yawW = job.src.boxYaw; dims = obj.C.TargetDimensions(:).';
        c = cos(pose(3)); s = sin(pose(3)); d = xyz(1:2) - pose(1:2);
        expRel = [c*d(1) + s*d(2), -s*d(1) + c*d(2), xyz(3)];                        % slot -> robot frame (robot's own pose estimate)
        opt = struct('minIoU',obj.S.CamMinIoU,'search',0.18,'minPix',150);
        [ok,rel,info] = p3CamLocate(img,expRel,yawW - pose(3),dims,lift,obj.C.P3.cartonColour,opt);
        obj.frames = obj.frames + 1;
        if ok, obj.rels(end+1,:) = rel; obj.scores(end+1,1) = info.score; obj.nFound = obj.nFound + 1; end
        try, obj.Fleet.camOv{obj.RobotId} = struct('hull',localGet(info,'hull',[]),'uv',localGet(info,'expUV',[NaN NaN]),'win',localGet(info,'win',[0 0 0 0]),'ok',ok,'t',now); catch, end      % the live camera window draws the same overlay as the saved cam_*.png
        if obj.saved < 3 || (~ok && obj.dbg < 6) || (obj.frames == 1 && false), obj.localDump(img,info,ok,job); end
        if obj.nFound >= obj.S.CamMinFound
            m = median(obj.rels,1); w = [c -s; s c]*m(1:2).'; obj.valid = true; obj.done = true;
            obj.out = [1 pose(1)+w(1) pose(2)+w(2) xyz(3) yawW median(obj.scores) obj.seq 1];
            fprintf('[CAM] %s: carton C%d found by camera at (%.3f, %.3f), %.0f mm from the database slot (IoU %.2f, %d frames)\n',obj.name,job.carton,obj.out(2),obj.out(3), ...
                1000*hypot(m(1)-expRel(1),m(2)-expRel(2)),obj.out(6),obj.frames);
        elseif obj.frames >= obj.S.CamFrames || (obj.frames - obj.nFound >= obj.S.CamFrames - obj.S.CamMinFound + 1)
            obj.valid = false; obj.done = true; obj.out = [0 0 0 0 0 0 obj.seq 1];
            fprintf('[CAM] %s: carton C%d NOT seen by the camera (%d of %d frames, last reason: %s)\n',obj.name,job.carton,obj.nFound,obj.frames,info.reason);
        end
    end
    function localDump(obj,img,info,ok,job)
        % debug picture for the log folder: expected centre (cross) and best silhouette (polygon)
        try
            P = phase3Paths(); if ~isfolder(P.cam), mkdir(P.cam); end
            im = img; im = localLine(im,info.hull,[0 255 0]); u = round(info.expUV); 
            if all(isfinite(u)), im = localLine(im,[u(1)-8 u(2); u(1)+8 u(2)],[255 0 255]); im = localLine(im,[u(1) u(2)-8; u(1) u(2)+8],[255 0 255]); end
            if info.win(3) > 0, w = info.win; im = localLine(im,[w(1) w(2); w(3) w(2); w(3) w(4); w(1) w(4); w(1) w(2)],[255 255 0]); end
            obj.saved = obj.saved + 1; if ~ok, obj.dbg = obj.dbg + 1; end
            imwrite(im,fullfile(P.cam,sprintf('cam_%s_scan%d_f%d_%s.png',obj.name,obj.seq,obj.frames,localTern(ok,'found','miss')))); %#ok<NASGU>
        catch
        end
    end
end
end
function im = localLine(im,pts,col)
for i = 1:size(pts,1)-1
    n = max(2,ceil(max(abs(pts(i+1,:)-pts(i,:)))));
    x = round(linspace(pts(i,1),pts(i+1,1),n)); y = round(linspace(pts(i,2),pts(i+1,2),n));
    k = x >= 1 & x <= size(im,2) & y >= 1 & y <= size(im,1);
    for c = 1:3, ch = im(:,:,c); ch(sub2ind(size(ch),y(k),x(k))) = col(c); im(:,:,c) = ch; end
end
end
function s = localTern(c,a,b), if c, s = a; else, s = b; end, end
function v = localGet(s,f,d)
if isfield(s,f), v = s.(f); else, v = d; end
end
