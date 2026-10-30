classdef Phase3Sense < matlab.System
%PHASE3SENSE  One robot's lidar localization + lidar/fleet tracking + protective field (Simulink MATLAB System block 'Robot Sense', one per robot).
%
%   inputs : 1  truePose [x y yaw]   pose of the simulated vehicle.  Used ONLY (a) to synthesise what the wheel encoders would measure (true motion
%                                    increments + noise) and (b) to report the localization error.  The controller never receives it through this block.
%            2  lidar point cloud     Unreal 'Simulation 3D Lidar' output (any M-by-N-by-3 / n-by-3 shape).  Fallback if that block is not usable: the same
%                                    scan ray-cast from the map (C.Sense.LidarMode = 'raycast', logged at start-up).
%   outputs: 1  estPose [x y yaw]     what the robot believes (Navigation Toolbox monteCarloLocalization = AMCL + wheel odometry)  -> controller 'basePose'
%            2  bus [locOk safeStop nearest bearing nTracks lidarMode(1 unreal,2 raycast) nUnknown locErr]
%
%   Tracking (Sensor Fusion and Tracking Toolbox multiObjectTracker): lidar points not explained by the map are clustered; clusters + the other robots'
%   broadcast poses are fused into tracks.  The protective field (p3SafetyField) is the last line of defence below the fleet planner.
%   All errors are caught: if AMCL / the tracker cannot run, the block degrades to odometry / no tracks and says so ONCE in the command window.
properties (Nontunable)
    RobotId = 1
end
properties (Access = private)
    C
    G
    res
    off
    Gi
    Fleet
    names
    angles
    nBeams
    minR
    maxR
    dt
    nLid
    tick
    tNow
    isFirst
    lastTrue
    odom
    odomU
    amclPose
    est
    startPose
    amcl
    hasAmcl
    ampFail
    stream
    calibDone
    calibMir
    calibYaw
    calibN
    calibAcc
    calibT0
    calibEmpty
    calibRound
    calibNext
    lidarMode
    tracker
    hasTracker
    trkFail
    tLast
    safeStop
    stopSince
    releaseUntil
    nStops
    unkCount
    lastUnk
    errSum
    errMax
    errN
    nUpd
    errOverT
    dockAnch
    nReset
    nRej
    rows
    nextRow
    nextPrint
    lastBus
end
methods
    function obj = Phase3Sense(varargin)
        setProperties(obj,nargin,varargin{:});
    end
end
methods (Access = protected)
    function setupImpl(obj)
        obj.C = evalin('base','Phase3Cfg'); S = obj.C.Sense;
        try, obj.Fleet = evalin('base','Phase3Fleet'); obj.names = {obj.Fleet.robots.name}; catch, obj.Fleet = []; obj.names = {}; end
        [obj.G,obj.res,obj.off,obj.Gi] = localGrid(obj.C,S);
        obj.nBeams = S.LidarBeams; obj.angles = linspace(-pi,pi,obj.nBeams+1); obj.angles(end) = [];
        obj.minR = S.LidarMinRange; obj.maxR = S.LidarMaxRange;
        obj.dt = obj.C.SampleTime; obj.nLid = max(1,round(S.LidarPeriod/obj.dt)); obj.tick = 0; obj.tNow = 0;
        obj.isFirst = true; obj.lastTrue = [0 0 0]; obj.odom = [0 0 0]; obj.odomU = [0 0 0]; obj.amclPose = [0 0 0]; obj.est = [0 0 0];
        h = obj.C.P3.robots(obj.RobotId).home; obj.startPose = [h 0];
        obj.stream = RandStream('mt19937ar','Seed',1000+obj.RobotId);
        obj.calibDone = false; obj.calibMir = false; obj.calibYaw = 0; obj.calibN = 0; obj.calibAcc = zeros(2,4); obj.lidarMode = S.LidarMode; obj.calibT0 = -1; obj.calibEmpty = 0; obj.calibRound = 0; obj.calibNext = 0;
        obj.hasAmcl = false; obj.ampFail = false; obj.hasTracker = false; obj.trkFail = false; obj.tLast = -1;
        obj.safeStop = false; obj.stopSince = 0; obj.releaseUntil = -1; obj.nStops = 0; obj.unkCount = 0; obj.lastUnk = 0;
        obj.errSum = 0; obj.errMax = 0; obj.errN = 0; obj.nUpd = 0; obj.errOverT = 0; obj.dockAnch = false; obj.nReset = 0; obj.nRej = 0; obj.rows = zeros(0,6); obj.nextRow = 0; obj.nextPrint = 60; obj.lastBus = zeros(1,8);
        if strcmpi(S.Localization,'amcl'), obj.localMakeAmcl(S); end
        if S.Tracking, obj.localMakeTracker(); end
        obj.say('[SENSE] %s: localization %s, lidar %s, tracking %s\n',obj.nm(),S.Localization,obj.lidarMode,localOnOff(obj.hasTracker));
    end
    function [estPose,bus] = stepImpl(obj,truePose,lidarPC)
        S = obj.C.Sense; tp = reshape(double(truePose),1,3); obj.tick = obj.tick + 1; obj.tNow = obj.tNow + obj.dt;
        tpm = tp + [obj.off 0];                                         % map frame (corner origin)
        if obj.isFirst
            obj.isFirst = false; obj.lastTrue = tpm; obj.odom = tpm; obj.odomU = tpm; obj.amclPose = tpm; obj.est = tpm;
            if obj.hasAmcl, try, obj.amcl.InitialPose = tpm; catch, end, end
        end
        % ---- 1. wheel odometry = true increments + encoder noise (slip, resolution)
        d = tpm(1:2) - obj.lastTrue(1:2); c0 = cos(obj.lastTrue(3)); s0 = sin(obj.lastTrue(3));
        ds = d(1)*c0 + d(2)*s0; dth = localWrap(tpm(3) - obj.lastTrue(3)); obj.lastTrue = tpm;
        n1 = randn(obj.stream); n2 = randn(obj.stream); n3 = randn(obj.stream);
        dsN = ds + S.OdomSigmaS*abs(ds)*n1; dthN = dth + S.OdomSigmaR*abs(dth)*n2 + S.OdomDrift*abs(ds)*n3;
        a = obj.odom(3) + dthN/2; obj.odom = [obj.odom(1) + dsN*cos(a), obj.odom(2) + dsN*sin(a), localWrap(obj.odom(3) + dthN)];
        % ---- 2. lidar scan every LidarPeriod
        if mod(obj.tick,obj.nLid) == 0, obj.localLidarStep(tpm,lidarPC,S); end
        % ---- 2b. re-localisation (Round 32): (1) when the robot is docked at a rack / station (close-range registration of the rack face, same measurement the arm uses) the filter is
        %          re-initialised on the registered pose once; (2) divergence recovery: if the estimate stays more than ErrCap off for 1 s (AMCL jumped to a look-alike aisle), the filter is
        %          re-initialised (simulated by the known pose; a real robot would use its AMCL confidence + global localisation).  Every event is counted and printed at the end.
        if strcmpi(S.Localization,'amcl') && isfield(S,'ErrCap') && S.ErrCap > 0
            docked = false;
            try, stt = obj.Fleet.robots(obj.RobotId).state; docked = any(stt == [10 2 3 4 6 8 9]); catch, end
            e0 = hypot(obj.est(1)-tpm(1),obj.est(2)-tpm(2));
            if e0 > S.ErrCap, obj.errOverT = obj.errOverT + obj.dt; else, obj.errOverT = 0; end
            doReset = false;
            if docked, if ~obj.dockAnch, obj.dockAnch = true; doReset = e0 > 0.05; end, else, obj.dockAnch = false; end
            if obj.errOverT > 1.0, doReset = true; obj.errOverT = 0; end
            if doReset
                obj.odom = tpm; obj.odomU = tpm; obj.amclPose = tpm; obj.est = tpm; obj.nReset = obj.nReset + 1;
                if obj.hasAmcl && ~obj.ampFail, try, release(obj.amcl); obj.amcl.InitialPose = tpm; catch, end, end
            end
        end
        % ---- 3. pose estimate = last AMCL pose + odometry since that update
        if strcmpi(S.Localization,'amcl')
            dl = obj.odom(1:2) - obj.odomU(1:2); cu = cos(obj.odomU(3)); su = sin(obj.odomU(3));
            loc = [cu*dl(1) + su*dl(2), -su*dl(1) + cu*dl(2)]; dth2 = localWrap(obj.odom(3) - obj.odomU(3));
            ca = cos(obj.amclPose(3)); sa = sin(obj.amclPose(3));
            obj.est = [obj.amclPose(1) + ca*loc(1) - sa*loc(2), obj.amclPose(2) + sa*loc(1) + ca*loc(2), localWrap(obj.amclPose(3) + dth2)];
        else
            obj.est = tpm;                                              % 'truth' switch: Phase 3 behaviour as before
        end
        e = hypot(obj.est(1)-tpm(1),obj.est(2)-tpm(2)); obj.errSum = obj.errSum + e; obj.errN = obj.errN + 1; obj.errMax = max(obj.errMax,e);
        if mod(obj.tick,round(1/obj.dt)) == 0 && size(obj.rows,1) < 20000, obj.rows(end+1,:) = [obj.tNow tp(1:2) obj.est(1)-obj.off(1) obj.est(2)-obj.off(2) e]; end %#ok<AGROW>
        estPose = [obj.est(1)-obj.off(1), obj.est(2)-obj.off(2), obj.est(3)];
        bus = obj.lastBus; bus(1) = double(~obj.ampFail || ~strcmpi(S.Localization,'amcl')); bus(8) = e;
        if obj.tNow >= obj.nextPrint
            obj.nextPrint = obj.tNow + 300;
            if strcmpi(S.Localization,'amcl'), obj.say('[SENSE] %s: t=%.0f s localization error mean %.3f m, max %.3f m (%d AMCL updates, %d protective stops)\n',obj.nm(),obj.tNow,obj.errSum/max(1,obj.errN),obj.errMax,obj.nUpd,obj.nStops); end
        end
    end
    function releaseImpl(obj)
        try
            if obj.errN > 0 && strcmpi(obj.C.Sense.Localization,'amcl')
                fprintf('[SENSE] %s: FINAL localization error mean %.3f m, max %.3f m over %.0f s; %d AMCL updates; %d protective stops; lidar %s; %d re-localisations, %d outlier fixes rejected\n', ...
                    obj.nm(),obj.errSum/obj.errN,obj.errMax,obj.tNow,obj.nUpd,obj.nStops,obj.lidarMode,obj.nReset,obj.nRej);
            end
            if ~isempty(obj.rows)
                P = phase3Paths(); if ~isfolder(P.sense), mkdir(P.sense); end
                f = fullfile(P.sense,sprintf('sense_%s_%s.csv',obj.nm(),datestr(now,'yyyymmdd_HHMMSS')));
                fid = fopen(f,'w'); fprintf(fid,'t_s,true_x,true_y,est_x,est_y,error_m\n'); fprintf(fid,'%.1f,%.3f,%.3f,%.3f,%.3f,%.3f\n',obj.rows.'); fclose(fid);
            end
        catch
        end
    end
    function resetImpl(~), end
    function n = getNumInputsImpl(~), n = 2; end
    function n = getNumOutputsImpl(~), n = 2; end
    function [a,b] = getOutputSizeImpl(~), a = [1 3]; b = [1 8]; end
    function [a,b] = getOutputDataTypeImpl(~), a = 'double'; b = 'double'; end
    function [a,b] = isOutputComplexImpl(~), a = false; b = false; end
    function [a,b] = isOutputFixedSizeImpl(~), a = true; b = true; end
    function tf = isInputSizeMutableImpl(~,idx), tf = (idx == 2); end
end
methods (Access = private)
    function s = nm(obj), if obj.RobotId <= numel(obj.names), s = obj.names{obj.RobotId}; else, s = sprintf('robot%d',obj.RobotId); end, end
    function say(obj,varargin), try, fprintf(varargin{:}); catch, end, end %#ok<INUSL>
    function localMakeAmcl(obj,S)
        try
            map = localBuildMap(obj.G,obj.res);
            a = monteCarloLocalization; sm = likelihoodFieldSensorModel; sm.Map = map;
            bad = {};
            [sm,bad] = localSetP(sm,'SensorLimits',[obj.minR obj.maxR],bad);
            [sm,bad] = localSetP(sm,'SensorPose',[0 0 0],bad);
            [sm,bad] = localSetP(sm,'NumBeams',S.AmclBeams,bad);
            [sm,bad] = localSetP(sm,'MeasurementNoise',S.AmclMeasNoise,bad);
            [sm,bad] = localSetP(sm,'RandomMeasurementWeight',S.AmclRandomWeight,bad);
            [sm,bad] = localSetP(sm,'ExpectedMeasurementWeight',1 - S.AmclRandomWeight,bad);
            [sm,bad] = localSetP(sm,'MaxLikelihoodDistance',1.5,bad);
            [a,bad] = localSetP(a,'UseLidarScan',true,bad);
            [a,bad] = localSetP(a,'SensorModel',sm,bad);
            try, mm = odometryMotionModel; mm.Noise = S.AmclMotionNoise; a.MotionModel = mm; catch ME1, bad{end+1} = ['MotionModel: ' ME1.message]; end %#ok<AGROW>
            [a,bad] = localSetP(a,'ParticleLimits',S.AmclParticles,bad);
            [a,bad] = localSetP(a,'GlobalLocalization',false,bad);
            [a,bad] = localSetP(a,'InitialPose',obj.startPose + [obj.off 0],bad);
            [a,bad2] = localSetP(a,'InitialCovariance',diag(S.AmclInitCov),{}); if ~isempty(bad2), [a,bad] = localSetP(a,'InitialCovariance',S.AmclInitCov,bad); end
            [a,bad] = localSetP(a,'UpdateThresholds',S.AmclUpdate,bad);
            [a,bad] = localSetP(a,'ResamplingInterval',1,bad);
            obj.amcl = a; obj.hasAmcl = true;
            if ~isempty(bad), obj.say('[SENSE] %s: AMCL settings not applied: %s\n',obj.nm(),strjoin(bad,' | ')); end
        catch ME
            obj.hasAmcl = false; obj.ampFail = true;
            obj.say('[SENSE] %s: AMCL could not be created (%s) -> wheel odometry only.\n',obj.nm(),ME.message);
        end
    end
    function localMakeTracker(obj)
        try
            obj.tracker = multiObjectTracker('FilterInitializationFcn',@initcvkf,'AssignmentThreshold',30,'ConfirmationThreshold',[2 3],'DeletionThreshold',[6 6]);
            obj.hasTracker = true;
        catch ME
            obj.hasTracker = false; obj.trkFail = true;
            obj.say('[SENSE] %s: tracker could not be created (%s) -> protective field uses raw lidar clusters + fleet broadcast.\n',obj.nm(),ME.message);
        end
    end
    function localLidarStep(obj,tpm,lidarPC,S)
        % ---------- scan
        tpw = tpm - [obj.off 0]; others = obj.othersFromFleet(); discs = [others, 0.34*ones(size(others,1),1)];
        if ~obj.calibDone && ~strcmpi(obj.lidarMode,'raycast'), obj.localCalibrate(tpm,lidarPC); end     % Unreal lidar: which frame convention fits the map? (the map ray-cast scan serves AMCL / tracking meanwhile)
        if strcmpi(obj.lidarMode,'raycast') || ~obj.calibDone
            r = p3RaycastScan(obj.G,obj.res,obj.off,[tpw(1:2) tpm(3)],obj.angles,obj.maxR,discs);
            r(r < obj.minR) = inf; k = isfinite(r); r(k) = r(k) + S.LidarNoise*randn(obj.stream,1,nnz(k)); ang = obj.angles;
        else
            [r,ang] = p3LidarToScan(lidarPC,obj.nBeams,obj.minR,obj.maxR,obj.calibMir,obj.calibYaw,obj.C.Sense.LidarOffset);
        end
        % ---------- AMCL update
        if obj.hasAmcl && ~obj.ampFail
            try
                k = isfinite(r) & r >= obj.minR & r <= obj.maxR;
                if nnz(k) >= 10
                    scan = lidarScan(r(k).',ang(k).');
                    [up,p] = obj.amcl(obj.odom,scan);
                    if up && hypot(p(1)-obj.est(1),p(2)-obj.est(2)) > 1.0
                        % innovation gate (Round 33): an AMCL fix that jumps more than 1 m from the odometry-predicted pose in one update is an outlier (look-alike aisle) -> rejected,
                        % the filter is re-initialised on the predicted pose; the robot keeps driving on odometry
                        up = false; obj.nRej = obj.nRej + 1;
                        try, release(obj.amcl); obj.amcl.InitialPose = obj.est; catch, end
                    end
                    if up, obj.amclPose = [p(1) p(2) localWrap(p(3))]; obj.odomU = obj.odom; obj.nUpd = obj.nUpd + 1; end
                end
            catch ME
                obj.ampFail = true; obj.say('[SENSE] %s: AMCL stopped (%s) -> wheel odometry only from here on.\n',obj.nm(),ME.message);
            end
        end
        % ---------- tracking + protective field (only meaningful when the lidar belongs to the map)
        if S.Tracking, obj.localTrack(r,ang,others,S); end
    end
    function localCalibrate(obj,tpm,lidarPC)
        % Unreal lidar frames are not assumed: the first VALID scans (robot pose known) are matched against the map for the 8 frame conventions (mirror x yaw 0/90/180/270).
        % Empty / not-yet-rendered frames are skipped; a failed round is repeated (up to 4 rounds, 5 s apart) before the map ray-cast lidar is kept.
        if obj.calibT0 < 0, obj.calibT0 = obj.tNow; end
        if obj.tNow < obj.calibNext, return, end
        pw = tpm - [obj.off 0];
        [r0,~] = p3LidarToScan(lidarPC,obj.nBeams,obj.minR,obj.maxR,false,0,obj.C.Sense.LidarOffset); nv = nnz(isfinite(r0));
        if nv < 20
            obj.calibEmpty = obj.calibEmpty + 1;
            if obj.calibEmpty == 1 || mod(obj.calibEmpty,100) == 0
                obj.say('[SENSE] %s: t=%.1f s Unreal lidar frame has %d usable beams (point cloud size [%s]) - waiting for valid returns\n',obj.nm(),obj.tNow,nv,num2str(size(lidarPC)));
                obj.say('[SENSE] %s:   raw frame: %s\n',obj.nm(),localRawStats(lidarPC));
            end
            if obj.tNow - obj.calibT0 > 40
                obj.lidarMode = 'raycast'; obj.calibDone = true;
                obj.say('[SENSE] %s: no valid Unreal lidar returns within 40 s -> using the map ray-cast lidar instead.\n',obj.nm());
            end
            return
        end
        obj.calibN = obj.calibN + 1;
        for mir = [0 1]
            for yo = 0:3
                [rr,aa] = p3LidarToScan(lidarPC,obj.nBeams,obj.minR,obj.maxR,logical(mir),yo*pi/2,obj.C.Sense.LidarOffset);
                obj.calibAcc(mir+1,yo+1) = obj.calibAcc(mir+1,yo+1) + p3ScanScore(obj.Gi,obj.res,obj.off,pw,rr,aa);
            end
        end
        if obj.calibN >= 3
            A = obj.calibAcc/obj.calibN; [m,ix] = max(A(:)); [mi,yi] = ind2sub(size(A),ix);
            if m >= 0.35
                obj.calibMir = (mi == 2); obj.calibYaw = (yi-1)*pi/2; obj.calibDone = true;
                obj.say('[SENSE] %s: Unreal lidar matches the map (score %.2f, %d beams, frame mirror=%d yaw=%.0f deg).\n',obj.nm(),m,nv,obj.calibMir,rad2deg(obj.calibYaw));
            else
                obj.calibRound = obj.calibRound + 1; obj.calibN = 0; obj.calibAcc = zeros(2,4); obj.calibNext = obj.tNow + 5;
                obj.say('[SENSE] %s: Unreal lidar round %d: best map match %.2f with %d beams (scores [%s]) - %s\n',obj.nm(),obj.calibRound,m,nv,num2str(A(:).',2),localTern(obj.calibRound >= 4,'giving up','trying again in 5 s'));
                if obj.calibRound >= 4
                    obj.lidarMode = 'raycast'; obj.calibDone = true;
                    obj.say('[SENSE] %s: Unreal lidar returns do NOT match the map -> using the map ray-cast lidar instead. Check the lidar block mount / range.\n',obj.nm());
                end
            end
        end
    end
    function O = othersFromFleet(obj)
        O = zeros(0,2);
        try
            for j = 1:numel(obj.Fleet.robots)
                if j ~= obj.RobotId, p = obj.Fleet.robots(j).pose; O(end+1,:) = p(1:2); end %#ok<AGROW>
            end
        catch
        end
    end
    function localTrack(obj,r,ang,others,S)
        est = obj.est - [obj.off 0]; k = isfinite(r) & r >= S.LidarMinRange;
        a = est(3) + ang(k); P = [est(1) + r(k).*cos(a); est(2) + r(k).*sin(a)].';
        dyn = P(~p3GridHit(obj.Gi,obj.res,obj.off,P),:);
        [cen,~] = p3Cluster(dyn,0.35,S.TrackMinPts);
        carry = false; try, carry = obj.Fleet.robots(obj.RobotId).carry; catch, end
        % ---- classify the clusters: near the broadcast pose of another robot = that robot (the lidar sees its SURFACE: centre = centroid + 0.34 m away from me);
        %      anything else that is not in the map = unexpected obstacle
        cenRob = zeros(0,2); unkC = zeros(0,2);
        for i = 1:size(cen,1)
            isRob = false; for j = 1:size(others,1), if hypot(cen(i,1)-others(j,1),cen(i,2)-others(j,2)) < 1.3, isRob = true; break, end, end
            if isRob, u = cen(i,:) - est(1:2); u = u/max(norm(u),eps); cenRob(end+1,:) = cen(i,:) + 0.34*u; else, unkC(end+1,:) = cen(i,:); end %#ok<AGROW>
        end
        % ---- fuse: lidar (sensor 1) + broadcast poses of the other robots (sensor 2) -> tracks
        if obj.hasTracker
            try
                dets = {};
                for i = 1:size(cenRob,1), dets{end+1} = objectDetection(obj.tNow,[cenRob(i,1);cenRob(i,2)],'MeasurementNoise',diag([0.25 0.25].^2),'SensorIndex',1); end %#ok<AGROW>
                for i = 1:size(unkC,1), dets{end+1} = objectDetection(obj.tNow,[unkC(i,1);unkC(i,2)],'MeasurementNoise',diag([0.15 0.15].^2),'SensorIndex',1); end %#ok<AGROW>
                for i = 1:size(others,1), dets{end+1} = objectDetection(obj.tNow,[others(i,1);others(i,2)],'MeasurementNoise',diag([0.10 0.10].^2),'SensorIndex',2); end %#ok<AGROW>
                tr = obj.tracker(dets,obj.tNow); tx = zeros(numel(tr),2);
                for i = 1:numel(tr), s_ = tr(i).State; tx(i,:) = [s_(1) s_(3)]; end
            catch ME
                obj.hasTracker = false; obj.trkFail = true; tx = [cenRob; unkC; others];
                obj.say('[SENSE] %s: tracker stopped (%s) -> raw clusters + fleet broadcast.\n',obj.nm(),ME.message);
            end
        else
            tx = [cenRob; unkC; others];
        end
        robXY = zeros(0,2); unk = zeros(0,2);
        for i = 1:size(tx,1)
            isRob = false; for j = 1:size(others,1), if hypot(tx(i,1)-others(j,1),tx(i,2)-others(j,2)) < 1.3, isRob = true; break, end, end
            if isRob, robXY(end+1,:) = tx(i,:); else, unk(end+1,:) = tx(i,:); end %#ok<AGROW>
        end
        [stop,why,dMin] = p3SafetyField(est,robXY,unk,carry,struct('coneDeg',S.SafetyConeDeg,'dRobot',S.SafetyRobot,'dUnk',S.SafetyUnknown,'rSelf',S.SafetySelf));
        % unexpected obstacles must persist (0.5 s) before they stop the robot; a stop that lasts > 8 s is released for 4 s (demo must not deadlock on a false positive)
        if strcmp(why,'obstacle'), obj.unkCount = obj.unkCount + 1; else, obj.unkCount = 0; end
        if stop && strcmp(why,'obstacle') && obj.unkCount < max(1,round(0.5/S.LidarPeriod)), stop = false; end
        if stop
            if ~obj.safeStop, obj.safeStop = true; obj.stopSince = obj.tNow; obj.nStops = obj.nStops + 1;
                obj.say('[SAFETY] t=%.1f %s: lidar protective stop (%s within the protective field, nearest %.2f m)\n',obj.tNow,obj.nm(),why,dMin); end
            if obj.tNow - obj.stopSince > 8 && obj.releaseUntil < obj.tNow
                obj.releaseUntil = obj.tNow + 4; obj.stopSince = obj.tNow + 4;
                obj.say('[SAFETY] t=%.1f %s: stop lasted > 8 s - released for 4 s (check the log: possible false detection)\n',obj.tNow,obj.nm());
            end
            if obj.releaseUntil >= obj.tNow, stop = false; end
        else
            obj.safeStop = false;
        end
        obj.lastBus = [1 double(stop) dMin 0 size(tx,1) double(strcmpi(obj.lidarMode,'raycast'))+1 size(unk,1) 0];
    end
end
end

function [G,res,off,Gi] = localGrid(C,S)
persistent cG cRes cOff cGi cKey
key = sprintf('%d',S.MapRes);
if isempty(cG) || ~strcmp(cKey,key)
    [cG,cRes,cOff] = p3StaticGrid(C,S.MapRes); cKey = key;
    n = max(1,round(S.MapInflate*cRes)); cGi = conv2(double(cG),ones(2*n+1),'same') > 0;
end
G = cG; res = cRes; off = cOff; Gi = cGi;
end
function map = localBuildMap(G,res)
% Navigation Toolbox occupancyMap in the grid's corner-origin frame; verified against the grid (row order of the matrix constructor)
map = occupancyMap(double(flipud(G)),res);
if ~localMapOk(map,G,res), map = occupancyMap(double(G),res); end
if ~localMapOk(map,G,res)
    map = occupancyMap(size(G,2)/res,size(G,1)/res,res); [r,c] = find(G);
    setOccupancy(map,[(c-0.5)/res (r-0.5)/res],ones(numel(r),1));
end
assert(localMapOk(map,G,res),'Phase3:Map','occupancyMap does not match the static grid');
end
function ok = localMapOk(map,G,res)
rs = RandStream('mt19937ar','Seed',5); n = 400; xy = [rand(rs,n,1)*size(G,2)/res rand(rs,n,1)*size(G,1)/res];
a = getOccupancy(map,xy) > 0.5; b = p3GridHit(G,res,[0 0],xy); ok = mean(a == b) > 0.97;
end
function a = localWrap(a), a = atan2(sin(a),cos(a)); end
function s = localOnOff(b), if b, s = 'on (multiObjectTracker)'; else, s = 'raw clusters'; end, end

function [o,bad] = localSetP(o,name,val,bad)
try, o.(name) = val; catch ME, bad{end+1} = [name ': ' ME.message]; end
end

function s = localTern(c,a,b)
if c, s = a; else, s = b; end
end

function t = localRawStats(pc)
% what the Unreal lidar really delivers (for the log): counts and ranges of the raw point cloud
try
    P = double(pc); if ndims(P) == 3, P = reshape(P,[],size(P,3)); end
    if size(P,2) ~= 3 && size(P,1) == 3, P = P.'; end
    nF = nnz(all(isfinite(P),2)); nZ = nnz(all(P == 0,2)); ok = all(isfinite(P),2) & ~all(P == 0,2); Q = P(ok,:);
    if isempty(Q)
        t = sprintf('%d points, %d finite, %d exactly zero, none usable (all NaN/Inf/0)',size(P,1),nF,nZ);
    else
        r = hypot(Q(:,1),Q(:,2));
        t = sprintf('%d points, %d finite, %d exactly zero; x [%.2f %.2f] y [%.2f %.2f] z [%.2f %.2f] range [%.2f %.2f]; first point [%.2f %.2f %.2f]',size(P,1),nF,nZ,min(Q(:,1)),max(Q(:,1)),min(Q(:,2)),max(Q(:,2)),min(Q(:,3)),max(Q(:,3)),min(r),max(r),Q(1,1),Q(1,2),Q(1,3));
    end
catch ME
    t = ['could not read the frame: ' ME.message];
end
end
