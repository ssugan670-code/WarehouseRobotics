classdef Phase3Fleet < handle
%PHASE3FLEET  Fleet manager for the multi-robot warehouse (pure MATLAB logic, no Simulink inside).
%   One object lives in the base workspace (Phase3Fleet).  The robot controllers talk to it through exchange(k,report) every 0.1 s,
%   the command window talks to it through p3.m.  It owns:
%     * the slot database (every shelf slot: FREE / OCCUPIED / RES_IN / RES_OUT / STATIC / UNKNOWN) and the carton pool
%     * tasks (putaway dock->shelf, retrieve shelf->station, cross-dock dock->station), orders, transactional reservations
%     * zone locks (shelf bay, station) - a task is only assigned when its source bay is free; the destination zone is taken before the
%       robot leaves the source bay.  Lock order (source bay -> destination) cannot form a cycle, so no lock deadlock.
%     * traffic: priority-based look-ahead conflict check -> speed scale / hold per robot, wait-for graph, replan request after a long hold
%     * metrics, event log, slot snapshot files, the camera lease for vision verification
%   Coordinates: right-handed warehouse frame (the same one the Phase 2 controller uses).
properties
    C; N = 0; M = 0;
    robots; slots; stations; cartons; tasks; orders; locks = cell(0,2); questions;
    t = 0; lastUpdate = -1; verbose = 1;
    watch = 0; recordVideo = false; camSel = 1; visionMode = 'on'; visionCount = [0 0 0 0];
    autoOn = false; autoPeriod = 20; autoNext = inf; autoArrive = 70; autoArriveNext = inf; autoStation = 0;
    finishing = false; allDone = false; stopFlag = false;
    camOv = cell(1,6);
    outDir = ''; dirs = struct(); stamp = ''; evlog = cell(0,3);
    dwell = 25; answerMode = 'ask';
    shipReq = false(1,8); map2D = true; mapEvery = 1; nextAssign = 0; assignGap = 4; nextBayStart = 0; bayGap = 10;      % robots leave their bays one after the other, bayGap seconds apart
    faultTries = zeros(0,2); excl = zeros(0,2);      % fault recovery: [task tries], [task robot] (a robot that failed a task does not get the same task again at once)
    nextTask = 1; nextOrder = 1; lcg = 12345;
    minSep = inf; nearMiss = 0; collisions = 0; deadlocks = 0; pullOvers = 0; replans = 0; sepFlag = false; colFlag = false; shipped = 0;
    grantTo = 0; grantSeq = 0; grantT = -inf;    % planning token: one robot at a time plans its route (first come = first route, the others plan around it)
    planner = 'mstar';       % 'mstar' = individual shortest routes + M* coupling of the robots whose routes collide (fleetMStar) | 'prioritised' = plan-time negotiation (fleetPlanRoute), the verified earlier planner
    gridFree = []; mgrid = []; mSig = []; mLast = -inf; mTmax = 2.5; mMaxGroup = 4; mMaxNodes = 10000; mFail = 0;      % M*: coarse planning grid, trigger state, time limit per run [s]
    mstat = struct('runs',0,'coupled',0,'pushed',0,'fail',0,'maxGroup',1,'nodes',0,'time',0,'maxTime',0,'reason','','robotsCoupled',0,'timed',0);
    freeFcn = []; planCount = 0; waitFor = zeros(0,2); bayPitch = 1.0; stSep = 3.2; faceCapPitch = 4.3; faceSep = 3.2; blkCache = []; vLastSel = []; vSelT = -9; massBase = 35; massArm = 8.2; massBox = 4; crr = 0.02; etaDrive = 0.75; etaLift = 0.6; pArm = 36; pBase = 30; pendStop = 4.5; dstStop = 5.5; strips = zeros(0,4); yieldHold = zeros(0); yieldT = 12; prec = false(0); precT = zeros(0); precNew = 0;
    TP = struct();           % traffic parameters (all derived from the robot radius - change them in ONE place, see localTrafficParams)
    DockShift = 0;
    autoFinished = false; parkT = inf; keepStations = false;     % automatic mode ends by itself: every station full, staging empty, robots home -> 3-D window closes 5 s later
    nPutAuto = 0; nRetAuto = 0; waveAt = inf; autoBase = 0; autoPerRobot = 3;     % automatic mode: balance putaway / retrieve 50:50, new delivery wave when every staging stand is empty
    mode = 'manual';         % 'manual' = every command runs at once | 'batch' = commands are collected, p3 go starts them | 'auto' = the system generates the work
    autoPutaway = false;     % manual mode: robots stay parked until a command (p3 putaway / p3 order / p3 <robot> ...) gives them a task
end
methods
    % ------------------------------------------------------------------ construction
    function obj = Phase3Fleet(C,nRobots,varargin)
        p = inputParser; addParameter(p,'answer','ask'); addParameter(p,'dwell',25); addParameter(p,'vision','on'); addParameter(p,'verbose',1); addParameter(p,'planner','mstar');
        parse(p,varargin{:});
        obj.C = C; obj.N = nRobots; obj.answerMode = p.Results.answer; obj.dwell = p.Results.dwell; obj.visionMode = p.Results.vision; obj.verbose = p.Results.verbose; obj.planner = lower(char(p.Results.planner));
        P3 = C.P3; obj.M = P3.nCartons; obj.TP = Phase3Fleet.localTrafficParams(C);
        if strcmp(obj.planner,'mstar'), obj.TP.plan.maxTries = 0; end      % M*: the controllers plan their INDIVIDUAL shortest route (the individual policy); the coupling is done by M* (mstarStep)
        for k = 1:obj.N
            r = P3.robots(k);
            if k==1, RB = Phase3Fleet.newRobot(r.id,r.name,r.colour,r.home); else, RB(k) = Phase3Fleet.newRobot(r.id,r.name,r.colour,r.home); end
        end
        obj.robots = RB;
        for i = 1:numel(P3.slots)
            s = P3.slots(i); s.state = 'FREE'; s.item = 0; s.by = 0; s.task = 0;
            if any(P3.static == i), s.state = 'STATIC'; end
            if i==1, SL = s; else, SL(i) = s; end
        end
        obj.slots = SL; obj.prec = false(nRobots); obj.precT = zeros(nRobots);
        pit = inf; for q = 1:numel(SL), if SL(q).shelf > 0 && SL(q).level == 1 && strcmp(SL(q).side,'A') && SL(q).shelf == SL(1).shelf, for q2 = q+1:numel(SL), if SL(q2).shelf == SL(q).shelf && SL(q2).level == 1 && strcmp(SL(q2).side,'A'), pit = min(pit,abs(SL(q2).xyz(2)-SL(q).xyz(2))); end, end, end, end
        if isfinite(pit) && pit > 0.2, obj.bayPitch = pit; end       % bay length = spacing of neighbouring slots on one face
        for t_ = 1:numel(C.Stations)
            st = struct('id',t_,'name',C.Stations(t_).name,'state',{repmat({'FREE'},1,size(C.Stations(t_).slotXYZ,1))},'item',zeros(1,size(C.Stations(t_).slotXYZ,1)),'since',zeros(1,size(C.Stations(t_).slotXYZ,1)),'by',zeros(1,size(C.Stations(t_).slotXYZ,1)));
            if t_==1, STT = st; else, STT(t_) = st; end
        end
        obj.stations = STT;
        for j = 1:obj.M
            c = P3.cartons(j); c.task = 0; c.drive = true; c.spawnUntil = 0; c.cmd = c.park; c.stationSlot = [0 0]; c.lost = false; c.cmdYaw = c.yaw;
            if strcmp(c.state,'RESERVE'), c.drive = true; c.cmd = c.park;
            else, c.drive = false; c.cmd = c.xyz; end
            if strcmp(c.state,'DOCK'), c.drive = true; c.cmd = c.xyz; end     % cartons on the staging stands are held at their stand until a robot picks them
            if any(j == P3.missing), c.drive = false; c.cmd = c.park; end
            if c.slot > 0, obj.slots(c.slot).state = 'OCCUPIED'; obj.slots(c.slot).item = j; end
            if j==1, CT = c; else, CT(j) = c; end
        end
        obj.cartons = CT;
        obj.tasks = Phase3Fleet.emptyTasks(); obj.orders = Phase3Fleet.emptyOrders();
        obj.questions = struct('id',{},'robot',{},'task',{},'slot',{},'text',{});
        obj.stamp = datestr(now,'yyyymmdd_HHMMSS');
        pth = phase3Paths(); obj.outDir = pth.out; obj.dirs = pth; for q_ = {pth.out,pth.metrics,pth.tasks,pth.events,pth.summary,pth.logs,pth.slots,pth.sense,pth.cam,pth.diag}, if ~isfolder(q_{1}), mkdir(q_{1}); end, end
        obj.ev('FLEET',0,sprintf('fleet created: %d robots, %d slots, %d cartons',obj.N,numel(obj.slots),obj.M));
    end

    % ------------------------------------------------------------------ robot <-> fleet
    function cmd = exchange(obj,k,rep)
        r = obj.robots(k);
        obj.robots(k).rep = true; obj.robots(k).repT = rep.t;
        obj.robots(k).pose = reshape(rep.pose,1,3); obj.robots(k).state = rep.state; obj.robots(k).carry = logical(rep.carry);
        obj.robots(k).wp = rep.wp; obj.robots(k).goal = rep.goal; obj.robots(k).jobSeq = rep.jobSeq; obj.robots(k).jobStatus = rep.jobStatus;
        obj.robots(k).scanReq = logical(rep.scanReq); obj.robots(k).faultFlag = logical(rep.fault);
        if isfield(rep,'planReq')
            if rep.planReq && ~obj.robots(k).planReq, obj.robots(k).planReqT = rep.t; end
            obj.robots(k).planReq = logical(rep.planReq); obj.robots(k).planSeq = rep.planSeq;
        end
        obj.update(rep.t);
        r = obj.robots(k);
        cmd = struct('job',[],'jobSeq',r.jobSeqSent,'scale',r.scale,'hold',r.hold || r.lockHold || r.pendHold || r.stopNow,'replanSeq',r.replanSeq, ...
            'others',obj.othersFor(k),'routes',obj.routesFor(k),'finish',r.finish,'vision',r.vis,'name',r.name, ...
            'planGrant',obj.grantTo == k,'stampPos',obj.TP.stampPos,'stampRoute',obj.TP.stampRoute,'planPar',obj.TP.plan,'evade',r.evade,'recover',r.recoverSeq,'dest',r.recoverDest,'abort',r.abortSeq,'mroute',r.mroute);
        cmd.dstUpd = struct('seq',r.dstUpdSeq,'dst',r.dstUpd);
        if r.jobSeqSent > rep.jobSeq, cmd.job = r.job; end
    end

    function planGrants(obj)
        % ONE robot at a time may plan a route.  The next robot plans only after the previous one has published its route, so every route is planned
        % around ALL routes that already exist (prioritised planning: first planner = first choice; working / carrying robots plan first).
        if obj.grantTo > 0
            r = obj.robots(obj.grantTo);
            if r.planSeq > obj.grantSeq || (~r.planReq && obj.t - obj.grantT > 0.3) || obj.t - obj.grantT > 2.0, obj.grantTo = 0; end
        end
        if obj.grantTo == 0
            c = find([obj.robots.planReq] & [obj.robots.rep]); if isempty(c), return, end
            best = c(1); bp = -inf;
            for q = c, v = obj.prio(q) - 1e-6*obj.robots(q).planReqT; if v > bp, bp = v; best = q; end, end
            obj.grantTo = best; obj.grantSeq = obj.robots(best).planSeq; obj.grantT = obj.t; obj.planCount = obj.planCount + 1; obj.robots(best).planNo = obj.planCount;
        end
    end
    function O = othersFor(obj,k)
        % positions of the robots that do NOT move (parked, working at a rack, stopped): they are obstacles for the new route.  Robots that drive
        % are handled in space AND time through their committed routes (routesFor).
        O = zeros(0,3); p = obj.robots(k).pose;
        for j = 1:obj.N
            if j == k || ~obj.robots(j).rep, continue, end
            moving = any(obj.robots(j).state == [1 5 7]) && obj.robots(j).scale >= 0.5 && ~isempty(obj.robots(j).wp) && ~obj.robots(j).pendHold;
            if moving, continue, end
            if norm(obj.robots(j).pose(1:2) - p(1:2)) < 40, wk = any(obj.robots(j).state == [10 2 3 4 6 8 9 11]); O(end+1,:) = [obj.robots(j).pose(1:2), obj.TP.armExtra*wk + obj.TP.carryExtra*(obj.robots(j).carry || (wk && any(obj.robots(j).state == [10 2 3]) && obj.robots(j).task > 0)) + obj.TP.stowExtra*(~wk && ~obj.robots(j).carry)]; end %#ok<AGROW>
        end
    end
    function Rt = routesFor(obj,k)
        % committed routes of every other robot that is driving: rows [x y eta id carry], a point about every 0.75 m, up to 40 m, eta = seconds from now
        % at that robot's cruise speed.  A new route is checked against them in space and time (fleetPlanRoute): robots that pass before / after each
        % other do not disturb each other; only real crossings at the same time are avoided.
        Rt = zeros(0,6); pk = obj.robots(k).pose(1:2); TP = obj.TP;
        for j = 1:obj.N
            if j == k || ~obj.robots(j).rep, continue, end
            r = obj.robots(j); if r.pendHold, continue, end        % a robot that waits for a shelf face stands still: it is an obstacle (othersFor), not a route
            if r.task > 0 && ~isempty(r.job) && (isempty(r.wp) || ~any(r.state == [1 5 7])) && any(r.state == [0 1 5 7])
                % robot has a task but no route yet (about to plan): its start point is reserved as the place where it will work
                try, g = r.job.src.staging(1:2); if r.state == 5, g = r.job.dst.staging(1:2); end
                    d0 = norm(g - pk); Rt = [Rt; [repmat(g(:).',20,1), d0/TP.vFree + 3*(0:19).', repmat(j,20,1), zeros(20,1), ones(20,1)]]; %#ok<AGROW>
                catch, end
                continue
            end
            if ~any(r.state == [1 5 7]) || isempty(r.wp), continue, end
            v = TP.vFree + (TP.vCarry - TP.vFree)*r.carry;
            R = [r.pose(1:2); r.wp]; acc = 0; q = zeros(0,6);
            for a = 1:size(R,1)-1
                d = norm(R(a+1,:) - R(a,:)); n = max(1,ceil(d/0.75));
                for u = 1:n
                    acc = acc + d/n; q(end+1,:) = [R(a,:) + (R(a+1,:) - R(a,:))*u/n, acc/v, j, r.carry, 0]; %#ok<AGROW>
                    if acc > 40, break, end
                end
                if acc > 40, break, end
            end
            if ~isempty(q) && acc <= 40       % the robot arrives and then WORKS there (arm, grasp, place ...): its destination stays occupied for a long time
                te = q(end,3); q = [q; [repmat(q(end,1:2),20,1), te + 3*(1:20).', repmat(j,20,1), repmat(r.carry,20,1), ones(20,1)]]; %#ok<AGROW>
            end
            q = q(vecnorm(q(:,1:2) - pk,2,2) < 60,:);
            Rt = [Rt; q]; %#ok<AGROW>
        end
    end
    function tf = hasWay(obj,j,k)
        % right of way: working (arm at a rack) > carrying a box > holding a task > waited longer > lower number.  Strict total order -> two robots never both yield (no flip-flop).
        tf = obj.prio(j) > obj.prio(k);
    end
    function p = prio(obj,k)
        r = obj.robots(k); man = r.rep && any(r.state == [10 2 3 4 6 8 9 11]);
        p = 100*man + 10*r.carry + 0.5*r.boost + 1e-3*(1e6 - r.planNo) + 1e-9*(obj.N-k);         % working first, then a robot carrying a box (it cannot back up / swerve), then the robot that planned its route EARLIER (planning order = priority 1, 2, 3 ...)
    end

    % ------------------------------------------------------------------ main step (called by whichever controller reports first each 0.1 s)
    function update(obj,t)
        if t < obj.lastUpdate + 0.095, return, end
        dt = t - obj.lastUpdate; if obj.lastUpdate < 0, dt = 0.1; end
        obj.t = t; obj.lastUpdate = t;
        obj.progress();
        obj.watchdog(dt);
        obj.locksRelease();
        obj.housekeeping();
        obj.autoGen();
        obj.makeTasks();
        obj.assign();
        obj.locksRequest();
        obj.planGrants();
        obj.mstarStep();
        obj.traffic(dt);
        obj.visionStep();
        obj.metricsStep(dt);
        obj.autoEnd();
        obj.checkDone();
        if obj.map2D, fleetMap2D(obj); end
    end

    % ------------------------------------------------------------------ job progress -> database
    function progress(obj)
        for k = 1:obj.N
            r = obj.robots(k); if ~r.rep, continue, end
            tid = r.task;
            if r.fault && r.recovering && r.state ~= 11 && ~r.faultFlag          % the robot has left FAULT_HOLD: it works again
                obj.robots(k).fault = false; obj.robots(k).recovering = false; r = obj.robots(k);
                obj.ev('FAULT',k,sprintf('%s recovered',r.name)); obj.say('[FLEET] %s has recovered from the fault and works again.\n',r.name);
            end
            if r.state == 11 || r.faultFlag
                if ~r.fault
                    obj.robots(k).fault = true; obj.robots(k).faultT = obj.t; obj.ev('FAULT',k,sprintf('%s is in FAULT_HOLD',r.name));
                    rec = false;
                    if tid > 0, rec = obj.recoverFault(tid,k);
                    else, obj.robots(k).recoverSeq = r.recoverSeq + 1; obj.robots(k).recoverDest = []; rec = ~r.carry;       % a fault without a task: the robot folds its arm back and goes home
                    end
                    obj.robots(k).recovering = rec;
                    if rec
                        obj.say('[FLEET] %s reported a FAULT (see the [FAULT] lines above) - recovery started, the other robots avoid it meanwhile.\n',r.name);
                    else
                        obj.say('[FLEET] %s is in FAULT_HOLD (see the [FAULT] lines above). Its task is cancelled; the other robots avoid it.\n',r.name);
                        if tid > 0, obj.failTask(tid,k,'robot fault'); end
                    end
                elseif r.recovering && obj.t - r.faultT > 60          % the controller did not take the recovery: give up, as before
                    obj.robots(k).recovering = false; obj.say('[FLEET] %s did not recover within 60 s - FAULT_HOLD, its task is cancelled.\n',r.name);
                    if tid > 0, obj.failTask(tid,k,'robot fault'); end
                end
                continue
            end
            if tid == 0, continue, end
            if r.jobSeq < r.jobSeqSent, continue, end            % job not yet accepted by the controller
            T = obj.tasks(Phase3Fleet.tix(obj.tasks,tid));
            if r.state >= 1 && strcmp(T.status,'ACTIVE'), obj.setTask(tid,'status','RUNNING'); obj.setTask(tid,'started',obj.t); end
            if (r.state == 4 || r.state == 5 || r.state == 6) && ~r.srcFreed
                si = T.src; obj.slots(si).state = 'FREE'; obj.slots(si).item = 0; obj.slots(si).by = 0; obj.slots(si).task = 0;
                j = T.item; obj.cartons(j).state = 'MOVING'; obj.cartons(j).slot = 0; obj.cartons(j).drive = false;
                obj.robots(k).srcFreed = true;
                obj.robots(k).stats.eLift = obj.robots(k).stats.eLift + (obj.massArm + obj.massBox)*9.81*max(0,obj.slots(si).lift)/obj.etaLift;      % raising arm + box to the shelf level (lowering is not recovered)
                obj.ev('PICK',k,sprintf('%s picked C%d from %s',r.name,j,obj.slots(si).id));
            end
            switch r.jobStatus
                case 'done',   obj.finishTask(tid,k);
                case 'absent', obj.absentTask(tid,k);
            end
        end
    end

    function ok = recoverFault(obj,tid,k)
        % A robot reported a fault (or the watchdog stopped it).  The robot ALWAYS gets a way out - it never stays in FAULT_HOLD for good:
        % (1) carton in the gripper: another destination (the failed slot is blocked), the robot backs off from the rack and drives there.  Up to 8 different destinations.
        % (2) no carton in the gripper: the robot folds its arm back and goes home.  The task goes back to the queue (not to the same robot at once); a task that has
        %     failed 3 times, or whose carton is not in the rack any more, is given up (FAILED) - the robot is free for other work in both cases.
        ok = false; r = obj.robots(k); ti = Phase3Fleet.tix(obj.tasks,tid); T = obj.tasks(ti); j = T.item;
        q = find(obj.faultTries(:,1) == tid,1); n = 0; if ~isempty(q), n = obj.faultTries(q,2); end
        if r.carry && n >= 8, obj.say('[FLEET] %s: 8 recoveries of this task failed - the carton stays in the gripper, operator needed.\n',r.name); return, end
        if isempty(q), obj.faultTries(end+1,:) = [tid 1]; else, obj.faultTries(q,2) = n + 1; end
        if r.carry
            % ---- another destination for the carton that is in the gripper
            oldKeys = r.dstKeys; oldD = T.dstSlot; oldS = T.dstStation;
            obj.releaseDst(T);
            if T.dstKind == 1
                obj.slots(oldD).state = 'UNKNOWN'; obj.slots(oldD).by = 0; obj.slots(oldD).task = 0;     % the robot could not put a carton there: blocked (clear it with  p3 slot <id> free )
                d = obj.nearestFreeSlot(r.pose(1:2),k,oldD);
                if d == 0, obj.slots(oldD).state = 'RES_IN'; obj.slots(oldD).by = k; obj.slots(oldD).task = tid; obj.say('[FLEET] %s: no other free slot for the carton - recovery impossible.\n',r.name); return, end
                obj.slots(d).state = 'RES_IN'; obj.slots(d).by = k; obj.slots(d).task = tid;
                obj.tasks(ti).dstSlot = d; newKeys = obj.bayKeys(d); newName = obj.slots(d).id;
            else
                bs = 0; bq = 0; bestc = inf;
                for st = 1:numel(obj.stations)
                    for qq = find(strcmp(obj.stations(st).state,'FREE'))
                        if st == oldS && qq == oldD, continue, end
                        if ~obj.stFree(st,qq,k), continue, end
                        c = norm(obj.C.Stations(st).approach(qq,:) - r.pose(1:2)) + 3*obj.stationLoad(st);
                        if c < bestc, bestc = c; bs = st; bq = qq; end
                    end
                end
                if bs == 0, obj.stations(oldS).state{oldD} = 'RES'; obj.stations(oldS).by(oldD) = k; obj.say('[FLEET] %s: no other free station place - recovery impossible.\n',r.name); return, end
                st = obj.stations(bs); st.state{bq} = 'RES'; st.by(bq) = k; obj.stations(bs) = st;
                obj.tasks(ti).dstSlot = bq; obj.tasks(ti).dstStation = bs; newKeys = obj.stKeys(bs,bq); newName = sprintf('%s slot %d',st.name,bq);
            end
            obj.lockDrop(k,oldKeys); obj.lockTake(newKeys,k); obj.robots(k).dstKeys = newKeys; obj.robots(k).dstLockOk = true; obj.robots(k).lockWait = 0;
            job = obj.makeJob(obj.robots(k).jobSeqSent,obj.tasks(ti)); obj.robots(k).dstPt = job.dst.approach(1:2);
            obj.robots(k).recoverSeq = r.recoverSeq + 1; obj.robots(k).recoverDest = job.dst;
            obj.ev('RECOVER',k,sprintf('task %d: new destination %s',tid,newName));
            obj.say('[FLEET] t=%.0f s  %s: new destination for C%d -> %s\n',obj.t,r.name,j,newName);
        else
            % ---- no carton in the gripper: the robot goes home; the task goes back to the queue (or is given up)
            if r.srcFreed || n >= 3
                obj.lockDrop(k); obj.robots(k).srcKeys = {}; obj.robots(k).dstKeys = {}; obj.robots(k).dstLockOk = false; obj.robots(k).srcLockOk = false; obj.robots(k).pendHold = false; obj.robots(k).holdPt = [];
                obj.failTask(tid,k,'given up after robot faults');
                why = 'failed 3 times'; if r.srcFreed, why = 'the carton is not in its rack slot any more'; end
                obj.say('[FLEET] t=%.0f s  %s: task %d is given up (%s); the robot folds its arm back and goes home.\n',obj.t,r.name,tid,why);
            else
                obj.releaseDst(T);
                si = T.src; if strcmp(obj.slots(si).state,'RES_OUT'), obj.slots(si).state = 'OCCUPIED'; obj.slots(si).by = 0; obj.slots(si).task = 0; end
                obj.cartons(j).task = tid;
                obj.setTask(tid,'status','QUEUED'); obj.setTask(tid,'robot',0); obj.setTask(tid,'note','back in the queue after a robot fault');
                obj.excl(end+1,:) = [tid k];
                obj.lockDrop(k); obj.robots(k).srcKeys = {}; obj.robots(k).dstKeys = {}; obj.robots(k).dstLockOk = false; obj.robots(k).srcLockOk = false; obj.robots(k).pendHold = false; obj.robots(k).holdPt = []; obj.robots(k).task = 0;
                obj.ev('RECOVER',k,sprintf('task %d back in the queue',tid));
                obj.say('[FLEET] t=%.0f s  %s: task %d goes back to the queue, the robot folds its arm back and goes home.\n',obj.t,r.name,tid);
            end
            obj.robots(k).recoverSeq = r.recoverSeq + 1; obj.robots(k).recoverDest = [];
        end
        ok = true;
    end
    function watchdog(obj,dt)
        % A robot with a task that makes no progress for too long is stopped by the fleet (so no robot can stand at a rack for ever, whatever the reason):
        %   arm states (dock / extend / grasp / retract / place): longer than the limit in the same state -> abort = the controller raises a FAULT, then the normal recovery runs.
        %   driving states (to the rack / to the station): standing still for 90 s without being held back by another robot -> new route (replan).
        % Time during which the robot is held by the traffic layer (waits for another robot / a zone) does not count.
        for k = 1:obj.N
            r = obj.robots(k); if ~r.rep, continue, end
            if r.state ~= r.stateSeen
                obj.robots(k).stateSeen = r.state; obj.robots(k).stateT = 0; obj.robots(k).posProg = r.pose(1:2); obj.robots(k).posProgT = 0; continue
            end
            if r.task == 0 || r.fault || r.paused || r.stopNow, continue, end
            if r.scale < 0.3 || r.lockHold || r.pendHold || r.hold || r.planReq, continue, end
            st = r.stateT + dt; obj.robots(k).stateT = st;
            lim = inf;
            switch r.state
                case 10, lim = 90;
                case 2,  lim = 150;
                case 3,  lim = 90;
                case 4,  lim = 180;
                case 6,  lim = 180;
            end
            if st > lim
                obj.robots(k).abortSeq = r.abortSeq + 1; obj.robots(k).stateT = 0; obj.robots(k).stallN = r.stallN + 1;
                obj.ev('STALL',k,sprintf('%s: no progress in state %d for %.0f s - task stopped',r.name,r.state,st));
                obj.say('[FLEET] t=%.0f s  %s made no progress in state %d for %.0f s - the fleet stops this task (fault + recovery).\n',obj.t,r.name,r.state,st);
            end
            if any(r.state == [1 5])
                if norm(r.pose(1:2) - r.posProg) > 0.5, obj.robots(k).posProg = r.pose(1:2); obj.robots(k).posProgT = 0;
                else
                    obj.robots(k).posProgT = r.posProgT + dt;
                    if obj.robots(k).posProgT > 90
                        obj.robots(k).posProgT = 0; obj.robots(k).replanSeq = r.replanSeq + 1; obj.replans = obj.replans + 1; obj.robots(k).stallN = r.stallN + 1;
                        obj.ev('STALL',k,sprintf('%s: standing still for 90 s while driving - new route',r.name));
                        obj.say('[FLEET] t=%.0f s  %s has not moved for 90 s while driving - it plans a new route.\n',obj.t,r.name);
                    end
                end
            end
        end
    end
    function d = nearestFreeSlot(obj,xy,k,notSlot)
        d = 0; bestc = inf;
        for i = 1:numel(obj.slots)
            s = obj.slots(i); if i == notSlot || ~strcmp(s.role,'store') || ~strcmp(s.state,'FREE'), continue, end
            if ~obj.lockFree(obj.bayKeys(i),k), continue, end
            c = norm(s.dock - xy) + 4*(s.level-1) + 0.25*obj.faceLoad(s.grp);
            if c < bestc, bestc = c; d = i; end
        end
    end
    function finishTask(obj,tid,k)
        T = obj.tasks(Phase3Fleet.tix(obj.tasks,tid)); j = T.item; r = obj.robots(k);
        if T.dstKind == 1       % shelf slot
            d = T.dstSlot; obj.slots(d).state = 'OCCUPIED'; obj.slots(d).item = j; obj.slots(d).by = 0; obj.slots(d).task = 0;
            obj.cartons(j).state = 'STORED'; obj.cartons(j).slot = d; obj.cartons(j).xyz = obj.slots(d).xyz; obj.cartons(j).cmd = obj.slots(d).xyz;
            obj.cartons(j).drive = false;
            txt = sprintf('%s stored C%d in %s',r.name,j,obj.slots(d).id);
        else
            s = T.dstStation; q = T.dstSlot; st = obj.stations(s);
            st.state{q} = 'OCC'; st.item(q) = j; st.since(q) = obj.t; st.by(q) = 0; obj.stations(s) = st;
            obj.cartons(j).state = 'STATION'; obj.cartons(j).stationSlot = [s q]; obj.cartons(j).slot = 0; obj.cartons(j).drive = false;
            txt = sprintf('%s delivered C%d to %s slot %d',r.name,j,st.name,q);
        end
        obj.cartons(j).task = 0;
        if T.src > 0 && strcmp(obj.slots(T.src).state,'RES_OUT'), obj.slots(T.src).state = 'FREE'; obj.slots(T.src).item = 0; obj.slots(T.src).by = 0; end
        obj.setTask(tid,'status','DONE'); obj.setTask(tid,'done',obj.t);
        if T.order > 0, obj.setOrder(T.order,'status','DONE'); end
        obj.robots(k).task = 0; obj.robots(k).nDone = obj.robots(k).nDone + 1; obj.robots(k).srcFreed = false;
        obj.ev('DONE',k,txt); obj.say('[FLEET] t=%.0f s  %s  (task %d %s, %.0f s)\n',obj.t,txt,tid,T.type,obj.t-T.created);
        obj.say('\n>>> [DONE] %s COMPLETED task %d (%s of C%d): %s.  %s is free - give the next command.\n\n',upper(r.name),tid,T.type,j,txt,r.name);
        if strcmp(obj.mode,'batch') && all(ismember({obj.tasks.status},{'DONE','FAILED','CANCELLED','ABSENT'}))
            obj.say('>>> [BATCH] all queued tasks are finished. Queue the next set, then  p3 go\n\n');
        end
    end

    function absentTask(obj,tid,k)
        T = obj.tasks(Phase3Fleet.tix(obj.tasks,tid)); j = T.item; r = obj.robots(k);
        si = T.src; obj.slots(si).state = 'UNKNOWN'; obj.slots(si).item = 0; obj.slots(si).by = 0; obj.slots(si).task = 0;
        obj.cartons(j).state = 'LOST'; obj.cartons(j).slot = 0; obj.cartons(j).task = 0;
        obj.releaseDst(T);
        obj.setTask(tid,'status','FAILED'); obj.setTask(tid,'note','carton absent at source'); obj.setTask(tid,'done',obj.t);
        if T.order > 0, obj.setOrder(T.order,'status','FAILED'); end
        obj.robots(k).task = 0; obj.robots(k).srcFreed = false;
        obj.ev('ABSENT',k,sprintf('%s: C%d NOT at %s',r.name,j,obj.slots(si).id));
        obj.say('[FLEET] %s: carton C%d is NOT at %s (scan). Slot marked UNKNOWN (clear it with  p3 slot %s free ), task %d failed.\n',r.name,j,obj.slots(si).id,obj.slots(si).id,tid);
    end

    function failTask(obj,tid,k,why)
        T = obj.tasks(Phase3Fleet.tix(obj.tasks,tid)); j = T.item;
        if ~obj.robots(k).srcFreed && T.src > 0, obj.slots(T.src).state = 'OCCUPIED'; obj.slots(T.src).by = 0; obj.slots(T.src).task = 0; end
        if obj.robots(k).srcFreed, obj.cartons(j).state = 'LOST'; end
        obj.cartons(j).task = 0; obj.releaseDst(T);
        obj.setTask(tid,'status','FAILED'); obj.setTask(tid,'note',why); obj.setTask(tid,'done',obj.t);
        if T.order > 0, obj.setOrder(T.order,'status','FAILED'); end
        obj.robots(k).task = 0;
        obj.ev('FAILTASK',k,sprintf('task %d failed: %s',tid,why));
    end

    function releaseDst(obj,T)
        if T.dstKind == 1 && T.dstSlot > 0
            if strcmp(obj.slots(T.dstSlot).state,'RES_IN'), obj.slots(T.dstSlot).state = 'FREE'; obj.slots(T.dstSlot).by = 0; obj.slots(T.dstSlot).task = 0; end
        elseif T.dstKind == 2 && T.dstStation > 0
            st = obj.stations(T.dstStation);
            if strcmp(st.state{T.dstSlot},'RES'), st.state{T.dstSlot} = 'FREE'; st.by(T.dstSlot) = 0; obj.stations(T.dstStation) = st; end
        end
    end

    % ------------------------------------------------------------------ locks
    function ok = lockFree(obj,keys,owner)
        % a key is free when nobody else holds it; shelf-face keys ('fb:') and station-place keys ('st:') follow capacity / distance rules instead of equality
        ok = true;
        for i = 1:numel(keys)
            key = keys{i};
            if strncmp(key,'fb:',3), if ~obj.faceFree(key,owner), ok = false; return, end, continue, end
            if strncmp(key,'st:',3)
                v = sscanf(key(4:end),'%d:%d'); if numel(v) == 2 && ~obj.stFree(v(1),v(2),owner), ok = false; return, end, continue
            end
            for q = 1:size(obj.locks,1)
                if strcmp(obj.locks{q,1},key) && obj.locks{q,2} ~= owner, ok = false; return, end
            end
        end
    end
    function b = keyBlock(obj,key)
        % BLOCK of a shelf-face key: a face is divided into blocks of about 4 slot positions (13 m shelf: 2 blocks per face, 19 m: 3, 30 m: 5); only ONE robot works in a block at a time
        % (different blocks work at the same time).  Returns a number unique per face (shelf, side, block) so two keys of one block compare equal.
        if isempty(obj.blkCache), obj.blkCache = containers.Map('KeyType','char','ValueType','double'); end
        if isKey(obj.blkCache,key), b = obj.blkCache(key); return, end
        b = -1; tok = regexp(key,'^fb:(\d+)([AB]):(-?\d+)$','tokens','once');
        if ~isempty(tok)
            sh = str2double(tok{1}); sd = tok{2}; y = str2double(tok{3});
            ix = find([obj.slots.shelf] == sh & strcmp({obj.slots.side},sd) & strcmp({obj.slots.role},'store'));
            if ~isempty(ix)
                np = max([obj.slots(ix).pos]); nB = max(1,ceil(np/4)); [~,q] = min(abs(arrayfun(@(i) round(100*obj.slots(i).xyz(2)),ix) - y));
                b = sh*1000 + (sd == 'B')*100 + min(nB,max(1,ceil(obj.slots(ix(q)).pos*nB/np)));
            end
        end
        obj.blkCache(key) = b;
    end
    function ok = faceFree(obj,key,owner)
        % one face of a shelf (A or B) takes at most faceCap(shelf) robots at the same time (about one per 4.3 m of shelf), and two of them must be at least faceSep apart along the face
        tok = regexp(key,'^fb:(\d+)([AB]):(-?\d+)$','tokens','once'); ok = true; if isempty(tok), return, end
        sh = str2double(tok{1}); y = str2double(tok{3})/100; pre = sprintf('fb:%s%s:',tok{1},tok{2}); own = zeros(1,0);
        for i = 1:size(obj.locks,1)
            k2 = obj.locks{i,1}; if obj.locks{i,2} == owner || ~strncmp(k2,pre,numel(pre)), continue, end
            if abs(str2double(k2(numel(pre)+1:end))/100 - y) < obj.faceSep - 1e-3 || (obj.keyBlock(key) > 0 && obj.keyBlock(k2) == obj.keyBlock(key)), ok = false; return, end     % one robot per BLOCK of the face (and never closer than faceSep)
            own(end+1) = obj.locks{i,2}; %#ok<AGROW>
        end
        if numel(unique(own)) >= obj.faceCap(sh), ok = false; end
    end
    function n = faceCap(obj,sh)
        n = max(1,min(obj.N,floor(obj.C.WH.shelves(sh,3)/obj.faceCapPitch + 1e-6)));
    end
    function lockTake(obj,keys,owner)
        for i = 1:numel(keys)
            have = false;
            for q = 1:size(obj.locks,1), if strcmp(obj.locks{q,1},keys{i}) && obj.locks{q,2} == owner, have = true; end, end
            if ~have, obj.locks(end+1,:) = {keys{i},owner}; end
        end
    end
    function lockDrop(obj,owner,keys)
        keep = true(size(obj.locks,1),1);
        for q = 1:size(obj.locks,1)
            if obj.locks{q,2} == owner && (nargin < 3 || any(strcmp(keys,obj.locks{q,1}))), keep(q) = false; end
        end
        obj.locks = obj.locks(keep,:);
    end
    function keys = stKeys(obj,st,q)
        % a packing-station place is locked by its own key only; whether another place may be used at the same time is decided by stFree (distance)
        keys = {sprintf('st:%d:%d',st,q)};
    end
    function ok = stFree(obj,st,q,owner)
        % place q of station st may be used when no OTHER robot holds a place closer than stSep (robots at different places of one table work at the same time when far enough apart)
        ok = true; A = obj.C.Stations(st).approach; pre = sprintf('st:%d:',st);
        for i = 1:size(obj.locks,1)
            key = obj.locks{i,1}; if obj.locks{i,2} == owner || ~strncmp(key,pre,numel(pre)), continue, end
            q2 = sscanf(key(numel(pre)+1:end),'%d');
            if q2 == q || norm(A(q2,:) - A(q,:)) < obj.stSep - 1e-3, ok = false; return, end
        end
    end
    function keys = bayKeys(obj,si)
        % shelf slot: ONE key of its face (shelf + side + position along the face) - faceFree decides capacity and spacing.  Staging stand: its own key and the two neighbours.
        s = obj.slots(si);
        if s.shelf == 0, keys = cell(1,3); for d = -1:1, keys{d+2} = sprintf('bay:%d:%d',s.grp,s.pos+d); end, return, end
        keys = {sprintf('fb:%d%s:%d',s.shelf,s.side,round(100*s.xyz(2)))};
    end
    function locksRelease(obj)
        for k = 1:obj.N
            r = obj.robots(k); if ~r.rep, continue, end
            if r.task > 0 && any(r.state == [5 6 7 8 9]) && ~isempty(r.srcKeys) && r.jobSeq >= r.jobSeqSent && norm(r.pose(1:2) - r.srcPt) > obj.TP.leaveDist
                obj.lockDrop(k,r.srcKeys); obj.robots(k).srcKeys = {};
            end
            if r.task > 0 && r.state == 1 && r.jobSeq >= r.jobSeqSent && ~isempty(obj.locks) && norm(r.pose(1:2) - r.prevPt) > obj.TP.leaveDist      % locks of the PREVIOUS job (a robot that gets its next task at once keeps them otherwise): released when it has left
                mine = find(cellfun(@(o) o == k,obj.locks(:,2))); stale = false(size(obj.locks,1),1);
                for q = mine(:).', if ~any(strcmp(obj.locks{q,1},[r.srcKeys r.dstKeys])), stale(q) = true; end, end
                if any(stale), obj.locks = obj.locks(~stale,:); end
            end
            if r.task == 0 && any(r.state == [0 7 12]) && any(cellfun(@(o) o == k,obj.locks(:,2))) && norm(r.pose(1:2) - r.dstPt) > obj.TP.leaveDist     % the bay stays reserved until the robot has really LEFT it
                obj.lockDrop(k); obj.robots(k).dstLockOk = false; obj.robots(k).srcLockOk = false; obj.robots(k).dstKeys = {}; obj.robots(k).srcKeys = {};
            end
        end
    end
    function [ok,drop] = retargetPlace(obj,k,claimed,earlier)
        % the robot has not picked yet: if the station place chosen at assignment cannot work now (another robot works next to it), take another place of the SAME table that can:
        % a FREE one (lowest number first), else swap with a robot that has not started either (its place is only promised, nobody works there).  The robots are told the new
        % place (cmd.dstUpd); nothing is carried yet, so nothing else changes.
        ok = false; drop = {}; r = obj.robots(k); ti = Phase3Fleet.tix(obj.tasks,r.task); if isempty(ti), return, end
        T = obj.tasks(ti); if T.dstKind ~= 2 || T.dstStation < 1, return, end
        st = T.dstStation; S = obj.stations(st); pick = 0; other = 0;
        for qq = find(strcmp(S.state,'FREE'))
            if qq == T.dstSlot, continue, end
            if obj.stFree(st,qq,k) && ~obj.claimHit(obj.stKeys(st,qq),claimed), pick = qq; break, end
        end
        if pick == 0
            for qq = find(strcmp(S.state,'RES'))
                m = S.by(qq); if qq == T.dstSlot || m == k || m < 1 || ismember(m,earlier) || obj.claimHit(obj.stKeys(st,qq),claimed) || ~obj.robots(m).rep || obj.robots(m).task < 1 || obj.robots(m).srcLockOk || obj.robots(m).dstLockOk, continue, end
                if obj.stFree(st,qq,k), pick = qq; other = m; break, end
            end
        end
        if pick == 0, return, end
        old = T.dstSlot;
        if other == 0
            S.state{old} = 'FREE'; S.by(old) = 0; S.state{pick} = 'RES'; S.by(pick) = k;
        else
            S.by(old) = other; S.by(pick) = k;
            tim = Phase3Fleet.tix(obj.tasks,obj.robots(other).task); obj.tasks(tim).dstSlot = old;
            jo = obj.makeJob(obj.robots(other).jobSeqSent,obj.tasks(tim)); drop = obj.robots(other).dstKeys;
            obj.robots(other).job = jo; obj.robots(other).dstKeys = obj.stKeys(st,old); obj.robots(other).dstPt = jo.dst.approach(1:2);
            obj.robots(other).dstUpd = jo.dst; obj.robots(other).dstUpdSeq = obj.robots(other).dstUpdSeq + 1;
        end
        obj.stations(st) = S; obj.tasks(ti).dstSlot = pick;
        job = obj.makeJob(r.jobSeqSent,obj.tasks(ti)); obj.robots(k).job = job; obj.robots(k).dstKeys = obj.stKeys(st,pick); obj.robots(k).dstPt = job.dst.approach(1:2);
        obj.robots(k).dstUpd = job.dst; obj.robots(k).dstUpdSeq = r.dstUpdSeq + 1;
        obj.ev('RETARGET',k,sprintf('%s: %s slot %d is crowded, takes slot %d (can work at the same time as the others)',r.name,S.name,old,pick));
        ok = true;
    end
    function ok = dstBusyOnlyCarriers(obj,kd,k)
        % true when every robot that blocks the station place keys kd is past its pick (carrying / delivering) and holds both of its keys - it will leave without waiting for anybody
        ok = true; nb = 0;
        for i = 1:numel(kd)
            v = sscanf(kd{i}(4:end),'%d:%d'); if ~strncmp(kd{i},'st:',3) || numel(v) ~= 2, ok = false; return, end
            A = obj.C.Stations(v(1)).approach; pre = sprintf('st:%d:',v(1));
            for j = 1:size(obj.locks,1)
                key = obj.locks{j,1}; m = obj.locks{j,2}; if m == k || ~strncmp(key,pre,numel(pre)), continue, end
                q2 = sscanf(key(numel(pre)+1:end),'%d');
                if q2 == v(2) || norm(A(q2,:) - A(v(2),:)) < obj.stSep - 1e-3
                    nb = nb + 1; rm = obj.robots(m);
                    if ~(rm.dstLockOk || rm.task == 0), ok = false; return, end      % it holds BOTH its keys (or has finished): it waits for nobody and leaves on its own
                end
            end
        end
        ok = ok && nb > 0;
    end
    function hit = claimHit(obj,keys,claimed)
        % first-come order WITHOUT blocking the whole face / table: a later robot is held back only by an earlier waiting robot whose wanted place it would take or crowd (same face and
        % closer than faceSep, same station place or closer than stSep, same key) - a free place elsewhere on the face goes to whoever comes next (more robots work in parallel)
        hit = false;
        for i = 1:numel(keys)
            ki = keys{i};
            for j = 1:numel(claimed)
                kj = claimed{j};
                if strcmp(ki,kj), hit = true; return, end
                if strncmp(ki,'fb:',3) && strncmp(kj,'fb:',3)
                    ci = find(ki == ':'); cj = find(kj == ':');
                    if numel(ci) >= 2 && numel(cj) >= 2 && strcmp(ki(1:ci(2)),kj(1:cj(2))) && (abs(str2double(ki(ci(2)+1:end)) - str2double(kj(cj(2)+1:end)))/100 < obj.faceSep - 1e-3 || (obj.keyBlock(ki) > 0 && obj.keyBlock(ki) == obj.keyBlock(kj))), hit = true; return, end
                elseif strncmp(ki,'st:',3) && strncmp(kj,'st:',3)
                    vi = sscanf(ki(4:end),'%d:%d'); vj = sscanf(kj(4:end),'%d:%d');
                    if numel(vi) == 2 && numel(vj) == 2 && vi(1) == vj(1) && norm(obj.C.Stations(vi(1)).approach(vi(2),:) - obj.C.Stations(vj(1)).approach(vj(2),:)) < obj.stSep - 1e-3, hit = true; return, end
                end
            end
        end
    end
    function locksRequest(obj)
        % Zone keys are taken in FIRST-COME order and ALL-OR-NOTHING per step: a robot that cannot get its source face yet holds NOTHING and waits (empty-handed) at a free point
        % at a safe distance from the shelf (pendHold); a carrying robot waits for its destination at dstStop from it.  Nobody waits while holding a key -> no lock deadlock.
        for k = 1:obj.N
            if obj.robots(k).task == 0, obj.robots(k).pendHold = false; obj.robots(k).holdPt = []; obj.robots(k).holdTried = false; end
        end
        cand = []; pr = [];
        for k = 1:obj.N
            r = obj.robots(k);
            if r.task > 0 && r.rep && r.jobSeq >= r.jobSeqSent && ~r.fault && ~(r.srcLockOk && r.dstLockOk) && ~(isempty(r.srcKeys) && isempty(r.dstKeys)), cand(end+1) = k; pr(end+1) = obj.tasks(Phase3Fleet.tix(obj.tasks,r.task)).prio*1e4 - obj.tasks(Phase3Fleet.tix(obj.tasks,r.task)).assigned; end %#ok<AGROW>
        end
        [~,ix] = sort(pr,'descend'); cand = cand(ix); claimed = {};
        for k = cand
            r = obj.robots(k);
            if ~r.srcLockOk && ~r.dstLockOk
                % a task STARTS only when its shelf face AND its station place / destination are both free (all-or-nothing): a robot that carries a box never has to stand and
                % wait for a place (no carrier queue in front of a station, no circular waiting between robots that hold one key and wait for the other)
                ks = r.srcKeys; kd = r.dstKeys;
                if (obj.claimHit(kd,claimed) || ~obj.lockFree(kd,k)) && ~obj.claimHit(ks,claimed) && obj.lockFree(ks,k)
                    [okR,dropK] = obj.retargetPlace(k,claimed,cand(1:find(cand==k)-1));
                    if okR, r = obj.robots(k); kd = r.dstKeys; if ~isempty(dropK), claimed = claimed(~ismember(claimed,dropK)); end, end
                end      % another FREE place of the same table that can work now
                if ~obj.claimHit(ks,claimed) && obj.lockFree(ks,k) && ~obj.claimHit(kd,claimed) && ~obj.lockFree(kd,k) && obj.dstBusyOnlyCarriers(kd,k)
                    % the station place is only busy because robots that already CARRY their box are delivering there (they leave on their own): go to the shelf now, the place will be
                    % free when the box arrives (a carrier that still finds it busy waits at a free point, see pendD) - the shelf work of earlier robots is not delayed by the station
                    obj.lockTake(ks,k); obj.robots(k).srcLockOk = true; obj.robots(k).lockWait = 0; claimed = [claimed kd]; %#ok<AGROW>
                    obj.robots(k).pendHold = false; obj.robots(k).holdPt = []; obj.robots(k).holdTried = false;
                    continue
                end
                if obj.claimHit([ks kd],claimed) || ~obj.lockFree(ks,k) || ~obj.lockFree(kd,k), if obj.claimHit(ks,claimed) || ~obj.lockFree(ks,k), claimed = [claimed ks]; else, claimed = [claimed ks kd]; end, obj.robots(k).lockWait = r.lockWait + 0.1; continue, end %#ok<AGROW>
                obj.lockTake(ks,k); obj.lockTake(kd,k); obj.robots(k).srcLockOk = true; obj.robots(k).dstLockOk = true; obj.robots(k).lockWait = 0;
                obj.robots(k).pendHold = false; obj.robots(k).holdPt = []; obj.robots(k).holdTried = false;
                continue
            end
            if ~r.srcLockOk
                ks = r.srcKeys;
                if obj.claimHit(ks,claimed) || ~obj.lockFree(ks,k), claimed = [claimed ks]; continue, end %#ok<AGROW>
                obj.lockTake(ks,k); obj.robots(k).srcLockOk = true; obj.robots(k).pendHold = false; obj.robots(k).holdPt = []; obj.robots(k).holdTried = false;
                r = obj.robots(k);
            end
            if ~r.dstLockOk
                kd = r.dstKeys;
                if obj.claimHit(kd,claimed) || ~obj.lockFree(kd,k), claimed = [claimed kd]; obj.robots(k).lockWait = r.lockWait + 0.1; continue, end %#ok<AGROW>
                obj.lockTake(kd,k); obj.robots(k).dstLockOk = true; obj.robots(k).lockWait = 0;
            end
        end
        % ---- waiting robots: where do they stand?
        for k = 1:obj.N
            r = obj.robots(k);
            pendD = r.task > 0 && r.rep && r.state == 5 && r.srcLockOk && ~r.dstLockOk && ~r.fault;
            if pendD
                % a CARRYING robot whose station place is not free yet waits at a free point (planned route, off every lane), not in the lane of the robot it waits for
                dD = norm(r.pose(1:2) - r.dstPt); atH = ~isempty(r.holdPt) && norm(r.pose(1:2) - r.holdPt) < 0.7;
                if ~r.holdTried && dD > obj.dstStop + 1.0
                    obj.robots(k).holdTried = true; pt = obj.waitPoint(k,r.dstPt);
                    if ~isempty(pt)
                        obj.robots(k).holdPt = pt; obj.robots(k).evade = [pt, r.evade(3) + 1]; obj.robots(k).evIssued = obj.t;
                        obj.ev('WAIT',k,sprintf('%s (carrying) waits for its station place at (%.1f, %.1f), %.1f m from it',r.name,pt,norm(pt - r.dstPt)));
                    end
                elseif ~atH && ~isempty(r.holdPt) && obj.t - r.evIssued > 14
                    obj.robots(k).evade = [r.holdPt, r.evade(3) + 1]; obj.robots(k).evIssued = obj.t;
                end
                obj.robots(k).pendHold = false; continue
            end
            pend = r.task > 0 && r.rep && r.jobSeq >= r.jobSeqSent && ~r.srcLockOk && ~r.fault && r.state == 1;
            if ~pend, obj.robots(k).pendHold = false; if r.task == 0 || r.srcLockOk, obj.robots(k).holdPt = []; obj.robots(k).holdTried = false; end, continue, end
            d = norm(r.pose(1:2) - r.srcPt); ph = r.pendHold;
            if ~isempty(r.holdPt) && norm(r.pose(1:2) - r.holdPt) < 0.7, ph = true; end
            if d < obj.pendStop, ph = true; end                                  % safety net: never drive into the shelf lane while the face is not yours
            if ~ph && ~r.holdTried && d > obj.pendStop + 1.0
                obj.robots(k).holdTried = true; pt = obj.waitPoint(k);
                if ~isempty(pt)
                    obj.robots(k).holdPt = pt; obj.robots(k).evade = [pt, r.evade(3) + 1]; obj.robots(k).evIssued = obj.t;
                    obj.ev('WAIT',k,sprintf('%s waits for %s at (%.1f, %.1f), %.1f m from it - its place (shelf face or station) is not free',r.name,r.job.src.name,pt,norm(pt - r.srcPt)));
                end
            elseif ~ph && ~isempty(r.holdPt) && obj.t - r.evIssued > 14
                obj.robots(k).evade = [r.holdPt, r.evade(3) + 1]; obj.robots(k).evIssued = obj.t;
            end
            obj.robots(k).pendHold = ph;
        end
        for k = 1:obj.N
            r = obj.robots(k);
            obj.robots(k).lockHold = r.task > 0 && r.state == 5 && ~r.dstLockOk && (norm(r.pose(1:2) - r.dstPt) < obj.dstStop || (~isempty(r.holdPt) && norm(r.pose(1:2) - r.holdPt) < 0.7));
        end
    end
    function pt = waitPoint(obj,k,S)
        % free floor point for a robot that has to wait for a shelf face: 6..11 m from the staging point of its slot, outside every shelf lane / bay / station approach / stand lane,
        % clear of the other robots and of their waiting points, as near to the robot as possible
        r = obj.robots(k); if nargin < 3, S = r.srcPt; end, pt = []; best = inf;
        if isempty(obj.strips), obj.strips = obj.workZones(); end
        Z = obj.strips;
        occ = zeros(0,2); for m = 1:obj.N, if m ~= k && obj.robots(m).rep, occ(end+1,:) = obj.robots(m).pose(1:2); if ~isempty(obj.robots(m).holdPt), occ(end+1,:) = obj.robots(m).holdPt; end, end, end %#ok<AGROW>
        for rad = [6 7.5 9 11]
            for ang = (0:20:340)*pi/180
                c = S + rad*[cos(ang) sin(ang)];
                if any(c(1) > Z(:,1) & c(1) < Z(:,2) & c(2) > Z(:,3) & c(2) < Z(:,4)), continue, end
                if ~obj.isFree(c), continue, end
                nearWall = false; for a2 = (0:45:315)*pi/180, if ~obj.isFree(c + 1.0*[cos(a2) sin(a2)]), nearWall = true; break, end, end
                if nearWall, continue, end            % a waiting robot stands >= 1 m clear of every shelf / wall
                if ~isempty(occ) && min(sqrt(sum((occ - c).^2,2))) < 3.0, continue, end
                cost = norm(c - r.pose(1:2)) + 0.4*abs(rad - 7);
                if cost < best, best = cost; pt = c; end
            end
        end
    end
    function Z = workZones(obj)
        % rectangles [x0 x1 y0 y1] where robots work or pass in a line: the lane in front of every shelf face, the bays, the stand lane, the station approaches
        Z = zeros(0,4); sl = obj.slots; seen = {};
        for i = 1:numel(sl)
            s = sl(i); if ~strcmp(s.role,'store'), continue, end
            g = sprintf('%d%s',s.shelf,s.side); if any(strcmp(seen,g)), continue, end
            seen{end+1} = g; %#ok<AGROW>
            ix = find([sl.shelf] == s.shelf & [sl.side] == s.side & strcmp({sl.role},'store')); ys = arrayfun(@(q) sl(q).xyz(2),ix);
            Z(end+1,:) = [s.staging(1) - 1.6, s.staging(1) + 1.6, min(ys) - 3.0, max(ys) + 3.0]; %#ok<AGROW>
        end
        for i = 1:numel(sl), if strcmp(sl(i).role,'dock'), q = sl(i).staging(1:2); Z(end+1,:) = [q(1) - 1.6, q(1) + 1.6, q(2) - 2.0, q(2) + 2.0]; end, end %#ok<AGROW>
        for m = 1:obj.N, h = obj.robots(m).home; Z(end+1,:) = [h(1) - 2.2, h(1) + 2.2, h(2) - 2.2, h(2) + 2.2]; end %#ok<AGROW>
        for st = 1:numel(obj.C.Stations), A = obj.C.Stations(st).approach; Z(end+1,:) = [min(A(:,1)) - 2.5, max(A(:,1)) + 2.5, min(A(:,2)) - 3.0, max(A(:,2)) + 3.0]; end %#ok<AGROW>
    end
    % ------------------------------------------------------------------ housekeeping, auto generator
    function housekeeping(obj)
        for j = 1:obj.M
            if obj.cartons(j).spawnUntil > 0 && obj.t > obj.cartons(j).spawnUntil && ~strcmp(obj.cartons(j).state,'DOCK')
                obj.cartons(j).drive = false; obj.cartons(j).spawnUntil = 0;
            end
        end
        for j = 1:obj.M       % an idle stored carton is held exactly in its slot (cures any drift); a carton with a task is left to the physics / the arm
            if strcmp(obj.cartons(j).state,'STORED') && obj.cartons(j).task == 0, obj.cartons(j).drive = true; obj.cartons(j).cmd = obj.cartons(j).xyz; obj.cartons(j).cmdYaw = obj.slots(obj.cartons(j).slot).boxYaw; end
            if strcmp(obj.cartons(j).state,'STORED') && obj.cartons(j).task ~= 0, obj.cartons(j).drive = false; end
        end
        for s = 1:numel(obj.stations)
            st = obj.stations(s); chg = false;
            nq = numel(st.state);
            full = all(strcmp(st.state,'OCC')) && obj.t - max(st.since) > obj.dwell && ~obj.keepStations;      % automatic mode: the boxes stay on the tables (that is the goal), they are not shipped
            if full || (obj.shipReq(s) && any(strcmp(st.state,'OCC')))      % a station is emptied only when every place of it is filled (or on  p3 ship)
                obj.shipReq(s) = false;
                for q = 1:nq
                    if ~strcmp(st.state{q},'OCC'), continue, end
                    j = st.item(q); obj.cartons(j).state = 'RESERVE'; obj.cartons(j).stationSlot = [0 0];
                    obj.cartons(j).drive = true; obj.cartons(j).cmd = obj.cartons(j).park; obj.cartons(j).cmdYaw = 0; obj.cartons(j).spawnUntil = 0;
                    st.state{q} = 'FREE'; st.item(q) = 0; chg = true; obj.shipped = obj.shipped + 1;
                    obj.ev('SHIP',0,sprintf('C%d shipped from %s slot %d',j,st.name,q));
                end
                obj.say('[FLEET] t=%.0f s  %s is full - all its boxes are shipped out together\n',obj.t,st.name);
            end
            if chg, obj.stations(s) = st; end
        end
    end
    function n = autoCount(obj)
        % tasks the automatic mode has created since it was switched on (running + open orders)
        open = 0; for q = 1:numel(obj.orders), if strcmp(obj.orders(q).status,'WAIT'), open = open + 1; end, end
        n = obj.nRetAuto + obj.nPutAuto - obj.autoBase + open;
    end
    function autoGen(obj)
        if ~obj.autoOn || obj.finishing || obj.autoCount() >= obj.autoPerRobot*obj.N, return, end       % AUTO = a fixed set of critical tasks (3 per robot), not an endless run
        % new delivery: only when EVERY staging stand is empty, then all stands are filled again D01, D02 ...
        dk = strcmp({obj.slots.role},'dock'); nres = sum(strcmp({obj.cartons.state},'RESERVE'));
        needRet = obj.autoNeed();       % boxes that still have to be ordered to the packing stations (capacity - boxes there - open orders)
        if all(strcmp({obj.slots(dk).state},'FREE')) && nres > 0 && needRet > 0       % no more fresh boxes once every station is covered
            if isinf(obj.waveAt), obj.waveAt = obj.t + 8; end
            if obj.t >= obj.waveAt
                obj.waveAt = inf; n = min(sum(dk),nres);
                obj.say('[FLEET] t=%.0f s  NEW DELIVERY: %d fresh boxes arrive on the staging stands\n',obj.t,n); obj.dockAdd(n,true);
            end
        else, obj.waveAt = inf; end
        nDock = sum(strcmp({obj.cartons.state},'DOCK'));
        nQ = sum(strcmp({obj.tasks.status},'QUEUED')); nAv = 0; for q = 1:obj.N, nAv = nAv + double(obj.robotAvailable(q)); end
        hungry = nAv > nQ;           % a robot is free and no task is waiting for it: create the next order at once (robots start one after the other, not every 20 s)
        if (obj.t >= obj.autoNext || hungry) && (obj.nRetAuto <= obj.nPutAuto || (nres == 0 && nDock == 0)) && needRet > 0        % retrieve orders and putaways are kept 50 : 50
            obj.autoNext = obj.t + obj.autoPeriod;
            cand = [];
            for j = 1:obj.M
                if strcmp(obj.cartons(j).state,'STORED') && obj.cartons(j).task == 0 && ~obj.hasOrder(j), cand(end+1) = j; end %#ok<AGROW>
            end
            if isempty(cand)
                for j = 1:obj.M
                    if strcmp(obj.cartons(j).state,'DOCK') && obj.cartons(j).task == 0 && ~obj.hasOrder(j), cand(end+1) = j; end %#ok<AGROW>
                end
            end
            if ~isempty(cand)
                obj.lcg = mod(obj.lcg*1103515245 + 12345,2^31); j = cand(1 + mod(floor(obj.lcg/65536),numel(cand)));
                obj.addOrder(j,obj.autoStation,true);
            end
        end
    end
    function n = autoNeed(obj)
        cap = 0; occ = 0; for q = 1:numel(obj.stations), cap = cap + numel(obj.stations(q).state); occ = occ + sum(~strcmp(obj.stations(q).state,'FREE')); end
        open = 0; for q = 1:numel(obj.orders), if any(strcmp(obj.orders(q).status,{'WAIT','TASK'})), open = open + 1; end, end
        n = cap - occ - open;
    end
    function autoEnd(obj)
        % AUTO mode is finished when its fixed set of tasks (3 per robot) is done and every robot is back home.  The simulation (same 3-D window) goes on: choose the next mode.
        if ~obj.autoOn || obj.finishing || obj.t < 30, return, end
        if obj.autoCount() < obj.autoPerRobot*obj.N, return, end
        for q = 1:numel(obj.tasks), if any(strcmp(obj.tasks(q).status,{'QUEUED','ACTIVE','RUNNING','HELD'})), return, end, end
        for q = 1:numel(obj.orders), if any(strcmp(obj.orders(q).status,{'WAIT','TASK'})), return, end, end
        for k = 1:obj.N
            r = obj.robots(k); if ~r.rep || r.fault, continue, end
            if r.task > 0 || ~any(r.state == [0 7]) || norm(r.pose(1:2) - r.home(1:2)) > 1.0, return, end
        end
        obj.autoOn = false; obj.autoPutaway = false; obj.autoNext = inf; obj.autoArriveNext = inf; obj.mode = 'standby'; obj.autoFinished = true;
        done = sum(strcmp({obj.tasks.status},'DONE'));
        obj.ev('FLEET',0,sprintf('AUTO finished: %d tasks done, all robots home',done)); obj.metricsWrite(false);
        obj.say(['\n[P3] ===== AUTO FINISHED: %d tasks done, all robots are home. The simulation keeps running (same window). =====\n' ...
            '     next:  p3 mode manual | p3 mode batch | p3 mode auto      (results so far are saved)\n'],done);
    end
    function tf = hasOrder(obj,j)
        tf = false;
        for i = 1:numel(obj.orders), if obj.orders(i).item == j && any(strcmp(obj.orders(i).status,{'WAIT','TASK'})), tf = true; end, end
    end

    % ------------------------------------------------------------------ tasks
    function makeTasks(obj)
        % open orders -> tasks
        for oi = 1:numel(obj.orders)
            o = obj.orders(oi); if ~strcmp(o.status,'WAIT'), continue, end
            j = o.item;
            if j == 0       % "any": oldest stored carton without a task
                for q = 1:obj.M
                    if strcmp(obj.cartons(q).state,'STORED') && obj.cartons(q).task == 0 && ~obj.hasOrder(q), j = q; break, end
                end
                if j == 0, continue, end
                obj.orders(oi).item = j;
            end
            c = obj.cartons(j);
            if c.task ~= 0, continue, end
            if strcmp(c.state,'STORED')
                obj.nRetAuto = obj.nRetAuto + 1;
                obj.newTask('retrieve',j,c.slot,2,o.station,0,2,oi,0);
                obj.setOrder(o.id,'status','TASK');
            elseif strcmp(c.state,'DOCK')
                obj.nRetAuto = obj.nRetAuto + 1;
                obj.newTask('crossdock',j,c.slot,2,o.station,0,3,oi,0);
                obj.setOrder(o.id,'status','TASK');
            end
        end
        % cartons waiting at the dock with no order -> putaway
        for j = 1:obj.M
            c = obj.cartons(j);
            if obj.autoPutaway && strcmp(c.state,'DOCK') && c.task == 0 && ~obj.hasOrder(j) && (~obj.autoOn || (obj.autoCount() < obj.autoPerRobot*obj.N && (obj.nPutAuto <= obj.nRetAuto + 1 || obj.autoNeed() <= 0)))
                obj.nPutAuto = obj.nPutAuto + 1; obj.newTask('putaway',j,c.slot,1,0,0,1,0,0);
            end
        end
    end
    function tid = newTask(obj,type,j,src,dstKind,station,dstSlot,prio,order,pinned)
        tid = obj.nextTask; obj.nextTask = tid + 1;
        T = Phase3Fleet.emptyTasks(); T = Phase3Fleet.blankTask(tid,type,j,src,dstKind,station,dstSlot,prio,order,pinned,obj.t);
        obj.tasks(Phase3Fleet.nextIx(obj.tasks)) = T;
        obj.cartons(j).task = tid;
        if order > 0, obj.setOrder(obj.orders(order).id,'task',tid); end
        obj.ev('TASK',0,sprintf('task %d %s C%d queued',tid,type,j));
    end
    function setTask(obj,tid,f,v), obj.tasks(Phase3Fleet.tix(obj.tasks,tid)).(f) = v; end
    function setOrder(obj,oid,f,v)
        for i = 1:numel(obj.orders), if obj.orders(i).id == oid, obj.orders(i).(f) = v; return, end, end
    end

    function ok = robotAvailable(obj,k)
        r = obj.robots(k);
        ok = r.rep && ~r.paused && ~r.fault && ~r.finish && r.task == 0 && any(r.state == [0 7]) && r.jobSeq >= r.jobSeqSent && ~r.stopNow;
    end

    function assign(obj)
        for k = 1:obj.N
            if ~obj.robotAvailable(k) || obj.t < obj.nextAssign, continue, end      % robots are sent out one after the other (no pile-up at the bays)
            atBay = norm(obj.robots(k).pose(1:2) - obj.robots(k).home) < 2.5;
            if atBay && obj.t < obj.nextBayStart, continue, end                    % a robot that leaves its bay starts bayGap seconds after the previous one
            best = 0; bestScore = inf; bestInfo = [];
            for ti = 1:numel(obj.tasks)
                T = obj.tasks(ti);
                if ~strcmp(T.status,'QUEUED'), continue, end
                if T.pinned > 0 && T.pinned ~= k, continue, end
                if T.pinned == 0 && ~isempty(obj.excl) && any(obj.excl(:,1) == T.id & obj.excl(:,2) == k), continue, end
                info = obj.tryPlan(T,k); if isempty(info), continue, end
                score = info.cost - 15*T.prio - 0.3*(obj.t - T.created);
                if score < bestScore, bestScore = score; best = ti; bestInfo = info; end
            end
            if best > 0, obj.commit(best,k,bestInfo); obj.nextAssign = obj.t + obj.assignGap; if atBay, obj.nextBayStart = obj.t + obj.bayGap; end, end
        end
    end

    function info = tryPlan(obj,T,k)
        info = []; r = obj.robots(k);
        s = obj.slots(T.src);
        if ~(strcmp(s.state,'OCCUPIED') && s.item == T.item), return, end
        sk = obj.bayKeys(T.src); pen = 0; if ~obj.lockFree(sk,k), pen = pen + 400; end      % zone busy now: the robot may still take the task (it waits at a safe distance), but a task whose zones are free is preferred
        if T.dstKind == 1
            d = T.dstSlot;
            if d == 0, d = obj.bestFreeSlot(T.src,k); end
            if d == 0 || ~strcmp(obj.slots(d).state,'FREE'), return, end
            dk = obj.bayKeys(d); if ~obj.lockFree(dk,k), pen = pen + 200; end
            appr = obj.slots(d).dock; info = struct('dstKind',1,'dstSlot',d,'dstStation',0,'srcKeys',{sk},'dstKeys',{dk});
        else
            sset = T.dstStation; if sset == 0, sset = 1:numel(obj.stations); end
            bestc = inf; bs = 0; bq = 0;
            for st = sset
                for q = find(strcmp(obj.stations(st).state,'FREE'))
                    c = 0.2*norm(obj.C.Stations(st).approach(q,:) - r.pose(1:2)) + 3*obj.stationLoad(st) + 2*q;
                    if ~obj.stFree(st,q,k), c = c + 40; end        % somebody works at (or next to) this place right now: another place is preferred
                    c = c + 25*obj.stNeighbours(st,q,k);           % places already promised to other robots within the working distance: choose a place that can work at the same time
                    if c < bestc, bestc = c; bs = st; bq = q; end
                end
            end
            if bs == 0, return, end
            appr = obj.C.Stations(bs).approach(bq,:);
            info = struct('dstKind',2,'dstSlot',bq,'dstStation',bs,'srcKeys',{sk},'dstKeys',{obj.stKeys(bs,bq)});
        end
        info.cost = norm(s.staging - r.pose(1:2)) + norm(s.dock - appr) + pen;
    end
    function n = stNeighbours(obj,st,q,k)
        A = obj.C.Stations(st).approach; n = 0; S = obj.stations(st);
        for q2 = 1:min(size(A,1),numel(S.state)), if q2 ~= q && strcmp(S.state{q2},'RES') && S.by(q2) ~= k && norm(A(q2,:) - A(q,:)) < obj.stSep - 1e-3, n = n + 1; end, end
    end
    function n = stationLoad(obj,st)
        n = sum(~strcmp(obj.stations(st).state,'FREE'));
    end
    function d = bestFreeSlot(obj,srcIdx,k)
        d = 0; bestc = inf; sd = obj.slots(srcIdx).dock;
        for i = 1:numel(obj.slots)
            s = obj.slots(i); if ~strcmp(s.role,'store') || ~strcmp(s.state,'FREE'), continue, end
            c = norm(s.dock - sd) + 4*(s.level-1) + 0.25*obj.faceLoad(s.grp);
            if ~obj.lockFree(obj.bayKeys(i),k), c = c + 60; end        % busy face: only if nothing else is free
            if c < bestc, bestc = c; d = i; end
        end
    end
    function n = faceLoad(obj,grp)
        n = 0; for i = 1:numel(obj.slots), if obj.slots(i).grp == grp && any(strcmp(obj.slots(i).state,{'OCCUPIED','RES_IN','RES_OUT'})), n = n + 1; end, end
    end

    function commit(obj,ti,k,info)
        T = obj.tasks(ti); r = obj.robots(k); j = T.item;
        obj.slots(T.src).state = 'RES_OUT'; obj.slots(T.src).by = k; obj.slots(T.src).task = T.id;
        if info.dstKind == 1
            obj.slots(info.dstSlot).state = 'RES_IN'; obj.slots(info.dstSlot).by = k; obj.slots(info.dstSlot).task = T.id;
            dname = obj.slots(info.dstSlot).id;
        else
            st = obj.stations(info.dstStation); st.state{info.dstSlot} = 'RES'; st.by(info.dstSlot) = k; obj.stations(info.dstStation) = st;
            dname = sprintf('%s slot %d',st.name,info.dstSlot);
        end
        obj.tasks(ti).dstKind = info.dstKind; obj.tasks(ti).dstSlot = info.dstSlot; obj.tasks(ti).dstStation = info.dstStation;
        obj.tasks(ti).status = 'ACTIVE'; obj.tasks(ti).robot = k; obj.tasks(ti).assigned = obj.t;
        % the zone keys are NOT taken here: locksRequest takes them first-come, all-or-nothing, as soon as the controller has the job; a robot whose face is full waits at a safe distance (pendHold)
        obj.robots(k).task = T.id; obj.robots(k).srcKeys = info.srcKeys; obj.robots(k).dstKeys = info.dstKeys; obj.robots(k).dstLockOk = false; obj.robots(k).srcLockOk = false;
        obj.robots(k).pendHold = false; obj.robots(k).holdPt = []; obj.robots(k).holdTried = false;
        obj.robots(k).srcFreed = false; obj.robots(k).lockWait = 0;
        obj.robots(k).jobSeqSent = obj.robots(k).jobSeqSent + 1;
        obj.robots(k).job = obj.makeJob(obj.robots(k).jobSeqSent,obj.tasks(ti));
        obj.robots(k).prevPt = obj.robots(k).dstPt; obj.robots(k).srcPt = obj.robots(k).job.src.staging(1:2); obj.robots(k).dstPt = obj.robots(k).job.dst.approach(1:2);
        obj.robots(k).stats.assignWait = obj.robots(k).stats.assignWait + (obj.t - T.created);
        obj.ev('ASSIGN',k,sprintf('task %d %s C%d: %s -> %s',T.id,T.type,j,obj.slots(T.src).id,dname));
        obj.say('[FLEET] t=%.0f s  %s takes task %d (%s): C%d  %s -> %s\n',obj.t,r.name,T.id,T.type,j,obj.slots(T.src).id,dname);
        obj.announceOthers(k);
    end
    function announceOthers(obj,k)
        if obj.verbose < 1, return, end
        txt = ''; for q = 1:obj.N, if q ~= k && obj.robots(q).task > 0, txt = [txt sprintf('%s:t%d ',obj.robots(q).name,obj.robots(q).task)]; end, end %#ok<AGROW>
        if ~isempty(txt), obj.say('        other robots are busy with: %s\n',txt); end
    end

    function job = makeJob(obj,seq,T)
        s = obj.slots(T.src); dims = obj.C.TargetDimensions(:).';
        src = struct('id',1,'shelf',s.shelf,'level',s.level,'xyz',s.xyz,'staging',s.staging,'dock',s.dock,'lift',s.lift,'dims',dims,'faceX',s.faceX,'name',s.id, ...
            'n',s.n,'faceD',s.faceD,'yaw',s.yaw,'boxYaw',s.boxYaw);
        if T.dstKind == 1
            d = obj.slots(T.dstSlot);
            dst = struct('kind',1,'station',0,'slot',T.dstSlot,'name',d.id,'yaw',d.yaw,'slotXYZ',d.xyz,'approach',d.dock,'n',d.n);
        else
            S = obj.C.Stations(T.dstStation);
            dst = struct('kind',2,'station',T.dstStation,'slot',T.dstSlot,'name',S.name,'yaw',S.yaw,'slotXYZ',S.slotXYZ(T.dstSlot,:),'approach',S.approach(T.dstSlot,:),'n',[1 0]);
        end
        job = struct('seq',seq,'task',T.id,'type',T.type,'carton',T.item,'src',src,'dst',dst);
    end

    % ------------------------------------------------------------------ traffic
    function traffic(obj,dt)
        TP = obj.TP; N = obj.N; P = zeros(N,2); act = false(1,N); manip = false(1,N); cj = false(1,N); pr = zeros(1,N);
        pts = cell(1,N);
        for k = 1:N
            r = obj.robots(k); P(k,:) = r.pose(1:2);
            act(k) = r.rep && any(r.state == [1 5 7]);
            manip(k) = r.rep && any(r.state == [10 2 3 4 6 8 9 11]);
            cj(k) = r.carry || (manip(k) && any(r.state == [10 2 3]) && r.task > 0);      % a robot that is about to pick a box will stick out by the box in a moment: keep that clearance already
            pr(k) = obj.prio(k);
            pts{k} = Phase3Fleet.lookahead(r,TP.look);
        end
        obj.waitFor = zeros(0,2);
        if size(obj.yieldHold,1) ~= N, obj.yieldHold = zeros(N); end
        gone = (obj.t - obj.precT > 2.5) & (obj.prec | obj.prec.') & ~(obj.yieldHold > obj.t | obj.yieldHold.' > obj.t);            % pair has no conflict any more for 2.5 s: forget who had right of way (not while one of them is pulling over for the other: that decision stands for yieldT seconds)
        obj.prec(gone) = false;
        for i = 1:N
            r = obj.robots(i); scale = 1; blocker = 0; headNow = false; ovr = obj.t < obj.robots(i).override;
            if act(i)
                for j = 1:N
                    if j == i || ~obj.robots(j).rep, continue, end
                    if norm(P(j,:) - P(i,:)) > 7, continue, end
                    jStopped = obj.robots(j).scale < 0.1;            % a robot that has stopped (yielding / waiting) is a wall for everyone
                    movJ = act(j) && ~jStopped;
                    ex = (TP.carryExtra+0.08)*(obj.robots(i).carry + cj(j)) + TP.armExtra*manip(j) + TP.stowExtra*(~movJ && ~manip(j) && ~obj.robots(j).carry);      % a parked robot keeps a wider berth too (stowed arm / mast / path-following error); carried boxes and an arm that is out stick out of the chassis circle
                    Rc = TP.rc + ex;
                    if ~movJ, Rc = TP.rcStill + ex; end   % a robot standing still (waiting, working at a rack, stopped) is passed at body width + margin, never waited for
                    headOn = false; fT = 1; dHc = TP.dHard + ex + 0.4*(TP.vFree + (TP.vCarry - TP.vFree)*obj.robots(i).carry)*obj.robots(i).scale;      % stop distance includes the carried boxes
                    if movJ         % both robots move: compare WHEN each one is at each point of the shared routes (pre-path planning in space and time)
                        vi = (TP.vFree + (TP.vCarry - TP.vFree)*obj.robots(i).carry)*max(0.3,obj.robots(i).scale);
                        vj = (TP.vFree + (TP.vCarry - TP.vFree)*obj.robots(j).carry)*max(0.3,obj.robots(j).scale);
                        [fT,headOn] = Phase3Fleet.conflictTime(pts{i},vi,pts{j},vj,Rc,dHc);
                        if fT >= 1, continue, end
                        sc = dHc + 2.0*fT;
                    else
                        sc = Phase3Fleet.conflictArc(pts{i},pts{j},P(j,:),false,Rc,dHc);
                        if isinf(sc), continue, end
                        fT = min(1,max(0,(sc - dHc)/2.0));
                    end
                    % ---- right of way is decided ONCE per encounter and kept (sticky): no robot changes its mind every step, so nobody flip-flops
                    obj.precT(i,j) = obj.t; obj.precT(j,i) = obj.t;
                    if ~obj.prec(i,j) && ~obj.prec(j,i)
                        jAhead = Phase3Fleet.isAhead(P(i,:),pts{i},P(j,:));
                        iAhead = movJ && Phase3Fleet.isAhead(P(j,:),pts{j},P(i,:));
                        if obj.yieldHold(i,j) > obj.t, iWins = false;                    % i is already pulling over for j (face to face): stays so
                        elseif obj.yieldHold(j,i) > obj.t, iWins = true;                 % j is pulling over for me
                        elseif jStopped && obj.robots(j).blockedBy == i, iWins = true;      % j stopped because it gives way to me
                        elseif jAhead && ~iAhead, iWins = false;                        % j is in front of me: I follow
                        elseif iAhead && ~jAhead, iWins = true;                         % I am in front: j follows me
                        else, iWins = pr(i) > pr(j); end                                % crossing / head-on: right of way (working > carrying > task > waited > number)
                        if iWins, obj.prec(j,i) = true; else, obj.prec(i,j) = true; end
                        obj.precNew = obj.precNew + 1;
                    end
                    yield = obj.prec(i,j);
                    if yield
                        if fT < scale, scale = fT; blocker = j; end
                        if headOn && fT < 0.6, headNow = true; end
                        if headOn && movJ && ~ovr && norm(P(i,:) - P(j,:)) < 7 && obj.t - obj.robots(i).evadeT > 6 && obj.t - obj.robots(i).replanT > 3 && ~(obj.yieldHold(j,i) > obj.t)      % never both: if j is the one who pulls over, i drives on
                            % FACE TO FACE: the one WITHOUT right of way (carrying > empty, then planning order) pulls over at once - it picks a free spot beside the oncoming robot's route, the other one drives on
                            ptp = obj.pullOverPoint(i,j,P,pts{j});
                            if ~isempty(ptp)
                                obj.robots(i).evade = [ptp, obj.robots(i).evade(3) + 1]; obj.robots(i).evadeT = obj.t; obj.pullOvers = obj.pullOvers + 1; obj.yieldHold(i,j) = obj.t + obj.yieldT; obj.yieldHold(j,i) = 0;
                                obj.ev('EVADE',i,sprintf('%s meets %s head-on -> pulls over to (%.1f, %.1f), %s keeps its way',r.name,obj.robots(j).name,ptp,obj.robots(j).name));
                                obj.say('[FLEET] %s and %s head-on: %s pulls over, %s drives on\n',r.name,obj.robots(j).name,r.name,obj.robots(j).name);
                            end
                        end
                    end
                end
            end
            % ---- hard safety bubble (last line of defence, whatever the plan says): never drive INTO a robot that is closer than dHard
            closeNow = false; dmin = inf;
            for j = 1:N
                if j == i || ~obj.robots(j).rep || ~act(i), continue, end
                d = norm(P(i,:) - P(j,:)); dmin = min(dmin,d);
                still = ~(act(j) && obj.robots(j).scale >= 0.1) && ~manip(j);          % parked / stopped robot: may be passed closer (body + 0.15), moving or working ones not
                dHj = TP.dHard; dSj = TP.dSlow; if still, dHj = TP.body + 0.15; dSj = TP.body + 0.40; end
                cx = (TP.carryExtra+0.08)*(obj.robots(i).carry + cj(j)) + TP.armExtra*manip(j) + TP.stowExtra*(still && ~manip(j) && ~obj.robots(j).carry); vbr = (TP.vFree + (TP.vCarry - TP.vFree)*obj.robots(i).carry)*obj.robots(i).scale; dHj = dHj + cx; dSj = dSj + cx; brk = 0.4*vbr;      % + reaction distance (pose / command delay about 0.4 s)
                if ~(act(j) && obj.robots(j).scale >= 0.5) && size(pts{i},1) >= 2         % robot j stands: the reaction distance counts only when my own route comes closer than the safety distance
                    if min(sqrt(sum((pts{i}(1:min(end,6),:) - P(j,:)).^2,2))) >= dHj, brk = 0; end
                end
                dHj = dHj + brk; dSj = dSj + brk;      % a carried box sticks out: more room
                if d >= dSj, continue, end
                clos = false;
                if size(pts{i},1) >= 2
                    q = pts{i}(min(3,size(pts{i},1)),:); qj = P(j,:);
                    if act(j) && obj.robots(j).scale >= 0.1 && size(pts{j},1) >= 2, qj = pts{j}(min(3,size(pts{j},1)),:); end      % a stopped robot does not move on
                    clos = norm(q - qj) < d - 0.02;                % distance to j would shrink on the next step
                end
                if ~(act(j) && obj.robots(j).scale >= 0.5) && size(pts{i},1) >= 2     % robot j is standing / stopped / creeping: my own path must not run into the spot where it is now
                    seg = pts{i}(1:min(size(pts{i},1),3),:);           % first ~0.8 m of my route
                    if min(sqrt(sum((seg - P(j,:)).^2,2))) < min(TP.body + 0.2, d - 0.02), clos = true; end
                end
                if ~clos, continue, end
                closeNow = true;
                if ovr && d >= TP.body + 0.12 + cx, continue, end      % wait-cycle override: this robot goes first (still never touches)
                jYields = obj.robots(j).blockedBy == i && act(j);
                if d < dHj && jYields && obj.robots(j).scale >= 0.5 && d >= TP.body + 0.12 + cx, continue, end     % the other one is stopping for me
                if d < dHj && (pr(j) > pr(i) || ~act(j) || (obj.robots(j).scale < 0.1 && ~jYields))
                    scale = 0; if blocker == 0, blocker = j; end
                elseif d < dSj && pr(j) > pr(i)
                    scale = min(scale,max(0.15,(d - dHj)/(dSj - dHj))); if blocker == 0, blocker = j; end
                elseif d < TP.body + 0.12 + cx
                    scale = 0;                                  % almost touching and still closing: both stop, the one without right of way plans a way out
                elseif d < dHj
                    scale = min(scale,0.25*(cx == 0));          % with a carried box / arm out there is no creeping inside the safety distance                    % I have right of way but we are close: creep, the other one is stopping
                end
            end
            if act(i) && size(pts{i},1) >= 2 && (r.holdT > TP.hold1 || obj.t < r.escUntil)
                segMin = inf; dminE = inf;
                for j = 1:N
                    if j == i || ~obj.robots(j).rep, continue, end
                    exj = TP.carryExtra*(r.carry + obj.robots(j).carry) + TP.armExtra*manip(j);
                    segMin = min(segMin,min(sqrt(sum((pts{i}(1:min(4,end),:) - P(j,:)).^2,2))) - exj); dminE = min(dminE,norm(P(i,:) - P(j,:)) - exj);
                end
                if dminE >= TP.body + 0.05 && segMin >= min(dminE,TP.body + 0.1) - 0.03
                    scale = max(scale,0.3);       % stand-off escape: it has planned a way out and that way does not bring it closer to anybody -> creep out
                    if obj.t >= r.escUntil, obj.robots(i).escUntil = obj.t + 1.5; end
                end
            end
            scale = min(scale,r.scale + 1.0*dt);          % smooth start-up: speed may rise by 1.0/s at most (braking is immediate) -> no stop/go flicker
            if r.pendHold, scale = 0; blocker = 0; end    % waits for its shelf face at a safe distance: standing still (the others pass it like any parked robot)
            obj.robots(i).scale = scale; obj.robots(i).blockedBy = blocker;
            if scale < 0.05 && act(i) && ~r.pendHold, obj.robots(i).holdT = r.holdT + dt; else, obj.robots(i).holdT = 0; end
            if blocker > 0 && scale < 0.3, obj.waitFor(end+1,:) = [i blocker]; end %#ok<AGROW>
            if headNow, obj.robots(i).headT = r.headT + dt; else, obj.robots(i).headT = 0; end
            lim = TP.hold1; if blocker > 0 && act(blocker) && obj.robots(blocker).scale > 0.3, lim = TP.holdMoving; end     % the other robot is just passing: wait for it
            gapNeed = TP.replanGap; if blocker > 0 && ~(act(blocker) && obj.robots(blocker).scale > 0.3), gapNeed = 2.5; end
            if blocker > 0 && manip(blocker) && scale < 0.3
                % the blocker is WORKING (stand / rack): estimated remaining work time decides - short -> wait for it, long -> drive around it NOW
                eta = obj.workEta(blocker); obj.robots(i).waitEta = eta;
                if eta <= 8, lim = min(10,eta + 2); gapNeed = max(gapNeed,eta + 3); else, lim = 0.8; gapNeed = 2.0; end
            end      % blocked by a robot that STANDS (working at a rack): its reach may have changed since my last plan - plan again soon (no flip-flop issue, nobody moves)
            if (obj.robots(i).holdT > lim || (obj.robots(i).headT > TP.headOnT && obj.robots(i).holdT > 1.0)) && obj.t - r.replanT > gapNeed
                obj.robots(i).replanSeq = r.replanSeq + 1; obj.robots(i).replanT = obj.t; obj.replans = obj.replans + 1;
                obj.ev('REPLAN',i,sprintf('%s waited %.1f s behind %s -> replan around robots',r.name,obj.robots(i).holdT,obj.robots(max(1,blocker)).name));
                obj.say('[FLEET] %s must give way to %s -> plans a way around (right of way: %s)\n',r.name,obj.robots(max(1,blocker)).name,obj.robots(max(1,blocker)).name);
            end
            if obj.robots(i).holdT > 25, obj.robots(i).boost = r.boost + 1; end
            if obj.robots(i).holdT == 0, obj.robots(i).boost = 0; end
        end
        if size(obj.waitFor,1) >= 2
            [cyc,w] = Phase3Fleet.hasCycle(obj.waitFor,N);
            if cyc && obj.t > obj.robots(w).override
                obj.deadlocks = obj.deadlocks + 1; obj.robots(w).override = obj.t + 4;
                obj.ev('CYCLE',w,sprintf('wait cycle %s -> %s goes first (careful, 4 s)',mat2str(obj.waitFor),obj.robots(w).name));
                obj.say('[FLEET] wait cycle %s -> %s goes first for a few seconds\n',mat2str(obj.waitFor),obj.robots(w).name);
            end
        end
        for i = 1:N          % cycle override: creeps, but never inside the hard bubble
            if obj.t < obj.robots(i).override && act(i)
                d = inf; for j = 1:N, if j ~= i && obj.robots(j).rep, d = min(d,norm(P(i,:)-P(j,:))); end, end
                obj.robots(i).scale = max(obj.robots(i).scale,min(0.6,max(0,(d-TP.dHard)/0.8)));
            end
        end
        % ---- stand-off breaker: two robots that stand still, closer than their reach allows, block each other.  The one with the LOWER priority backs off
        %      (straight away from the other one, onto free floor) and then plans its way again - so a stand-off never lasts and nobody has to touch.
        for i = 1:N
            r = obj.robots(i); j = r.blockedBy;
            if ~act(i) || j < 1 || r.scale >= 0.1 || r.holdT < 1.0 || ~obj.robots(j).rep, continue, end
            waitJ = obj.robots(j).lockHold || obj.robots(j).pendHold;       % j STANDS at its waiting point (station place / shelf face not free): it is not blocked by traffic, it is simply parked in the way
            mutual = act(j) && ((obj.robots(j).scale < 0.1 && obj.robots(j).holdT >= 0.5) || waitJ);
            inReach = manip(j) && r.holdT >= 1.5 && norm(P(i,:) - P(j,:)) < TP.body + TP.armExtra + TP.carryExtra*(r.carry + obj.robots(j).carry) + 0.35;     % stuck INSIDE the reach of a robot that is working: the planner cannot route around from there
            if ~mutual && ~inReach, continue, end
            e = i; o = j; if mutual && pr(j) < pr(i), e = j; o = i; end
            if mutual && waitJ && ~(r.lockHold || r.pendHold), e = j; o = i; end      % the robot that is only WAITING pulls over for the one that holds its place and wants to go on
            if mutual && obj.yieldHold(o,e) > obj.t, tmp = e; e = o; o = tmp; end         % the robot that already pulls over for the other one stays the yielder
            if obj.t - obj.robots(e).evadeT < 6, continue, end
            pt = obj.evadePoint(e,o,P);
            if ~isempty(pt)
                obj.robots(e).evade = [pt, obj.robots(e).evade(3) + 1]; obj.robots(e).evadeT = obj.t; obj.yieldHold(e,o) = obj.t + obj.yieldT; obj.yieldHold(o,e) = 0;
                if obj.robots(e).lockHold || obj.robots(e).pendHold, obj.robots(e).holdPt = pt; obj.robots(e).evIssued = obj.t; end      % it waits at the new point from now on, not back in the way
                obj.ev('EVADE',e,sprintf('%s backs off to (%.1f, %.1f) to let %s pass',obj.robots(e).name,pt,obj.robots(o).name));
                obj.say('[FLEET] %s backs off to let %s pass\n',obj.robots(e).name,obj.robots(o).name);
            end
        end
        % separation bookkeeping
        low = inf; lp = [0 0];
        for i = 1:N-1
            for j = i+1:N
                if obj.robots(i).rep && obj.robots(j).rep
                    d = norm(P(i,:)-P(j,:)) - TP.carryExtra*(obj.robots(i).carry + obj.robots(j).carry) - TP.armExtra*(manip(i) + manip(j));     % reach-aware: carried boxes and arms that are out count
                    if d < low, low = d; lp = [i j]; end
                end
            end
        end
        if low < obj.minSep, obj.minSep = low; end
        if low < TP.body && ~obj.colFlag
            obj.colFlag = true; obj.collisions = obj.collisions + 1;
            obj.ev('COLLISION',lp(1),sprintf('%s and %s centres %.2f m apart at (%.1f,%.1f)',obj.robots(lp(1)).name,obj.robots(lp(2)).name,low,P(lp(1),1),P(lp(1),2)));
            obj.say('[FLEET] !! %s and %s are only %.2f m apart at (%.1f, %.1f)\n',obj.robots(lp(1)).name,obj.robots(lp(2)).name,low,P(lp(1),1),P(lp(1),2));
        elseif low >= TP.body + 0.12, obj.colFlag = false; end
        if low < TP.dHard && ~obj.sepFlag, obj.sepFlag = true; obj.nearMiss = obj.nearMiss + 1;
        elseif low >= TP.dHard + 0.1, obj.sepFlag = false; end
    end

    function eta = workEta(obj,k)
        % remaining time [s] until robot k has left its working envelope (typical phase durations of the arm sequence minus the time already spent in the phase)
        r = obj.robots(k); st = r.state;
        sts = [10 2 3 4 6 8 9 11]; typ = [8 12 10 12 12 6 10 30]; q = find(st == sts,1);
        if isempty(q), eta = 0; return, end
        rest = typ(q) - r.stateT; if st == 11, rest = max(rest,10); end
        eta = max(1,rest);
        if any(st == [10 2 3]), eta = eta + 12 + 10 + 6; elseif st == 4, eta = eta + 12 + 10; elseif st == 6, eta = eta + 6 + 10; elseif st == 8, eta = eta + 10; end    % phases still to come before the robot drives away
    end
    function pt = pullOverPoint(obj,e,o,P,ptsO)
        % head-on pre-emption: free floor point off the oncoming robot's route (and off the other robots), close to e, reachable in a straight free line
        TP = obj.TP; pt = []; pe = P(e,:);
        ex = TP.carryExtra*(obj.robots(e).carry + obj.robots(o).carry) + TP.armExtra*any(obj.robots(o).state == [10 2 3 4 6 8 9 11]);
        need = TP.body + ex + 0.55; best = inf;
        for rad = [1.2 1.8 2.5 3.5]
            for ang = (0:15:345)*pi/180
                cand = pe + rad*[cos(ang) sin(ang)];
                if min(sqrt(sum((ptsO - cand).^2,2))) < need || norm(cand - P(o,:)) < need + 0.3, continue, end
                ok = true; for q = 0:4, if ~obj.isFree(pe + (cand - pe)*q/4), ok = false; break, end, end
                if ~ok, continue, end
                for m = 1:obj.N
                    if m == e || m == o || ~obj.robots(m).rep, continue, end
                    if norm(cand - P(m,:)) < TP.body + 0.5, ok = false; end
                end
                if ok && rad < best, best = rad; pt = cand; end
            end
            if ~isempty(pt), return, end
        end
    end
    function pt = evadePoint(obj,e,o,P)
        % free floor point straight away from robot o (several directions tried), far enough that the reach of both robots is clear
        TP = obj.TP; pt = []; pe = P(e,:); po = P(o,:);
        ex = TP.carryExtra*(obj.robots(e).carry + obj.robots(o).carry) + TP.armExtra*any(obj.robots(o).state == [10 2 3 4 6 8 9 11]);
        need = TP.body + 0.15 + ex + 0.9; u = pe - po; if norm(u) < 0.05, u = [cos(obj.robots(e).pose(3)) sin(obj.robots(e).pose(3))]; end, u = u/norm(u);
        % first choice: SIDEWAYS (across the line between the two robots) toward the side with more free floor - backing straight away would only slide along the
        % lane the other robot is driving down, and the stand-off would repeat every metre
        sd = [-u(2) u(1)]; bestRun = 0; bestC = [];
        for sg = [1 -1]
            v = sg*sd; run = 0;
            for q = 1:12, if obj.isFree(pe + 0.5*q*v), run = q; else, break, end, end
            lat = max(2.0,need + 0.3);
            if run*0.5 >= lat + 0.3 && run > bestRun
                cand = pe + lat*v; ok = true;
                for m = 1:obj.N, if m ~= e && obj.robots(m).rep && norm(cand - P(m,:)) < TP.body + 0.4 + ex*(m == o), ok = false; end, end
                if ok, bestRun = run; bestC = cand; end
            end
        end
        if ~isempty(bestC), pt = bestC; return, end
        for scl = [1 1.5 2]
            dist = max(1.8,(need - norm(pe - po) + 0.4))*scl;
            for ang = [0 40 -40 80 -80 120 -120] *pi/180
                c = cos(ang); sn = sin(ang); v = [c -sn; sn c]*u(:); cand = pe + dist*v(:).';
                ok = true;
                for q = 0:3, if ~obj.isFree(pe + (cand - pe)*q/3), ok = false; break, end, end
                if ok
                    for m = 1:obj.N
                        if m == e || ~obj.robots(m).rep, continue, end
                        if norm(cand - P(m,:)) < TP.body + 0.4 + ex*(m == o), ok = false; end
                    end
                end
                if ok, pt = cand; return, end
            end
        end
    end
    % ------------------------------------------------------------------ M* multirobot path planning (planner 'mstar')
    function mstarStep(obj)
        % The controllers plan the INDIVIDUAL shortest route of their robot (individual optimal policy, A* on the inflated map).  Whenever a new route has been published, M*
        % (fleetMStar: Wagner & Choset, subdimensional expansion) plans ALL driving robots together on a coarse grid: robots whose shortest routes do not interfere keep
        % them; robots whose routes collide are coupled (collision set) and searched jointly, which gives them collision-free routes (detours).  A changed route is
        % sent to the robot (cmd.mroute) and followed like any other route.  Waiting is not part of a route: the speed layer (traffic) does it.  Never a robot stops because of M*.
        if ~strcmp(obj.planner,'mstar'), return, end
        Dr = [];
        for j = 1:obj.N
            r = obj.robots(j);
            if r.rep && any(r.state == [1 5 7]) && ~r.pendHold && ~isempty(r.wp) && ~r.planReq && ~r.faultFlag && obj.t - r.evadeT > 16 && norm(r.wp(end,:) - r.goal(1:2)) < 2.0, Dr(end+1) = j; end %#ok<AGROW>
        end
        if numel(Dr) < 2, obj.mSig = []; return, end
        sig = [Dr; [obj.robots(Dr).planSeq]];
        if obj.t - obj.mLast < 1.0 || (isequal(sig,obj.mSig) && obj.t - obj.mLast < 4.0), return, end
        obj.mSig = sig; obj.mLast = obj.t; n = numel(Dr); TP = obj.TP; ms = obj.mstat;
        try
            g = obj.mgridGet(); starts = zeros(n,2); goals = zeros(n,2); tails = cell(n,1); carry = false(n,1); Fm = false(g.ny,g.nx,n); isDr = false(1,obj.N); isDr(Dr) = true;
            for q = 1:n
                r = obj.robots(Dr(q)); wq = r.wp; starts(q,:) = r.pose(1:2); carry(q) = r.carry; tl = zeros(0,2); gl = wq(end,:);
                if size(wq,1) >= 2 && norm(wq(end,:) - wq(end-1,:)) <= 2.5, tl = wq(end,:); gl = wq(end-1,:); end       % the last short straight leg (gate -> table / dock) stays as it is
                goals(q,:) = gl; tails{q} = tl;
            end
            for q = 1:n
                Fq = g.FN; if carry(q), Fq = g.FC; end
                for j = 1:obj.N
                    if isDr(j) || ~obj.robots(j).rep, continue, end
                    rj = obj.robots(j); wk = any(rj.state == [10 2 3 4 6 8 9 11]);
                    ex = TP.armExtra*wk + TP.carryExtra*rj.carry + TP.stowExtra*(~wk && ~rj.carry);
                    rr = min([TP.rcStill + ex, max(0.3,norm(rj.pose(1:2) - starts(q,:)) - 0.12), max(0.3,norm(rj.pose(1:2) - goals(q,:)) - 0.12)]);
                    Fq = Fq & ~Phase3Fleet.discMask(g,rj.pose(1:2),rr);
                end
                Fm(:,:,q) = Fq;
            end
            pr = zeros(1,n); for q = 1:n, pr(q) = obj.prio(Dr(q)); end
            [~,ord] = sort(pr,'ascend'); w = zeros(n,1); w(ord) = 1 + 0.04*(0:n-1).';           % higher priority (planned earlier / working) = its route is kept shorter, the others give way
            ds = zeros(n); for a = 1:n, for b = 1:n, if a ~= b, ds(a,b) = TP.rc + TP.carryExtra*(carry(a) + carry(b)); end, end, end
            [paths,info] = fleetMStar(Fm,g.res,g.x0,starts,goals,w,ds,struct('maxNodes',obj.mMaxNodes,'maxGroup',obj.mMaxGroup,'tMax',obj.mTmax));
        catch ME
            ms.fail = ms.fail + 1; ms.reason = ME.message; obj.mstat = ms; obj.mFail = obj.mFail + 1;
            if obj.mFail >= 3, obj.planner = 'prioritised'; obj.TP.plan.maxTries = 3; obj.say('[FLEET] M* switched off (%s) - the prioritised planner is used instead\n',ME.message); obj.ev('MSTAR',0,['switched off: ' ME.message]); end
            return
        end
        ms.runs = ms.runs + 1; ms.nodes = ms.nodes + info.nodes; ms.time = ms.time + info.time; ms.maxTime = max(ms.maxTime,info.time);
        timed = false;
        if ~info.ok
            % joint M* ran into its limit -> TIMED level: prioritised space-time search (waiting is part of the plan), robots planned in priority order
            try
                [~,ordT] = sort(pr,'descend'); sts = struct('xy',{},'req',{},'t0',{},'t1',{});
                [pathsT,infoT] = fleetSTPlan(Fm,g.res,g.x0,starts,goals,ordT,ds,sts,struct('tMax',obj.mTmax,'maxNodes',40000));
            catch ME2
                infoT = struct('ok',false,'reason',ME2.message,'time',0,'nodes',0); pathsT = {};
            end
            ms.nodes = ms.nodes + infoT.nodes; ms.time = ms.time + infoT.time; ms.maxTime = max(ms.maxTime,infoT.time);
            if infoT.ok
                timed = true; paths = pathsT; ms.timed = ms.timed + 1; info.deviated = true(n,1); info.coupled = true; info.maxGroup = n; info.nodes = infoT.nodes; info.time = infoT.time;
                obj.ev('MSTAR',0,sprintf('joint search limit (%s) -> timed plan with waiting (%d robots, %d steps of %.1f s, %.2f s)',info.reason,n,infoT.steps,infoT.dt,infoT.time));
            else
                ms.fail = ms.fail + 1; ms.reason = info.reason; obj.mstat = ms; obj.ev('MSTAR',0,['no joint plan: ' info.reason '; timed: ' infoT.reason]); return
            end
        end
        obj.mFail = 0;
        if info.coupled, ms.coupled = ms.coupled + 1; ms.maxGroup = max(ms.maxGroup,info.maxGroup); end
        for q = 1:n
            j = Dr(q); r = obj.robots(j);
            if ~info.deviated(q) || obj.t - r.mpushT < 5.0, continue, end
            nw = obj.mstarRoute(paths{q},starts(q,:),Fm(:,:,q),g,tails{q},goals(q,:));
            old = [r.pose(1:2); r.wp]; if size(nw,1) < 2 || pathLength(old) < 3.0, continue, end
            dev = Phase3Fleet.polyDev(nw,old); if dev < 0.5, continue, end
            obj.robots(j).mroute = struct('seq',r.mroute.seq + 1,'wp',nw(2:end,:)); obj.robots(j).mpushT = obj.t;
            ms.pushed = ms.pushed + 1; ms.robotsCoupled = ms.robotsCoupled + 1;
            names = strjoin({obj.robots(Dr(info.deviated | ((1:n).' == q))).name},' + ');
            obj.ev('MSTAR',j,sprintf('coupled search (%d robots, %d nodes, %.2f s): %s gets a new route (%.1f m, was %.1f m, max difference %.1f m)',max(2,info.maxGroup),info.nodes,info.time,r.name,pathLength(nw),pathLength(old),dev));
            if obj.verbose >= 1, obj.say('[FLEET] t=%.0f s  M*: %s were on a collision course - %s takes a new collision-free route (%.1f m instead of %.1f m)\n',obj.t,names,r.name,pathLength(nw),pathLength(old)); end
        end
        obj.mstat = ms;
    end
    function g = mgridGet(obj)
        % coarse grid (0.5 m) for M*: a coarse cell is free only when all four 0.25 m cells of the controller's inflated map are free -> every M* route lies inside the free space of the normal planner
        if ~isempty(obj.mgrid), g = obj.mgrid; return, end
        WH = obj.C.WH; res = 0.5; fr = 0.25; nx = ceil(WH.X/res); ny = ceil(WH.Y/res);
        [fx,fy] = meshgrid(((1:2*nx) - 0.5)*fr - WH.X/2,((1:2*ny) - 0.5)*fr);
        FN = obj.sampleFree(fx,fy,false); FC = obj.sampleFree(fx,fy,true);
        g = struct('res',res,'nx',nx,'ny',ny,'x0',WH.X/2,'FN',Phase3Fleet.coarse(FN),'FC',Phase3Fleet.coarse(FC));
        obj.mgrid = g;
    end
    function F = sampleFree(obj,fx,fy,carry)
        if ~isempty(obj.gridFree), F = obj.gridFree(fx,fy,carry); return, end
        WH = obj.C.WH; m = WH.navMap; if carry && isfield(WH,'navMapCarry'), m = WH.navMapCarry; end
        F = reshape(checkOccupancy(m,[fx(:) + WH.X/2, fy(:)]) == 0,size(fx));
    end
    function wp = mstarRoute(~,pc,start,Fq,g,tail,goal)
        % M* cell path -> waypoints: waiting steps dropped, straightened inside the free cells (line of sight), exact start / goal, the short final leg appended
        keep = [true; any(diff(pc,1,1) ~= 0,2)]; pc = pc(keep,:); pc(1,:) = start;
        out = pc(1,:); a = 1; np = size(pc,1);
        while a < np
            b = np;
            while b > a + 1 && ~Phase3Fleet.losFree(Fq,g,pc(a,:),pc(b,:)), b = b - 1; end
            out(end+1,:) = pc(b,:); a = b; %#ok<AGROW>
        end
        if norm(out(end,:) - goal) > 0.05, out(end+1,:) = goal; end %#ok<AGROW>
        out = [out; tail]; wp = fleetSimplify(out,0.10);
    end
    function tf = isFree(obj,xy)
        tf = true;
        try
            if ~isempty(obj.freeFcn), tf = obj.freeFcn(xy); return, end
            tf = checkOccupancy(obj.C.WH.navMap,[xy(1) + obj.C.WH.X/2, xy(2)]) == 0;
        catch
            tf = true;
        end
    end
    % ------------------------------------------------------------------ metrics
    function metricsStep(obj,dt)
        for k = 1:obj.N
            r = obj.robots(k); if ~r.rep, continue, end
            d = norm(r.pose(1:2) - r.lastPose(1:2)); if d < 1.0, obj.robots(k).stats.dist = r.stats.dist + d; if r.carry, obj.robots(k).stats.distCarry = r.stats.distCarry + d; end, end
            if any(r.state == [10 2 3 4 6 8 9 11]), obj.robots(k).stats.tArm = r.stats.tArm + dt; end      % arm / mast moving (scan, grasp, lift, place)
            % ---- drive energy from the REAL motion of the robot: wheel power = (m a v + rolling friction m g crr v + turning: J w' w + scrub torque w) / drive efficiency; braking is not recovered
            m_ = obj.massBase + obj.massArm + obj.massBox*r.carry; v_ = 0; w_ = 0;
            if d < 1.0 && dt > 0, v_ = d/dt; dth = atan2(sin(r.pose(3) - r.lastPose(3)),cos(r.pose(3) - r.lastPose(3))); w_ = dth/dt; end
            vf0 = r.stats.vf; wf0 = r.stats.wf; vf1 = vf0 + 0.25*(v_ - vf0); wf1 = wf0 + 0.25*(w_ - wf0);
            Rch = 0.34; Jz = 0.5*m_*Rch^2; g_ = 9.81;
            Pd = max(0,m_*((vf1 - vf0)/max(dt,1e-3))*vf1) + obj.crr*m_*g_*vf1 + max(0,Jz*((wf1 - wf0)/max(dt,1e-3))*wf1) + obj.crr*m_*g_*Rch*abs(wf1);
            obj.robots(k).stats.eDrive = r.stats.eDrive + Pd/obj.etaDrive*dt; obj.robots(k).stats.vf = vf1; obj.robots(k).stats.wf = wf1;
            obj.robots(k).lastPose = r.pose;
            if r.task > 0, obj.robots(k).stats.busy = r.stats.busy + dt; else, obj.robots(k).stats.idle = r.stats.idle + dt; end
            if r.pendHold || r.lockHold, obj.robots(k).stats.queue = r.stats.queue + dt;       % waiting for a shelf face / station place (not held back by another robot)
            elseif r.scale < 0.3, obj.robots(k).stats.wait = r.stats.wait + dt; end
        end
    end

    function checkDone(obj)
        if obj.finishing
            busy = false; for i = 1:numel(obj.tasks), if any(strcmp(obj.tasks(i).status,{'QUEUED','ACTIVE','RUNNING','HELD'})), busy = true; end, end
            for k = 1:obj.N, if obj.robots(k).task > 0, busy = true; end, end
            if ~busy, for k = 1:obj.N, obj.robots(k).finish = true; end, end
            if ~busy && all([obj.robots.state] == 12 | [obj.robots.fault] == 1)
                if ~obj.allDone
                    if obj.autoFinished
                        if isinf(obj.parkT), obj.parkT = obj.t; obj.say('[FLEET] all robots are home - the 3-D window closes in 5 s\n'); end
                        if obj.t - obj.parkT >= 5
                            obj.allDone = true; obj.stopFlag = true; obj.ev('FLEET',0,'all robots parked, stop'); obj.metricsWrite(true);
                            obj.say('\n[P3] ===== AUTO MODE FINISHED: every packing station is full, the staging area is empty, all robots are home =====\n');
                        end
                    else
                        obj.allDone = true; obj.stopFlag = true; obj.ev('FLEET',0,'all robots parked, stop'); obj.metricsWrite(true);
                    end
                end
            end
        end
    end

    % ------------------------------------------------------------------ vision (camera lease)
    function visionStep(obj)
        if strcmp(obj.visionMode,'off')
            for k = 1:obj.N, obj.robots(k).vis = 'unavail'; end
            if obj.watch > 0, obj.camSel = obj.watch; end
            return
        end
        owner = 0;
        for k = 1:obj.N
            r = obj.robots(k);
            if ~r.scanReq
                obj.robots(k).vis = 'none'; obj.robots(k).visFrames = 0; obj.robots(k).visHits = 0; obj.robots(k).visUnk = 0; obj.robots(k).visWait = 0;
            end
        end
        want = find([obj.robots.scanReq]);
        if obj.watch > 0 && any(want == obj.watch), owner = obj.watch;
        elseif obj.watch == 0 && ~isempty(want)
            cur = obj.camSel; if any(want == cur), owner = cur; else, [~,ix] = min([obj.robots(want).visWait]*-1); owner = want(ix); end
        end
        if obj.watch > 0, obj.camSel = obj.watch; elseif owner > 0, obj.camSel = owner; end
        for k = want
            r = obj.robots(k);
            if k == owner && obj.camSel == k
                if ~any(strcmp(r.vis,{'pass','fail','unavail'})), obj.robots(k).vis = 'pending'; end       % 'unavail' stays (it was flipped back to 'pending' every frame and counted again: 2 real cases were reported as 33)
            else
                obj.robots(k).visWait = r.visWait + 0.1;
                if r.visWait > 6 && ~strcmp(r.vis,'unavail'), obj.robots(k).vis = 'unavail'; obj.visionCount(4) = obj.visionCount(4) + 1; obj.ev('VISION',k,sprintf('%s: camera busy -> geometric scan only',r.name)); end
                if ~any(strcmp(r.vis,{'unavail','pass','fail'})), obj.robots(k).vis = 'wait'; end
            end
        end
    end
    function visionFrame(obj,sel,img,pose,lift)
        if sel < 1 || sel > obj.N, return, end
        r = obj.robots(sel);
        if ~r.scanReq || r.task == 0 || ~strcmp(r.vis,'pending'), return, end
        job = r.job; if isempty(job), return, end
        if abs(lift - p3ScanLift(job.src.xyz(3))) > 0.03, return, end            % the carriage is still on its way to the height the camera needs for this level
        [hit,vinf,known] = fleetVisionCheck(img,pose,lift,job.src.xyz,obj.C);
        if ~known
            obj.robots(sel).vis = 'unavail'; obj.visionCount(4) = obj.visionCount(4) + 1; obj.ev('VISION',sel,sprintf('%s: slot %s at pixel (%.0f,%.0f) is outside the 640x480 picture (lift %.2f m) -> geometric scan only',r.name,job.src.name,vinf.uv(1),vinf.uv(2),lift)); return
        end
        if isempty(obj.vLastSel) || obj.vLastSel ~= sel, obj.vLastSel = sel; obj.vSelT = obj.t; end
        if obj.t - obj.vSelT < 0.7, return, end                         % one camera view is shared by the robots: the first frames after it switched to this robot may still show the previous picture
        if vinf.inside < 0.65, obj.robots(sel).visUnk = r.visUnk + 1; if obj.robots(sel).visUnk >= 25 && r.visFrames < 8, obj.robots(sel).vis = 'unavail'; obj.visionCount(4) = obj.visionCount(4) + 1; obj.ev('VISION',sel,sprintf('%s: slot %s is cut off by the picture edge (lift %.2f m) -> geometric scan only',r.name,job.src.name,lift)); end, return, end
        obj.robots(sel).visFrames = r.visFrames + 1; obj.robots(sel).visHits = r.visHits + double(hit);
        if obj.robots(sel).visFrames >= 8
            if obj.robots(sel).visHits >= 5, obj.robots(sel).vis = 'pass'; obj.visionCount(1) = obj.visionCount(1) + 1;
            else, obj.robots(sel).vis = 'fail'; obj.visionCount(2) = obj.visionCount(2) + 1; end
            obj.ev('VISION',sel,sprintf('%s camera check of %s: %d/8 frames show the carton -> %s',r.name,job.src.name,obj.robots(sel).visHits,obj.robots(sel).vis));
        end
    end

    % ================================================================== operator interface (used by p3.m)
    function dockAdd(obj,n,quiet,ids)
        if nargin < 3, quiet = false; end
        if nargin < 4, ids = []; else, n = numel(ids); end
        if obj.finishing, obj.say('[P3] mission is ending - no new boxes accepted\n'); return, end
        added = [];
        for q = 1:n
            if ~isempty(ids), j = ids(q); if j < 1 || j > obj.M || ~any(strcmp(obj.cartons(j).state,{'RESERVE'})), obj.say('[P3] C%d is not waiting outside (state %s)\n',j,obj.cartons(min(max(j,1),obj.M)).state); continue, end
            else
                j = 0; for z = 1:obj.M, if strcmp(obj.cartons(z).state,'RESERVE'), j = z; break, end, end
                if j == 0, obj.say('[P3] no spare carton left (pool size %d)\n',obj.M); break, end
            end
            si = 0; best = inf;
            for i = 1:numel(obj.slots)
                s = obj.slots(i); if strcmp(s.role,'dock') && strcmp(s.state,'FREE'), c = s.level*100 + s.pos; if c < best, best = c; si = i; end, end
            end
            if si == 0, obj.say('[P3] all %d staging stands are occupied - carton C%d waits outside\n',sum(strcmp({obj.slots.role},'dock')),j); break, end
            obj.slots(si).state = 'OCCUPIED'; obj.slots(si).item = j;
            obj.cartons(j).state = 'DOCK'; obj.cartons(j).slot = si; obj.cartons(j).xyz = obj.slots(si).xyz; obj.cartons(j).cmd = obj.slots(si).xyz; obj.cartons(j).cmdYaw = obj.slots(si).boxYaw;
            obj.cartons(j).drive = true; obj.cartons(j).spawnUntil = obj.t + 1.5; added(end+1) = j; %#ok<AGROW>
            obj.ev('ARRIVAL',0,sprintf('C%d arrives at dock %s',j,obj.slots(si).id));
            if ~quiet, obj.say('[P3] t=%.0f s  carton C%d arrived at the dock (%s)\n',obj.t,j,obj.slots(si).id); else, obj.say('[FLEET] t=%.0f s  new carton C%d arrived at the dock (%s)\n',obj.t,j,obj.slots(si).id); end
        end
    end

    function addOrder(obj,item,station,quiet)
        if nargin < 4, quiet = false; end
        if obj.finishing, obj.say('[P3] mission is ending - no new orders accepted\n'); return, end
        if item > obj.M, obj.say('[P3] carton C%d does not exist (1..%d)\n',item,obj.M); return, end
        if item > 0
            st = obj.cartons(item).state;
            if any(strcmp(st,{'MOVING','STATION','LOST'})) || obj.cartons(item).task ~= 0 && ~strcmp(st,'DOCK'), obj.say('[P3] C%d is %s - order not possible now\n',item,st); return, end
        end
        o = Phase3Fleet.blankOrder(obj.nextOrder,item,station,obj.t); obj.nextOrder = obj.nextOrder + 1;
        obj.orders(Phase3Fleet.nextIx(obj.orders)) = o;
        where = 'any station'; if station > 0, where = obj.stations(station).name; end
        what = 'any stored carton'; if item > 0, what = sprintf('C%d (%s)',item,obj.cartons(item).state); end
        obj.ev('ORDER',0,sprintf('order %d: %s -> %s',o.id,what,where));
        if ~quiet || obj.verbose >= 1, obj.say('[P3] order %d: %s -> %s\n',o.id,what,where); end
        if item > 0 && any(strcmp(obj.cartons(item).state,{'RESERVE'})), obj.say('     (not in the warehouse yet: the order waits; when it arrives at the dock it goes straight to the station - cross-dock)\n'); end
    end

    function manualTask(obj,k,type,item,slotName,station)
        % direct order to one robot: retrieve <slot|carton> to <station>, putaway <carton> to <slot>
        r = obj.robots(k);
        switch type
            case 'retrieve'
                si = obj.slotIndex(slotName); if si == 0, obj.say('[P3] unknown slot %s\n',slotName); return, end
                if ~strcmp(obj.slots(si).state,'OCCUPIED'), obj.say('[P3] slot %s is %s - nothing to retrieve there\n',obj.slots(si).id,obj.slots(si).state); return, end
                j = obj.slots(si).item; if obj.cartons(j).task ~= 0, obj.say('[P3] C%d already has a task\n',j); return, end
                tid = obj.newTask('retrieve',j,si,2,station,0,3,0,k); obj.holdIfBatch(tid);
                obj.say('[P3] %s: retrieve C%d from %s%s\n',r.name,j,obj.slots(si).id,p3tern(strcmp(obj.mode,'batch'),'  (queued - starts with  p3 go)',''));
            case 'putaway'
                if item >= 1 && item <= obj.M && strcmp(obj.cartons(item).state,'RESERVE'), obj.say('[P3] C%d is not in the warehouse yet - it arrives on a staging stand first\n',item); obj.dockAdd(0,true,item); end
                if item < 1 || item > obj.M || ~strcmp(obj.cartons(item).state,'DOCK'), obj.say('[P3] C%d is not waiting at the dock\n',item); return, end
                if obj.cartons(item).task ~= 0, obj.say('[P3] C%d already has a task\n',item); return, end
                d = 0; if ~isempty(slotName), d = obj.slotIndex(slotName); if d == 0, obj.say('[P3] unknown slot %s\n',slotName); return, end, end
                if d > 0 && ~strcmp(obj.slots(d).state,'FREE') || d > 0 && ~strcmp(obj.slots(d).role,'store')
                    obj.askAlternative(k,item,d); return
                end
                tid = obj.newTask('putaway',item,obj.cartons(item).slot,1,0,d,3,0,k); obj.holdIfBatch(tid);
                obj.say('[P3] %s: put away C%d into %s%s\n',r.name,item,p3tern(d>0,obj.slots(max(d,1)).id,'the best free slot'),p3tern(strcmp(obj.mode,'batch'),'  (queued - starts with  p3 go)',''));
        end
    end
    function queuePutaway(obj,item,slotName)
        % any free robot puts dock carton <item> (0 = every carton waiting at the dock) into <slotName> (empty = best free slot)
        if item == 0, list = find(strcmp({obj.cartons.state},'DOCK') & [obj.cartons.task] == 0); else, list = item; end
        if isempty(list), obj.say('[P3] no carton waiting at the dock\n'); return, end
        for j = list(:).'
            if j >= 1 && j <= obj.M && strcmp(obj.cartons(j).state,'RESERVE')       % spare carton: it arrives on a free stand first
                obj.say('[P3] C%d is not in the warehouse yet - it arrives on a staging stand first\n',j); obj.dockAdd(0,true,j);
            end
            if j < 1 || j > obj.M || ~strcmp(obj.cartons(j).state,'DOCK') || obj.cartons(j).task ~= 0, obj.say('[P3] C%d is not waiting at the dock (or already has a task)\n',j); continue, end
            d = 0; if ~isempty(slotName), d = obj.slotIndex(slotName); if d == 0, obj.say('[P3] unknown slot %s\n',slotName); return, end, end
            if d > 0 && (~strcmp(obj.slots(d).state,'FREE') || ~strcmp(obj.slots(d).role,'store')), obj.askAlternative(0,j,d); continue, end
            tid = obj.newTask('putaway',j,obj.cartons(j).slot,1,0,d,1,0,0); obj.holdIfBatch(tid);
            obj.say('[P3] putaway of C%d queued (%s)\n',j,p3tern(d>0,obj.slots(max(d,1)).id,'best free slot'));
        end
    end
    % ------------------------------------------------------------------ Round 17: warehouse view, urgent job, move
    function txt = wmsText(obj,arg)
        % p3 wms [free]: every box and where it is (shelf faces, staging stands D01.., stations, moving, outside); "free" = the free shelf slots instead
        if nargin < 2, arg = ''; end
        L = char(10); isSt = strcmp({obj.slots.role},'store'); ids = {obj.slots.id};
        face = cellfun(@(x) x(1:find(x=='-',1)-1),ids,'UniformOutput',false);
        faces = unique(face(isSt));
        [~,o] = sort(cellfun(@(f) sscanf(f(1:end-1),'%d')*10 + double(f(end)=='B'),faces)); faces = faces(o);
        cs = {obj.cartons.state};
        txt = sprintf('WMS  t=%.0f s   boxes: %d on shelves | %d on stands | %d moving | %d at stations | %d outside%s',obj.t,sum(strcmp(cs,'STORED')),sum(strcmp(cs,'DOCK')),sum(strcmp(cs,'MOVING')),sum(strcmp(cs,'STATION')),sum(strcmp(cs,'RESERVE')),L);
        txt = [txt 'SHELVES  (P = slot, C = box in it, "-" = free slot)' L];
        for q = 1:numel(faces)
            idx = find(isSt & strcmp(face,faces{q})); line = ''; nOcc = 0; nFree = 0;
            for lv = 1:max([obj.slots(idx).level])
                part = '';
                for i = idx(:).'
                    if obj.slots(i).level ~= lv, continue, end
                    if strcmp(obj.slots(i).state,'OCCUPIED') && obj.slots(i).item > 0, nOcc = nOcc + 1; part = [part sprintf(' P%02d=C%d',obj.slots(i).pos,obj.slots(i).item)]; %#ok<AGROW>
                    elseif strcmp(obj.slots(i).state,'FREE'), nFree = nFree + 1; part = [part sprintf(' P%02d=-',obj.slots(i).pos)]; end %#ok<AGROW>
                end
                if ~isempty(part), line = [line sprintf('  L%d:%s',lv,part)]; end %#ok<AGROW>
            end
            txt = [txt sprintf('  %-3s %2d box / %2d free%s%s',faces{q},nOcc,nFree,line,L)]; %#ok<AGROW>
        end
        dk = find(strcmp({obj.slots.role},'dock')); part = ''; free = '';
        for i = dk(:).'
            if strcmp(obj.slots(i).state,'FREE'), free = [free ' ' obj.slots(i).id]; else, part = [part sprintf(' %s=C%d',obj.slots(i).id,obj.slots(i).item)]; end
        end
        txt = [txt 'STAGING STANDS (D area)' L '  boxes:' part L '  free :' free L];
        txt = [txt 'PACKING STATIONS' L];
        for q = 1:numel(obj.stations)
            part = ''; for z = 1:numel(obj.stations(q).state), if ~strcmp(obj.stations(q).state{z},'FREE'), part = [part sprintf(' slot%d=C%d',z,obj.stations(q).item(z))]; end, end
            if isempty(part), part = ' empty'; end
            txt = [txt sprintf('  %s:%s%s',obj.C.Stations(q).name,part,L)]; %#ok<AGROW>
        end
        mv = find(strcmp(cs,'MOVING')); if ~isempty(mv), txt = [txt sprintf('MOVING NOW: %s%s',strtrim(sprintf(' C%d',mv)),L)]; end
        rs = find(strcmp(cs,'RESERVE')); if ~isempty(rs), txt = [txt sprintf('OUTSIDE (spare, arrive with  p3 dock add): %s%s',strtrim(sprintf(' C%d',rs)),L)]; end
    end
    function j = boxOf(obj,spec)
        % "C12" or a slot name (shelf slot or stand) -> carton index, 0 if none (says why)
        j = 0; spec = char(spec);
        if ~isempty(regexp(spec,'^[cC]\d+$','once'))
            j = str2double(spec(2:end)); if j < 1 || j > obj.M, obj.say('[P3] no box %s (1..%d)\n',spec,obj.M); j = 0; end
        else
            si = obj.slotIndex(spec); if si == 0, obj.say('[P3] unknown slot or box "%s"  (p3 wms shows every box)\n',spec); return, end
            if ~strcmp(obj.slots(si).state,'OCCUPIED') || obj.slots(si).item < 1, obj.say('[P3] slot %s is %s - no box there\n',obj.slots(si).id,obj.slots(si).state); return, end
            j = obj.slots(si).item;
        end
    end
    function urgentJob(obj,spec,station)
        % emergency job: box (slot or C<id>) -> packing station.  Nearest FREE robot (empty hand, no task) takes it; if none is free the FIRST robot that
        % finishes its task takes it (the job has the highest priority).  Only the order of work changes - every robot still drives and plans as always.
        if obj.finishing, obj.say('[P3] mission is ending - no new jobs accepted\n'); return, end
        j = obj.boxOf(spec); if j == 0, return, end
        c = obj.cartons(j);
        if ~any(strcmp(c.state,{'STORED','DOCK'})), obj.say('[P3] C%d is %s - urgent job not possible now\n',j,c.state); return, end
        if c.task ~= 0, obj.say('[P3] C%d already has a task (task %d)\n',j,c.task); return, end
        src = c.slot; k = 0; best = inf;
        for q = 1:obj.N
            if obj.robotAvailable(q) && ~any(strcmp({obj.tasks.status},'QUEUED') & [obj.tasks.pinned] == q), d = norm(obj.robots(q).pose(1:2) - obj.slots(src).staging); if d < best, best = d; k = q; end, end
        end
        type = 'retrieve'; if strcmp(c.state,'DOCK'), type = 'crossdock'; end
        tid = obj.newTask(type,j,src,2,station,0,100,0,k);
        where = 'any station'; if station > 0, where = obj.stations(station).name; end
        if k > 0, who = sprintf('%s (nearest free robot, %.0f m away)',obj.robots(k).name,best); else, who = 'the first robot that finishes its task (all robots are busy)'; end
        obj.say('[P3] URGENT task %d: C%d from %s -> %s.  Taken by %s\n',tid,j,obj.slots(src).id,where,who);
        obj.ev('URGENT',k,sprintf('task %d urgent C%d %s',tid,j,obj.slots(src).id));
    end
    function moveJob(obj,spec,dest,station,k)
        % p3 move <slot|C<id>> station [n] | <free slot>   [robot <name>]    k = 0: any free robot (normal priority)
        j = obj.boxOf(spec); if j == 0, return, end
        c = obj.cartons(j);
        if ~any(strcmp(c.state,{'STORED','DOCK'})), obj.say('[P3] C%d is %s - cannot be moved now\n',j,c.state); return, end
        if c.task ~= 0, obj.say('[P3] C%d already has a task\n',j); return, end
        if strcmpi(dest,'station')
            type = 'retrieve'; if strcmp(c.state,'DOCK'), type = 'crossdock'; end
            tid = obj.newTask(type,j,c.slot,2,station,0,3,0,k); obj.holdIfBatch(tid);
            obj.say('[P3] task %d: C%d from %s -> %s (%s)%s\n',tid,j,obj.slots(c.slot).id,p3tern(station>0,obj.stations(max(station,1)).name,'any station'),p3tern(k>0,obj.robots(max(k,1)).name,'any free robot'),p3tern(strcmp(obj.mode,'batch'),'  (queued - starts with  p3 go)',''));
        else
            if ~strcmp(c.state,'DOCK'), obj.say('[P3] only a box on a staging stand (D area) can be put into a shelf slot; a shelf box goes to a station (p3 move <slot> station)\n'); return, end
            d = obj.slotIndex(dest); if d == 0, obj.say('[P3] unknown slot %s\n',dest); return, end
            if k > 0, obj.manualTask(k,'putaway',j,dest,0); else, obj.queuePutaway(j,dest); end
        end
    end
    function holdIfBatch(obj,tid)
        if strcmp(obj.mode,'batch'), obj.setTask(tid,'status','HELD'); end
    end
    function setMode(obj,m)
        obj.mode = m;
        switch m
            case 'manual', obj.autoPutaway = false; obj.setAuto(false); obj.say(['[P3] MODE MANUAL: you give every job, it starts at once, several jobs run in parallel in this one window.\n' ...
                '     p3 wms                              all boxes and where they are\n' ...
                '     p3 move 8B-L3-P06 station 1         box (slot or C<id>) -> packing station, any free robot\n' ...
                '     p3 move D03 1A-L1-P02               box on a stand -> shelf slot\n' ...
                '     p3 red retrieve 3A-L2-P04 station 1 the same for one named robot\n']);
            case 'batch',  obj.autoPutaway = false; obj.setAuto(false); obj.say(['[P3] MODE BATCH: single-robot tests. Give each robot its own jobs, nothing moves until  p3 go.\n' ...
                '     p3 blue retrieve 3A-L2-P04 station 1     p3 red putaway C5 1A-L1-P03     p3 queue (list)     p3 go [robot]\n']);
            case 'auto',   obj.autoPutaway = true;  obj.setAuto(true);  obj.say(['[P3] MODE AUTO: the system works by itself (puts boxes away, creates orders and deliveries).\n' ...
                '     Emergency job while it runs:  p3 urgent 8B-L3-P06 [station n]  (nearest free robot takes it, else the first that finishes)\n']);
        end
        if ~strcmp(m,'batch') && any(strcmp({obj.tasks.status},'HELD')), obj.say('     (there are still queued tasks - p3 go starts them)\n'); end
    end
    function go(obj,k)
        n = 0;
        for i = 1:numel(obj.tasks)
            if strcmp(obj.tasks(i).status,'HELD') && (k == 0 || obj.tasks(i).pinned == k), obj.tasks(i).status = 'QUEUED'; n = n + 1; end
        end
        obj.say('[P3] GO: %d queued task(s) released%s\n',n,p3tern(k>0,[' for ' obj.robots(max(k,1)).name],''));
    end
    function txt = queueText(obj)
        L = char(10); txt = ['Queued tasks (waiting for  p3 go):' L]; n = 0;
        for i = 1:numel(obj.tasks)
            T = obj.tasks(i); if ~strcmp(T.status,'HELD'), continue, end
            who = 'any robot'; if T.pinned > 0, who = obj.robots(T.pinned).name; end
            if T.dstKind == 1, to = p3tern(T.dstSlot>0,obj.slots(max(T.dstSlot,1)).id,'best free slot'); else, to = sprintf('Packing Station %d',T.dstStation); end
            txt = [txt sprintf('  task %-3d %-9s %-9s C%-2d -> %s%s',T.id,who,T.type,T.item,to,L)]; n = n + 1; %#ok<AGROW>
        end
        if n == 0, txt = [txt '  (none)' L]; end
    end
    function shipNow(obj,s)
        for q = 1:numel(obj.stations), if s == 0 || s == q, obj.shipReq(q) = true; end, end
        obj.say('[P3] boxes on %s will be shipped out now\n',p3tern(s>0,obj.stations(max(s,1)).name,'all stations'));
    end
    function addBoxAt(obj,spec,j)
        % p3 add <slot|D..> [C<id>]: a spare box appears in a FREE shelf slot or staging stand (test / refill).  Slot must be free (no robot is assigned to it).
        if nargin < 3, j = 0; end
        if obj.finishing, obj.say('[P3] mission is ending - no new boxes accepted\n'); return, end
        si = obj.slotIndex(spec); if si == 0, obj.say('[P3] unknown slot %s\n',char(spec)); return, end
        if ~strcmp(obj.slots(si).state,'FREE'), obj.say('[P3] slot %s is %s - not free\n',obj.slots(si).id,obj.slots(si).state); return, end
        if j == 0, for z = 1:obj.M, if strcmp(obj.cartons(z).state,'RESERVE'), j = z; break, end, end, end
        if j == 0, obj.say('[P3] no spare box left (p3 remove C<id> / p3 clear ... gives boxes back)\n'); return, end
        if ~strcmp(obj.cartons(j).state,'RESERVE'), obj.say('[P3] C%d is not a spare box (it is %s)\n',j,obj.cartons(j).state); return, end
        isDock = strcmp(obj.slots(si).role,'dock');
        obj.slots(si).state = 'OCCUPIED'; obj.slots(si).item = j;
        obj.cartons(j).state = p3tern(isDock,'DOCK','STORED'); obj.cartons(j).slot = si; obj.cartons(j).xyz = obj.slots(si).xyz; obj.cartons(j).cmd = obj.slots(si).xyz; obj.cartons(j).cmdYaw = obj.slots(si).boxYaw;
        obj.cartons(j).drive = true; obj.cartons(j).spawnUntil = obj.t + 1.5; obj.cartons(j).task = 0;
        obj.ev('ADD',0,sprintf('C%d put into %s',j,obj.slots(si).id)); obj.say('[P3] C%d is now in %s\n',j,obj.slots(si).id);
    end
    function addBoxStation(obj,s,j)
        % p3 add station <n> [C<id>]: a spare box appears on the next free place of packing station n
        if nargin < 3, j = 0; end
        if s < 1 || s > numel(obj.stations), obj.say('[P3] no such station\n'); return, end
        st = obj.stations(s); q = find(strcmp(st.state,'FREE'),1);
        if isempty(q), obj.say('[P3] %s is full\n',st.name); return, end
        if j == 0, for z = 1:obj.M, if strcmp(obj.cartons(z).state,'RESERVE'), j = z; break, end, end, end
        if j == 0 || ~strcmp(obj.cartons(j).state,'RESERVE'), obj.say('[P3] no spare box available\n'); return, end
        st.state{q} = 'OCC'; st.item(q) = j; st.since(q) = obj.t; st.by(q) = 0; obj.stations(s) = st;
        S = obj.C.Stations(s);
        obj.cartons(j).state = 'STATION'; obj.cartons(j).stationSlot = [s q]; obj.cartons(j).slot = 0; obj.cartons(j).xyz = S.slotXYZ(q,:); obj.cartons(j).cmd = S.slotXYZ(q,:); obj.cartons(j).cmdYaw = S.yaw;
        obj.cartons(j).drive = true; obj.cartons(j).spawnUntil = obj.t + 1.5; obj.cartons(j).task = 0;
        obj.ev('ADD',0,sprintf('C%d put on %s place %d',j,st.name,q)); obj.say('[P3] C%d is now on %s place %d\n',j,st.name,q);
    end
    function n = clearBoxes(obj,what,arg)
        % p3 clear shelves | stands | stations [n] | all : boxes without a task go back to the spare pool (outside)
        if nargin < 3, arg = 0; end
        what = lower(char(what)); n = 0; skipped = 0;
        for j = 1:obj.M
            c = obj.cartons(j); st = c.state;
            hit = false;
            switch what
                case 'shelves', hit = strcmp(st,'STORED');
                case 'stands',  hit = strcmp(st,'DOCK');
                case 'stations', hit = strcmp(st,'STATION') && (arg == 0 || c.stationSlot(1) == arg);
                case 'all', hit = any(strcmp(st,{'STORED','DOCK','STATION'}));
            end
            if ~hit, continue, end
            if c.task ~= 0, skipped = skipped + 1; continue, end
            obj.removeCarton(j,true); n = n + 1;
        end
        obj.say('[P3] %d box(es) removed (%s)%s\n',n,what,p3tern(skipped>0,sprintf(', %d skipped: a robot is working on them',skipped),''));
    end
    function ok = resetBoxes(obj)
        % p3 reset: every box back to its START place (30 on shelves, 10 on stands, the rest outside).  Only when no robot has a job.
        ok = false;
        for k = 1:obj.N, if obj.robots(k).task > 0, obj.say('[P3] reset only when no robot has a job (p3 status) - try again when they are free\n'); return, end, end
        for q = 1:numel(obj.tasks), if any(strcmp(obj.tasks(q).status,{'QUEUED','ACTIVE','RUNNING','HELD'})), obj.say('[P3] reset only when no task is open (p3 queue)\n'); return, end, end
        P3 = obj.C.P3;
        for i = 1:numel(obj.slots), if any(strcmp(obj.slots(i).state,{'OCCUPIED','RES_IN','RES_OUT'})), obj.slots(i).state = 'FREE'; obj.slots(i).item = 0; obj.slots(i).by = 0; obj.slots(i).task = 0; end, end
        for q = 1:numel(obj.stations), st = obj.stations(q); st.state(:) = {'FREE'}; st.item(:) = 0; st.by(:) = 0; obj.stations(q) = st; end
        for j = 1:obj.M
            c0 = P3.cartons(j); obj.cartons(j).task = 0; obj.cartons(j).stationSlot = [0 0]; obj.cartons(j).state = c0.state; obj.cartons(j).slot = c0.slot; obj.cartons(j).xyz = c0.xyz;
            obj.cartons(j).drive = true; obj.cartons(j).spawnUntil = obj.t + 1.5; obj.cartons(j).cmdYaw = c0.yaw;
            if strcmp(c0.state,'RESERVE'), obj.cartons(j).cmd = c0.park; else, obj.cartons(j).cmd = c0.xyz; end
            if c0.slot > 0, obj.slots(c0.slot).state = 'OCCUPIED'; obj.slots(c0.slot).item = j; end
        end
        obj.orders = Phase3Fleet.emptyOrders(); obj.locks = cell(0,2); ok = true;
        obj.ev('RESET',0,'boxes back to their start places'); obj.say('[P3] all boxes are back at their START places (shelves + stands as at the beginning, stations empty)\n');
    end
    function removeCarton(obj,j,quiet)
        if nargin < 3, quiet = false; end
        % take a carton out of the warehouse (it goes under the floor and is a spare again)
        if j < 1 || j > obj.M, obj.say('[P3] no such carton\n'); return, end
        c = obj.cartons(j); if c.task ~= 0, obj.say('[P3] C%d has a task - cannot remove now\n',j); return, end
        if any(strcmp(c.state,{'STORED','DOCK'})) && c.slot > 0, obj.slots(c.slot).state = 'FREE'; obj.slots(c.slot).item = 0; end
        if strcmp(c.state,'STATION') && c.stationSlot(1) > 0, st = obj.stations(c.stationSlot(1)); st.state{c.stationSlot(2)} = 'FREE'; st.item(c.stationSlot(2)) = 0; obj.stations(c.stationSlot(1)) = st; end
        obj.cartons(j).state = 'RESERVE'; obj.cartons(j).slot = 0; obj.cartons(j).stationSlot = [0 0]; obj.cartons(j).drive = true; obj.cartons(j).cmd = c.park; obj.cartons(j).cmdYaw = 0;
        if ~quiet, obj.say('[P3] C%d removed from the warehouse (now a spare box)\n',j); end
    end
    function txt = cartonList(obj,all)
        if nargin < 2, all = false; end
        L = char(10); st = {obj.cartons.state};
        txt = sprintf('Cartons: %d total | %d stored on shelves | %d on stands | %d moving | %d at stations | %d spare%s',obj.M,sum(strcmp(st,'STORED')),sum(strcmp(st,'DOCK')),sum(strcmp(st,'MOVING')),sum(strcmp(st,'STATION')),sum(strcmp(st,'RESERVE')),L);
        for j = 1:obj.M
            c = obj.cartons(j); where = '';
            if strcmp(c.state,'STORED') && ~all, continue, end
            if strcmp(c.state,'RESERVE') && ~all, continue, end
            switch c.state
                case {'STORED','DOCK'}, where = obj.slots(c.slot).id;
                case 'STATION', where = sprintf('%s slot %d',obj.stations(c.stationSlot(1)).name,c.stationSlot(2));
                case 'RESERVE', where = 'spare (arrives with the next delivery or p3 dock add)';
            end
            txt = [txt sprintf('  C%-2d  %-8s %-34s xyz=[%.1f %.1f %.2f]%s',j,c.state,where,c.cmd(1),c.cmd(2),c.cmd(3),L)]; %#ok<AGROW>
        end
    end
    function askAlternative(obj,k,item,d)
        rn = 'fleet'; if k > 0, rn = obj.robots(k).name; end
        s = obj.slots(d);
        free = obj.freeSlotsNear(obj.cartons(item).slot,5);
        if strcmp(obj.answerMode,'auto') && ~isempty(free)
            obj.say('[P3] %s: slot %s is %s -> auto-choosing %s\n',rn,s.id,s.state,obj.slots(free(1)).id);
            obj.newTask('putaway',item,obj.cartons(item).slot,1,0,free(1),3,0,k); return
        end
        qid = numel(obj.questions) + 1;
        txt = sprintf('%s: slot %s is %s. Another shelf for C%d?  free nearby: %s',rn,s.id,s.state,item,strjoin(arrayfun(@(i) obj.slots(i).id,free,'UniformOutput',false),'  '));
        obj.questions(qid) = struct('id',qid,'robot',k,'task',item,'slot',d,'text',txt);
        obj.say('[ASK] %s\n      answer with   p3 reply <slot id>     or   p3 reply auto     (e.g.  p3 reply %s)\n',txt,p3tern(~isempty(free),obj.slots(free(1)).id,'3A-L1-P04'));
    end
    function reply(obj,arg)
        if isempty(obj.questions), obj.say('[P3] no open question\n'); return, end
        q = obj.questions(1); obj.questions(1) = []; item = q.task; k = q.robot;
        if strcmpi(arg,'auto')
            free = obj.freeSlotsNear(obj.cartons(item).slot,1); if isempty(free), obj.say('[P3] no free slot at all\n'); return, end, d = free(1);
        else
            d = obj.slotIndex(arg);
            if d == 0 || ~strcmp(obj.slots(d).state,'FREE') || ~strcmp(obj.slots(d).role,'store')
                obj.say('[P3] %s is not a free shelf slot - question stays open\n',arg); obj.questions = [q obj.questions]; return
            end
        end
        obj.newTask('putaway',item,obj.cartons(item).slot,1,0,d,3,0,k);
        rn = 'fleet'; if k > 0, rn = obj.robots(k).name; end
        obj.say('[P3] %s: put away C%d into %s\n',rn,item,obj.slots(d).id);
    end
    function idx = freeSlotsNear(obj,srcIdx,n)
        c = []; d = [];
        for i = 1:numel(obj.slots)
            if strcmp(obj.slots(i).role,'store') && strcmp(obj.slots(i).state,'FREE'), c(end+1) = i; d(end+1) = norm(obj.slots(i).dock - obj.slots(srcIdx).dock) + 4*(obj.slots(i).level-1); end %#ok<AGROW>
        end
        [~,o] = sort(d); c = c(o); idx = c(1:min(n,numel(c)));
    end
    function si = slotIndex(obj,name)
        si = 0;
        if isnumeric(name), if name >= 1 && name <= numel(obj.slots), si = name; end, return, end
        name = upper(strtrim(char(name)));
        t1 = regexp(name,'^(\d+)([AB])[-_ ]?L(\d)[-_ ]?P(\d+)$','tokens','once');
        if ~isempty(t1), name = sprintf('%s%s-L%s-P%02d',t1{1},t1{2},t1{3},str2double(t1{4})); end
        t2 = regexp(name,'^D(\d+)$','tokens','once'); if ~isempty(t2), name = sprintf('D%02d',str2double(t2{1})); end
        for i = 1:numel(obj.slots), if strcmp(obj.slots(i).id,name), si = i; return, end, end
        tok = regexp(name,'^#?(\d+)$','tokens','once'); if ~isempty(tok), si = obj.slotIndex(str2double(tok{1})); end
    end
    function k = robotIndex(obj,name)
        k = 0; name = lower(strtrim(char(name)));
        for i = 1:obj.N, if strcmp(obj.robots(i).name,name), k = i; return, end, end
        v = str2double(name); if isfinite(v) && v >= 1 && v <= obj.N && v == round(v), k = v; end
    end
    function setSlot(obj,name,state)
        si = obj.slotIndex(name); if si == 0, obj.say('[P3] unknown slot %s\n',name); return, end
        if strcmp(state,'free')
            if obj.slots(si).item > 0, obj.say('[P3] %s holds C%d - cannot clear\n',obj.slots(si).id,obj.slots(si).item); return, end
            obj.slots(si).state = 'FREE';
        elseif strcmp(state,'blocked'), obj.slots(si).state = 'UNKNOWN'; end
        obj.say('[P3] slot %s is now %s\n',obj.slots(si).id,obj.slots(si).state);
    end
    function robotCmd(obj,k,what)
        r = obj.robots(k);
        switch what
            case 'pause',  obj.robots(k).paused = true;  obj.say('[P3] %s: no new tasks (finishes the current one)\n',r.name);
            case 'resume', obj.robots(k).paused = false; obj.robots(k).stopNow = false; obj.say('[P3] %s: resumed\n',r.name);
            case 'stop',   obj.robots(k).stopNow = true;  obj.say('[P3] %s: STOP (holds position; "p3 %s resume" releases)\n',r.name,r.name);
            case 'status', obj.say('%s\n',obj.robotLine(k));
        end
    end
    function setWatch(obj,k)
        obj.watch = k; if k > 0, obj.camSel = k; obj.say('[P3] camera window now shows robot %s\n',obj.robots(k).name); else, obj.say('[P3] camera follows the scanning robots only\n'); end
    end
    function setAuto(obj,on,period,arrive,station)
        obj.autoOn = on;
        if on
            if nargin > 2 && ~isempty(period), obj.autoPeriod = period; end
            if nargin > 3 && ~isempty(arrive), obj.autoArrive = arrive; end
            if nargin > 4 && ~isempty(station), obj.autoStation = station; end
            obj.autoNext = obj.t + 5; obj.autoArriveNext = obj.t + 10; obj.keepStations = true; obj.autoBase = obj.nRetAuto + obj.nPutAuto; obj.autoFinished = false;
            obj.say('[P3] AUTO mode on: putaway and retrieve 50:50, a retrieve order about every %.0f s, new delivery of boxes only when every staging stand is empty\n',obj.autoPeriod);
        else
            obj.autoNext = inf; obj.autoArriveNext = inf; obj.say('[P3] AUTO mode off\n');
        end
    end
    function finishAll(obj)
        obj.go(0); obj.finishing = true; obj.autoOn = false; obj.say('[P3] END: queued tasks are finished, then all robots go home and park\n');
    end
    function stopNow(obj), obj.stopFlag = true; obj.say('[P3] stopping the simulation\n'); obj.metricsWrite(true); end

    % ------------------------------------------------------------------ text views and files
    function s = robotLine(obj,k)
        r = obj.robots(k); names = {'IDLE','NAV-SHELF','ARM-EXT','GRASP','RETRACT','NAV-DEST','PLACE','HOME','RELEASE','STOW','DOCKING','FAULT','DONE'};
        st = '?'; if r.rep, st = names{min(13,r.state+1)}; end
        job = '-';
        if r.task > 0 && ~isempty(r.job), job = sprintf('t%d %s C%d %s -> %s',r.task,r.job.type,r.job.carton,r.job.src.name,r.job.dst.name); end
        flag = ''; if r.paused, flag = [flag ' paused']; end; if r.fault, flag = [flag ' FAULT']; end; if r.stopNow, flag = [flag ' STOP']; end
        if r.pendHold || r.lockHold, flag = [flag ' waits-for-zone']; elseif r.scale < 0.3, flag = [flag sprintf(' yields-to-%s',obj.robots(max(1,r.blockedBy)).name)]; end
        s = sprintf('  %-6s %-9s (%5.1f,%5.1f)  %s%s',r.name,st,r.pose(1),r.pose(2),job,flag);
    end
    function statusText(obj)
        obj.say('---- t=%.0f s ---- orders waiting %d, tasks queued %d, shipped %d\n',obj.t,sum(strcmp({obj.orders.status},'WAIT')),sum(strcmp({obj.tasks.status},'QUEUED')),obj.shipped);
        for k = 1:obj.N, obj.say('%s\n',obj.robotLine(k)); end
        c = {obj.cartons.state}; obj.say('  cartons: yard %d | dock %d | shelf %d | moving %d | station %d | lost %d\n', ...
            sum(strcmp(c,'RESERVE')),sum(strcmp(c,'DOCK')),sum(strcmp(c,'STORED')),sum(strcmp(c,'MOVING')),sum(strcmp(c,'STATION')),sum(strcmp(c,'LOST')));
        if ~isempty(obj.questions), obj.say('  OPEN QUESTION: %s\n',obj.questions(1).text); end
    end
    function f = snapshot(obj,tag)
        if nargin < 2, tag = ''; end
        stamp = datestr(now,'HHMMSS'); base = fullfile(obj.dirs.slots,sprintf('Slots_%s_t%04d%s_%s',obj.stamp,round(obj.t),tag,stamp));
        f = [base '.csv']; fid = fopen(f,'w');
        fprintf(fid,'num,slot,shelf,side,level,pos,role,state,carton,robot,x,y,z\n');
        for i = 1:numel(obj.slots)
            s = obj.slots(i); rb = ''; if s.by > 0, rb = obj.robots(s.by).name; end
            it = ''; if s.item > 0, it = sprintf('C%d',s.item); end
            fprintf(fid,'%d,%s,%d,%s,%d,%d,%s,%s,%s,%s,%.3f,%.3f,%.3f\n',s.num,s.id,s.shelf,s.side,s.level,s.pos,s.role,s.state,it,rb,s.xyz);
        end
        for si = 1:numel(obj.stations)
            for q = 1:numel(obj.stations(si).state)
                st = obj.stations(si); it = ''; if st.item(q) > 0, it = sprintf('C%d',st.item(q)); end
                fprintf(fid,'ST%d-%d,%s slot %d,0,-,0,%d,station,%s,%s,,,,\n',si,q,st.name,q,q,st.state{q},it);
            end
        end
        fclose(fid);
        g = [base '.txt']; fid = fopen(g,'w'); fprintf(fid,'%s',obj.slotMap()); fclose(fid);
        s = {obj.slots.state}; role = {obj.slots.role}; stor = strcmp(role,'store');
        obj.say('[P3] slot snapshot saved:\n     %s\n     %s\n     shelf slots: %d free, %d occupied, %d reserved, %d static stock, %d unknown | dock: %d of %d occupied\n', ...
            f,g,sum(stor & strcmp(s,'FREE')),sum(stor & strcmp(s,'OCCUPIED')),sum(stor & (strcmp(s,'RES_IN') | strcmp(s,'RES_OUT'))),sum(stor & strcmp(s,'STATIC')),sum(stor & strcmp(s,'UNKNOWN')), ...
            sum(strcmp(role,'dock') & strcmp(s,'OCCUPIED')),sum(strcmp(role,'dock')));
    end
    function txt = slotMap(obj)
        L = char(10); txt = sprintf('Slot map at t=%.0f s   (shelf 3A = shelf 3 side A (west face), 3B = side B (east face), D = staging stands)   (. free   C07 carton   ## static stock   >> reserved for incoming   << reserved outgoing   ?? unknown)%s',obj.t,L);
        groups = unique([obj.slots.grp]);
        for gp = groups
            ig = find([obj.slots.grp] == gp); gn = obj.slots(ig(1)).gname;
            if gp == 100, txt = [txt sprintf('%sStaging stands (receiving area)',L)]; else, txt = [txt sprintf('%sShelf %s',L,gn)]; end %#ok<AGROW>
            for lv = 3:-1:1
                ix = find([obj.slots.grp] == gp & [obj.slots.level] == lv); if isempty(ix), continue, end
                [~,o] = sort([obj.slots(ix).pos]); ix = ix(o); row = sprintf('%s  %s |',L,p3tern(gp==100,'D ',sprintf('L%d',lv)));
                for i = ix
                    s = obj.slots(i); switch s.state
                        case 'FREE', c = '  . '; case 'OCCUPIED', c = sprintf('C%02d ',s.item); case 'STATIC', c = ' ## ';
                        case 'RES_IN', c = ' >> '; case 'RES_OUT', c = ' << '; otherwise, c = ' ?? '; end
                    row = [row sprintf('%s',c)]; %#ok<AGROW>
                end
                txt = [txt row]; %#ok<AGROW>
            end
            txt = [txt sprintf('%s       positions: %s',L,strjoin(arrayfun(@(i) sprintf('P%02d',obj.slots(i).pos),ix,'UniformOutput',false),' '))]; %#ok<AGROW>
        end
        txt = [txt sprintf('%s%sStations%s',L,L,L)];
        for si = 1:numel(obj.stations), st = obj.stations(si); txt = [txt sprintf('  %s: %s%s',st.name,strjoin(st.state,' | '),L)]; end %#ok<AGROW>
        txt = [txt sprintf('%sCartons: %s%s',L,strjoin(arrayfun(@(c) sprintf('C%d=%s',c.id,c.state),obj.cartons,'UniformOutput',false),' '),L)];
    end
    function [E,parts] = energyOf(obj,k)
        % ENERGY from the simulated motion (Wh).  drive: wheel power from the real speed / acceleration / turning of the robot, mass = base + Kinova Gen3 (8.2 kg, datasheet) + carried carton
        % (4 kg, the mass set in the 3-D scene), rolling friction crr, drive efficiency;  lift: (arm + carton) g h / eta for every pick, h = shelf level of the slot;
        % arm: 36 W (Kinova Gen3 datasheet, average power) while the arm moves;  base: pBase W for the whole run (computer, lidar, camera, controllers).
        % ASSUMED (no datasheet): massBase, crr, etaDrive, etaLift, pBase - properties of Phase3Fleet.  Returns Wh and parts [drive lift arm base].
        st = obj.robots(k).stats; T = max(obj.t,st.busy + st.idle);
        parts = [st.eDrive, st.eLift, obj.pArm*st.tArm, obj.pBase*T]/3600; E = sum(parts);
    end
    function u = utilOf(obj,k)
        % utilisation of robot k over the WORKING period of the run (first assignment .. last delivery / now), not over the whole session: waiting for the operator before the first
        % command or after the last one is not idle time of the fleet.  busy = time between assignment and delivery of its tasks.
        tk = obj.tasks; ok = false(1,numel(tk)); for i = 1:numel(tk), ok(i) = tk(i).assigned > 0; end
        if ~any(ok), u = 0; return, end
        t0 = min([tk(ok).assigned]); t1 = t0; busy = 0;
        for i = find(ok)
            te = tk(i).done; if te <= 0, te = obj.t; end
            t1 = max(t1,te); if tk(i).robot == k, busy = busy + max(0,te - tk(i).assigned); end
        end
        u = 100*min(1,busy/max(1,t1 - t0));
    end
    function metricsWrite(obj,final)
        if nargin < 2, final = false; end
        f = fullfile(obj.dirs.metrics,sprintf('Metrics_%s.csv',obj.stamp)); fid = fopen(f,'w');
        fprintf(fid,'robot,tasks_done,distance_m,busy_s,idle_s,wait_s,utilisation_pct,mean_assign_wait_s,energy_Wh,energy_drive_Wh,energy_lift_Wh,energy_arm_Wh,energy_base_Wh\n');
        for k = 1:obj.N
            r = obj.robots(k); u = obj.utilOf(k);
            [E_,pt_] = obj.energyOf(k); fprintf(fid,'%s,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f\n',r.name,r.nDone,r.stats.dist,r.stats.busy,r.stats.idle,r.stats.wait,u,r.stats.assignWait/max(1,r.nDone),E_,pt_(1),pt_(2),pt_(3),pt_(4));
        end
        fclose(fid);
        f2 = fullfile(obj.dirs.tasks,sprintf('Tasks_%s.csv',obj.stamp)); fid = fopen(f2,'w');
        fprintf(fid,'task,type,carton,robot,status,created_s,assigned_s,done_s,queue_wait_s,cycle_s,note\n');
        for i = 1:numel(obj.tasks)
            T = obj.tasks(i); rb = ''; if T.robot > 0, rb = obj.robots(T.robot).name; end
            fprintf(fid,'%d,%s,C%d,%s,%s,%.1f,%.1f,%.1f,%.1f,%.1f,%s\n',T.id,T.type,T.item,rb,T.status,T.created,T.assigned,T.done,max(0,T.assigned-T.created),max(0,T.done-T.created),T.note);
        end
        fclose(fid);
        f3 = fullfile(obj.dirs.events,sprintf('Events_%s.csv',obj.stamp)); fid = fopen(f3,'w'); fprintf(fid,'t_s,robot,event\n');
        for i = 1:size(obj.evlog,1), fprintf(fid,'%.1f,%s,"%s"\n',obj.evlog{i,1},obj.evlog{i,2},strrep(obj.evlog{i,3},'"','''')); end
        fclose(fid);
        done = obj.tasks(strcmp({obj.tasks.status},'DONE')); T = max(obj.t,1);
        f4 = fullfile(obj.dirs.summary,sprintf('Summary_%s.txt',obj.stamp)); fid = fopen(f4,'w');
        fprintf(fid,'Phase 2 run summary (%s)\nrobots %d, simulated time %.0f s\n',obj.stamp,obj.N,obj.t);
        fprintf(fid,'tasks done %d (putaway %d, retrieve %d, cross-dock %d), failed %d\n',numel(done),sum(strcmp({done.type},'putaway')),sum(strcmp({done.type},'retrieve')),sum(strcmp({done.type},'crossdock')),sum(strcmp({obj.tasks.status},'FAILED')));
        fprintf(fid,'throughput %.2f tasks/min, shipped cartons %d\n',60*numel(done)/T,obj.shipped);
        if ~isempty(done), fprintf(fid,'mean task cycle (created->done) %.1f s, mean queue wait %.1f s\n',mean([done.done]-[done.created]),mean([done.assigned]-[done.created])); end
        % ---- efficiency / suitability of the fleet for this workload
        ut = zeros(1,obj.N); wt = zeros(1,obj.N); ds = zeros(1,obj.N); for k = 1:obj.N, st_ = obj.robots(k).stats; ut(k) = obj.utilOf(k); wt(k) = 100*st_.wait/max(1,st_.busy); ds(k) = st_.dist; end
        fprintf(fid,'EFFICIENCY: throughput %.1f tasks/h (%.2f per robot and hour), mean robot utilisation %.0f %% (min %.0f %%, max %.0f %%), time held back by other robots %.1f %% of the working time, distance %.0f m (%.1f m per task)\n', ...
            3600*numel(done)/T,3600*numel(done)/T/obj.N,mean(ut),min(ut),max(ut),mean(wt),sum(ds),sum(ds)/max(1,numel(done)));
        eT_ = 0; eP_ = zeros(1,4); for k = 1:obj.N, [e_,p_] = obj.energyOf(k); eT_ = eT_ + e_; eP_ = eP_ + p_; end
        fprintf(fid,'ENERGY (simulated motion + datasheet: 36 W arm, %.1f kg arm, %.0f kg carton; assumed: %.0f kg base, %.0f W base electronics): total %.1f Wh = driving %.1f + lifting %.2f + arm %.1f + base %.1f Wh;  %.2f Wh per task, %.3f Wh per metre driven\n', ...
            obj.massArm,obj.massBox,obj.massBase,obj.pBase,eT_,eP_(1),eP_(2),eP_(3),eP_(4),eT_/max(1,numel(done)),eT_/max(1,sum(ds)));
        for ty_ = {'putaway','retrieve','crossdock'}
            dd_ = done(strcmp({done.type},ty_{1}));
            if ~isempty(dd_), fprintf(fid,'            %s: %d done, mean service time (assigned->done) %.1f s, mean queue wait %.1f s\n',ty_{1},numel(dd_),mean([dd_.done]-[dd_.assigned]),mean([dd_.assigned]-[dd_.created])); end
        end
        nF = size(obj.faultTries,1); nS = sum([obj.robots.stallN]);
        fprintf(fid,'            faults handled %d (tasks affected), watchdog interventions %d, tasks given up %d\n',nF,nS,sum(strcmp({obj.tasks.status},'FAILED')));
        if mean(ut) < 55, fprintf(fid,'            SUITABILITY: the robots are only %.0f %% busy on average - for this workload fewer robots would do the job (less congestion, same throughput).\n',mean(ut));
        elseif mean(ut) > 90, fprintf(fid,'            SUITABILITY: the robots are busy %.0f %% of the time - the fleet is the bottleneck, more robots (or more packing / staging capacity) would raise the throughput.\n',mean(ut));
        else, fprintf(fid,'            SUITABILITY: utilisation %.0f %% is in the 55-90 %% band - the fleet size fits this workload.\n',mean(ut)); end
        fprintf(fid,'min robot-robot distance (centres minus carried-box / arm reach) %.2f m, near misses (<%.2f m) %d, collisions (<%.2f m = chassis circles / boxes / arms touch) %d, wait cycles %d, replans %d\n',obj.minSep,obj.TP.dHard,obj.nearMiss,obj.TP.body,obj.collisions,obj.deadlocks,obj.replans);
        qs_ = zeros(1,obj.N); for k = 1:obj.N, qs_(k) = obj.robots(k).stats.queue; end
        fprintf(fid,'zone queue: robots waited for a free shelf face / station place %.0f s in total (%.1f %% of the working time) - they stood still at a safe distance, empty-handed, and went as soon as a place was free; %d waiting points used\n',sum(qs_),100*sum(qs_)/max(1,sum(arrayfun(@(q) q.stats.busy,obj.robots))),sum(cellfun(@(s) strncmp(s,'WAIT: ',6),obj.evlog(:,3))));
        fprintf(fid,'camera verification: pass %d, fail %d, unavailable %d\n',obj.visionCount(1),obj.visionCount(2),obj.visionCount(4));
        ms_ = obj.mstat; fprintf(fid,'multi-robot planning (M*): %d searches, %d coupled, %d solved by the timed level (waiting in the plan), %d without result, %d pull-overs (face to face), collisions %d, near misses %d\n',ms_.runs,ms_.coupled,ms_.timed,ms_.fail,obj.pullOvers,obj.collisions,obj.nearMiss);
        fclose(fid);
        obj.say('[P3] metrics written:\n     %s\n     %s\n     %s\n     %s\n',f,f2,f3,f4);
        if final
            fid = fopen(f4,'r'); txt = fread(fid,'*char').'; fclose(fid); obj.say('%s',txt);
        end
    end

    % ------------------------------------------------------------------ carton commands for the Simulink block
    function M4 = cartonCmd(obj)
        M4 = zeros(obj.M,5);
        for j = 1:obj.M, M4(j,:) = [obj.cartons(j).cmd double(obj.cartons(j).drive) obj.cartons(j).cmdYaw]; end
    end

    function ev(obj,type,k,msg)
        who = ''; if k > 0, who = obj.robots(k).name; end
        obj.evlog(end+1,:) = {obj.t,who,[type ': ' msg]};
    end
    function say(obj,varargin)
        fprintf(varargin{:});
    end
end
methods (Static)
    function r = newRobot(id,name,colour,home)
        r = struct('id',id,'name',name,'colour',colour,'home',home,'pose',[home 0],'state',0,'carry',false,'wp',zeros(0,2),'goal',home,'jobSeq',0,'jobStatus','idle', ...
            'rep',false,'repT',0,'task',0,'job',[],'jobSeqSent',0,'paused',false,'fault',false,'faultFlag',false,'finish',false,'stopNow',false, ...
            'scale',1,'dstUpdSeq',0,'dstUpd',[],'lockHold',false,'hold',false,'holdT',0,'headT',0,'planReq',false,'planSeq',0,'planNo',1e6,'planReqT',0,'escUntil',-1,'replanSeq',0,'replanT',-inf,'boost',0,'srcFreed',false,'dstLockOk',false,'srcLockOk',false,'pendHold',false,'holdPt',[],'holdTried',false,'evIssued',-inf,'srcKeys',{{}},'dstKeys',{{}},'srcPt',[0 0],'dstPt',[0 0],'evade',[NaN NaN 0],'evadeT',-inf, ...
            'lockWait',0,'scanReq',false,'vis','none','visFrames',0,'visHits',0,'visUnk',0,'visWait',0,'blockedBy',0,'override',-1,'nDone',0,'lastPose',[home 0], 'recoverSeq',0,'recoverDest',[],'recovering',false,'faultT',0,'prevPt',[0 0],'stateSeen',-1,'stateT',0,'waitEta',0,'abortSeq',0,'mroute',struct('seq',0,'wp',zeros(0,2)),'mpushT',-inf,'posProg',[0 0],'posProgT',0,'stallN',0, ...
            'stats',struct('dist',0,'busy',0,'idle',0,'wait',0,'assignWait',0,'queue',0,'distCarry',0,'tArm',0,'eDrive',0,'eLift',0,'vf',0,'wf',0));
    end
    function T = emptyTasks()
        T = Phase3Fleet.blankTask(0,'',0,0,1,0,0,0,0,0,0); T(1) = []; T = T(1:0);
    end
    function T = blankTask(id,type,item,src,dstKind,station,dstSlot,prio,order,pinned,t)
        if dstKind == 2, ds = station; dsl = dstSlot; else, ds = 0; dsl = dstSlot; end
        T = struct('id',id,'type',type,'item',item,'src',src,'dstKind',dstKind,'dstStation',ds,'dstSlot',dsl,'prio',prio,'order',order,'pinned',pinned, ...
            'robot',0,'status','QUEUED','created',t,'assigned',0,'started',0,'done',0,'note','');
    end
    function O = emptyOrders()
        O = Phase3Fleet.blankOrder(0,0,0,0); O = O(1:0);
    end
    function O = blankOrder(id,item,station,t)
        O = struct('id',id,'item',item,'station',station,'created',t,'status','WAIT','task',0);
    end
    function TP = localTrafficParams(C)
        % ALL traffic constants live here and follow from the robot radius, so the same logic works for any robot size / environment.
        R = 0.45; if isfield(C,'SafetyRadius'), R = C.SafetyRadius; end     % radius of the circle that contains the real chassis (+ column), measured from the 3-D pictures
        TP.body = 2*R;                 % two robots touch when their centres are this far apart
        TP.dHard = TP.body + 0.27;     % hard safety bubble: never drive closer than this towards another robot
        TP.dSlow = TP.body + 0.65;     % slow-down band
        TP.rc = TP.body + 0.47;        % passing distance between two moving robots
        TP.rcStill = TP.body + 0.30;   % passing distance to a robot that stands still
        TP.stampPos = R + 0.85;        % radius stamped around a robot when a route is planned around it
        TP.stampRoute = R + 0.45;      % radius stamped around the route points of other robots
        TP.carryExtra = 0.60; TP.armExtra = 0.50;      % a carried box / an arm that is out reaches this much beyond the chassis circle (measured from the 3-D pictures: carried box about 1 m from the robot centre)
        try, TP.carryExtra = C.CarryReach; TP.armExtra = C.ArmReach; catch, end
        TP.stowExtra = 0.35; try, TP.stowExtra = C.StowReach; catch, end      % a PARKED / waiting robot is passed this much wider (stowed arm, mast, path-following error): the bay area is where robots park
        TP.vFree = 0.78; TP.vCarry = 0.60; try, TP.vFree = C.NavSpeed; TP.vCarry = C.NavSpeedCarry; catch, end   % nominal speeds [m/s] used to predict WHEN each robot is where
        TP.plan = struct('Rc',TP.rc,'carryExtra',TP.carryExtra,'armExtra',TP.armExtra,'tWin',3.0,'maxStretch',1.25,'maxExtra',1.5,'maxTries',3,'blockRad',TP.body + 0.35);   % plan-time negotiation (fleetPlanRoute)
        TP.leaveDist = 2.6;            % a bay / stand / station lock is released only when the robot is this far away from it
        TP.look = 6.0;                 % look-ahead length of the route that is shared between robots [m]
        TP.hold1 = 2.0; TP.holdMoving = 5.0; TP.headOnT = 1.5;                % s a robot waits before it plans a way around
        TP.replanGap = 8.0;            % s minimum between two re-plans of the same robot (no flip-flop)
    end
    function res = keyRes(keys)
        % resource a key belongs to: a shelf face, a station table, or the key itself (used for first-come ordering of waiting robots)
        res = cell(1,numel(keys));
        for q = 1:numel(keys)
            k = keys{q}; c = find(k == ':'); if (strncmp(k,'fb:',3) || strncmp(k,'st:',3)) && numel(c) >= 2, res{q} = k(1:c(2)-1); else, res{q} = k; end
        end
    end
    function ix = nextIx(a), ix = numel(a) + 1; end
    function ix = tix(a,id), ix = find([a.id] == id,1); end
    function c = coarse(f), c = f(1:2:end,1:2:end) & f(2:2:end,1:2:end) & f(1:2:end,2:2:end) & f(2:2:end,2:2:end); end
    function M = discMask(g,c,rr)
        % logical ny x nx: coarse cells whose centre is within rr of c
        [ii,jj] = ndgrid(1:g.ny,1:g.nx); M = (((jj - 0.5)*g.res - g.x0 - c(1)).^2 + ((ii - 0.5)*g.res - c(2)).^2) <= rr^2;
    end
    function ok = losFree(Fq,g,p,q)
        d = norm(q - p); n = max(2,ceil(d/(0.4*g.res))); ok = true;
        for u = 0:n
            s0 = p + (q - p)*u/n; nv = [-(q(2) - p(2)) q(1) - p(1)]; if norm(nv) > 1e-9, nv = 0.2*nv/norm(nv); end
            for sg = [0 1 -1]      % the straight line must stay a margin (0.2 m) clear of the blocked cells on both sides: a cell is only known to be free at its centre, a line through its corner could graze a shelf
                s = s0 + sg*nv; j = min(g.nx,max(1,floor((s(1) + g.x0)/g.res) + 1)); i = min(g.ny,max(1,floor(s(2)/g.res) + 1));
                if ~Fq(i,j), ok = false; return, end
            end
        end
    end
    function d = polyDev(a,b)
        % largest distance of the points of polyline a (every 0.5 m) from polyline b
        d = 0; L = pathLength(a); n = max(2,ceil(L/0.5)); s = [0; cumsum(sqrt(sum(diff(a,1,1).^2,2)))];
        for u = linspace(0,L,n)
            p = [interp1(s,a(:,1),u) interp1(s,a(:,2),u)]; dm = inf;
            for k = 1:size(b,1)-1
                e = b(k+1,:) - b(k,:); t = max(0,min(1,dot(p - b(k,:),e)/max(dot(e,e),1e-9))); dm = min(dm,norm(p - (b(k,:) + t*e)));
            end
            d = max(d,dm);
        end
    end
    function pts = lookahead(r,L)
        pts = r.pose(1:2); if ~r.rep, return, end
        route = r.wp; if ~isempty(route) && any(r.state == [1 5 7]), route = [r.pose(1:2); route]; else, route = r.pose(1:2); end
        pts = []; acc = 0; step = 0.4; pts = route(1,:);
        for i = 1:size(route,1)-1
            a = route(i,:); b = route(i+1,:); d = norm(b-a); n = max(1,ceil(d/step));
            for q = 1:n
                pts(end+1,:) = a + (b-a)*q/n; %#ok<AGROW>
                acc = acc + d/n; if acc >= L, return, end
            end
        end
    end
    function sc = conflictArc(pi_,pj,Pj,moving,Rc,dHard)
        % arc length along i's look-ahead at which i would come too close to j (inf = no conflict).  A path that moves AWAY from j is no conflict:
        % two robots that are already close but driving apart must not freeze each other.
        sc = inf; si = 0; d0 = norm(pi_(1,:) - Pj);
        for a = 1:size(pi_,1)
            if a > 1, si = si + norm(pi_(a,:) - pi_(a-1,:)); end
            d = norm(pi_(a,:) - Pj);
            if d < Rc && d < d0 - 0.05, sc = si; return, end
            if moving && a <= size(pj,1)
                dm = norm(pi_(a,:) - pj(a,:));
                if dm < Rc && dm < d0 - 0.05, sc = si; return, end
            end
        end
    end
    function [f,headOn] = conflictTime(pi_,vi,pj,vj,Rc,dHard)
        % Space-time check of two shared routes.  f = speed factor for robot i (1 = no conflict): slow down so that i reaches the first shared point
        % only AFTER j has passed it (no bending needed).  headOn = routes run against each other (then a slow-down is not enough, i must go around).
        f = 1; headOn = false; ni = size(pi_,1); nj = size(pj,1); if ni < 2 || nj < 1, return, end
        ai = [0; cumsum(sqrt(sum(diff(pi_,1,1).^2,2)))]; aj = zeros(nj,1); if nj > 1, aj = [0; cumsum(sqrt(sum(diff(pj,1,1).^2,2)))]; end
        d0 = norm(pi_(1,:) - pj(1,:));
        hz = dHard + 2.0;       % only a conflict that is close ahead slows me down; far ahead the plan-time negotiation has already dealt with it (no needless crawling)
        for a = 1:ni
            if ai(a) > hz, break, end
            for b = 1:nj
                dab = norm(pi_(a,:) - pj(b,:)); if dab >= Rc, continue, end
                if dab >= d0 - 0.05, continue, end      % already close but driving apart: no conflict
                ti = ai(a)/vi; tj = aj(b)/vj; tm = 0.6*Rc/vj + 0.4;
                if abs(ti - tj) < tm
                    % the arrival-time test is a coarse filter: check the SAME instants of both robots - if they stay clear of each other all the time (parallel / far apart) nobody has to slow down
                    Tm = min(ai(end)/vi,aj(end)/vj); tt = 0:0.25:Tm;
                    if numel(tt) >= 2 && nj > 1
                        qi = [interp1(ai,pi_(:,1),vi*tt(:)) interp1(ai,pi_(:,2),vi*tt(:))];
                        qj = [interp1(aj,pj(:,1),vj*tt(:)) interp1(aj,pj(:,2),vj*tt(:))];
                        if min(sqrt(sum((qi - qj).^2,2))) >= Rc, continue, end
                    end
                    f = min(1,max(0,ti/(tj + tm)));
                    if ai(a) < dHard, f = 0; end
                    hi = pi_(2,:) - pi_(1,:); hj = [0 0]; if nj > 1, hj = pj(2,:) - pj(1,:); end
                    headOn = dot(hi,hj) < -0.5*norm(hi)*norm(hj);
                    return
                end
            end
        end
    end
    function tf = isAhead(Pi,pi_,Pj)
        h = [0 0]; for a = 2:size(pi_,1), h = pi_(a,:) - Pi; if norm(h) > 0.2, break, end, end
        if norm(h) < 0.05, tf = true; return, end
        tf = dot((Pj - Pi)/max(norm(Pj-Pi),1e-6),h/norm(h)) > 0.3;
    end
    function [tf,node] = hasCycle(E,N)
        A = false(N); for q = 1:size(E,1), A(E(q,1),E(q,2)) = true; end
        tf = false; node = 0;
        for s = 1:N
            vis = false(1,N); stack = s;
            while ~isempty(stack)
                u = stack(end); stack(end) = []; nx = find(A(u,:));
                for v = nx
                    if v == s, tf = true; node = s; return, end
                    if ~vis(v), vis(v) = true; stack(end+1) = v; end %#ok<AGROW>
                end
            end
        end
    end
end
end
