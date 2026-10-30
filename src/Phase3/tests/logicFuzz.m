function R = logicFuzz(seed,varargin)
%LOGICFUZZ  Physics-free test of the fleet manager's JOB ALLOCATION / LOCK logic with many random task sets (hot shelf, same station, mixed, pinned robots, faults).
%   The real Phase3Fleet runs; robots are mock robots that follow the same message protocol (exchange) and need realistic times for driving and for every arm phase.
%   Checked: (1) two robots never WORK (arm out) closer than minWork (1.5 m);  (2) every task is done;  (3) no task waits unreasonably long;  (4) boxes / slots / stations stay consistent;
%   (5) how many robots really work at the same time (parallelism).   Traffic / collision avoidance is NOT part of this test (see WorldSim).
p = inputParser; addParameter(p,'mode','mixed'); addParameter(p,'nTasks',18); addParameter(p,'tEnd',1500); addParameter(p,'minWork',1.5); addParameter(p,'pinned',false); addParameter(p,'faults',0); addParameter(p,'verbose',false); addParameter(p,'dt',0.1); addParameter(p,'diagT',[]);
parse(p,varargin{:}); o = p.Results;
rand('twister',1000 + seed); randn('state',1000 + seed); %#ok<RAND>
C = helperFakeConfig(30,10,54,seed);
F = Phase3Fleet(C,6,'answer','auto','verbose',0,'planner','prioritised'); F.map2D = false; F.setMode('manual'); F.mMaxGroup = 1;
N = 6; dt = o.dt;
% ---- scenario: put boxes on a HOT shelf (or the same area) so that many tasks want the same shelf / station
S = F.slots; isSt = find(strcmp({S.role},'store')); face = cellfun(@(x) x(1:find(x=='-',1)-1),{S.id},'UniformOutput',false);
faces = unique(face(isSt)); hot = faces{1 + floor(rand*numel(faces))}; hotAB = [hot(1:end-1) '*'];
nT = o.nTasks; jobs = {};
stationHot = 1 + floor(rand*3); stCnt = zeros(1,3);
switch o.mode
    case 'hotshelf', pHot = 1; case 'mixed', pHot = 0.5; otherwise, pHot = 0;   % 'random'
end
% make sure enough boxes stand in the hot shelf (both faces): fill free slots of the hot shelf with spare boxes
if pHot > 0
    fs = find(strcmp({S.state},'FREE') & strcmp({S.role},'store') & strncmp(face,hot(1:end-1),numel(hot)-1));
    fs = fs(randperm(numel(fs))); nAdd = min(numel(fs),ceil(nT*pHot)+2);
    for q = 1:nAdd, F.addBoxAt(S(fs(q)).id,0); end
end
S = F.slots; stored = find(strcmp({F.cartons.state},'STORED')); dock = find(strcmp({F.cartons.state},'DOCK'));
isHot = @(j) strncmp(S(F.cartons(j).slot).id,hot(1:end-1),numel(hot)-1);
hotB = stored(arrayfun(isHot,stored)); coldB = setdiff(stored,hotB);
hotB = hotB(randperm(numel(hotB))); coldB = coldB(randperm(numel(coldB))); dock = dock(randperm(numel(dock)));
freeSt = find(strcmp({S.state},'FREE') & strcmp({S.role},'store')); freeSt = freeSt(randperm(numel(freeSt)));
nHot = round(nT*pHot); nPut = round(nT*0.3);
for q = 1:nT
    k = 0; if o.pinned, k = 1 + mod(q-1,N); end
    if q <= nPut && q <= numel(dock) && q <= numel(freeSt)
        jobs{end+1} = {'putaway',dock(q),S(freeSt(q)).id,0,k}; %#ok<AGROW>
    else
        if numel(hotB) > 0 && (q - nPut) <= nHot, j = hotB(1); hotB(1) = []; elseif ~isempty(coldB), j = coldB(1); coldB(1) = []; elseif ~isempty(hotB), j = hotB(1); hotB(1) = []; else, break, end
        st = stationHot; if strcmp(o.mode,'random') || rand < 0.3, st = 1 + floor(rand*3); end
        if stCnt(st) >= 8, [~,st] = min(stCnt); end       % a station has 9 places: never more than 8 boxes per station (a full station is shipped, that is a different story)
        stCnt(st) = stCnt(st) + 1;
        jobs{end+1} = {'retrieve',j,S(F.cartons(j).slot).id,st,k}; %#ok<AGROW>
    end
end
rand('twister',2000 + seed);
% ---- mock robots
M = repmat(struct('pose',[0 0 0],'state',0,'seq',0,'status','idle','job',[],'ph',0,'tph',0,'carry',false,'goal',[0 0],'fault',false,'dstSeen',0),1,N);
for k = 1:N, M(k).pose = [F.robots(k).pose(1:2) 0]; end
t = 0; viol = 0; faceViol = 0; maxPar = 0; parT = zeros(1,N+1); worstPair = inf; nDoneLast = 0; lastProg = 0; violList = {};
faultAt = sort(10 + 200*rand(1,o.faults)); fi = 1;
released = false;
while t < o.tEnd
    t = t + dt;
    if ~released && t >= 20
        released = true;
        for q = 1:numel(jobs)
            jb = jobs{q};
            if strcmp(jb{1},'retrieve'), if jb{5} > 0, F.manualTask(jb{5},'retrieve',jb{2},jb{3},jb{4}); else, F.moveJob(jb{3},'station',jb{4},0); end
            else, if jb{5} > 0, F.manualTask(jb{5},'putaway',jb{2},jb{3},0); else, F.moveJob(F.slots(F.cartons(jb{2}).slot).id,jb{3},0,0); end, end
        end
    end
    for k = 1:N
        m = M(k);
        rep = struct('t',t,'pose',m.pose,'state',m.state,'carry',m.carry,'wp',zeros(0,2),'goal',m.goal,'jobSeq',m.seq,'jobStatus',m.status,'scanReq',false,'fault',false,'planReq',false,'planSeq',0);
        cmd = F.exchange(k,rep);
        if isfield(cmd,'dstUpd') && isfield(m,'dstSeen') && cmd.dstUpd.seq > m.dstSeen && ~isempty(cmd.dstUpd.dst)
            m.dstSeen = cmd.dstUpd.seq; if ~isempty(m.job) && ~m.carry, m.job.dst = cmd.dstUpd.dst; end
        end
        if ~isempty(cmd.job) && cmd.job.seq > m.seq && any(m.state == [0 7])
            m.job = cmd.job; m.seq = cmd.job.seq; m.status = 'running'; m.ph = 1; m.state = 1; m.goal = m.job.src.staging(1:2); m.carry = false;
        end
        if cmd.hold || cmd.scale < 0.05, mv = 0; else, mv = 1; end
        switch m.ph
            case 1  % drive to the source
                [m,arr] = localDrive(m,m.goal,1.4*mv,dt); if arr, m.ph = 2; m.tph = 0; m.state = 10; end
            case 2  % dock / extend / grasp / retract at the source
                m.tph = m.tph + dt; if m.tph > 6, m.state = 2; end, if m.tph > 14, m.state = 3; end, if m.tph > 20, m.state = 4; m.carry = true; end
                if m.tph > 28, m.ph = 3; m.state = 5; m.goal = m.job.dst.approach(1:2); end
            case 3  % drive to the destination
                [m,arr] = localDrive(m,m.goal,1.05*mv,dt); if arr, m.ph = 4; m.tph = 0; m.state = 10; end
            case 4  % dock / place / release / stow
                m.tph = m.tph + dt; if m.tph > 6, m.state = 6; end, if m.tph > 16, m.state = 8; m.carry = false; end, if m.tph > 20, m.state = 9; end
                if m.tph > 26, m.ph = 5; m.state = 7; m.status = 'done'; m.goal = C.P3.robots(k).home; end
            case 5  % home
                [m,arr] = localDrive(m,m.goal,1.4*mv,dt); if arr, m.state = 0; m.ph = 0; end
        end
        M(k) = m;
    end
    % invariant: arms out / docking robots must be far enough apart
    work = find(arrayfun(@(m) any(m.state == [10 2 3 4 6 8 9]), M));
    par = numel(work); maxPar = max(maxPar,par); parT(par+1) = parT(par+1) + dt;
    for a = 1:numel(work)-1
        for b = a+1:numel(work)
            d = norm(M(work(a)).pose(1:2) - M(work(b)).pose(1:2)); worstPair = min(worstPair,d);
            if d < o.minWork, viol = viol + 1; if numel(violList) < 3, violList{end+1} = sprintf('t=%.0f %s/%s %.2f m',t,F.robots(work(a)).name,F.robots(work(b)).name,d); end, end %#ok<AGROW>
        end
    end
    % invariant (round 20): per shelf FACE at most faceCap robots work at the same time, at least faceSep apart along the face
    srcW = find(arrayfun(@(m) any(m.state == [10 2 3 4]) && ~isempty(m.job) && ~m.carry, M));
    for a = 1:numel(srcW)
        na = M(srcW(a)).job.src.name; ia = find(na == '-',1) - 1; if M(srcW(a)).job.src.shelf == 0, continue, end
        cnt = 1;
        for b = [1:a-1 a+1:numel(srcW)]
            nb = M(srcW(b)).job.src.name; if M(srcW(b)).job.src.shelf == 0 || ~strcmp(na(1:ia),nb(1:find(nb == '-',1) - 1)), continue, end
            cnt = cnt + 1; dy = abs(M(srcW(a)).job.src.xyz(2) - M(srcW(b)).job.src.xyz(2));
            if dy < F.faceSep - 0.01 && b > a, faceViol = faceViol + 1; end
        end
        if cnt > F.faceCap(M(srcW(a)).job.src.shelf), faceViol = faceViol + 1; end
    end
    if ~isempty(o.diagT) && any(abs(t - o.diagT) < dt/2 + 1e-9), localDiag(F,M,t); end
    nd = sum(strcmp({F.tasks.status},'DONE')); if nd > nDoneLast, nDoneLast = nd; lastProg = t; end
    openT = sum(ismember({F.tasks.status},{'QUEUED','ACTIVE','RUNNING','HELD'}));
    if released && openT == 0 && all([M.ph] == 0 | [M.ph] == 5), if all([M.ph] == 0) || t - lastProg > 60, break, end, end
    if released && t - lastProg > 400, break, end      % no progress: dead
end
T = F.tasks; st = {T.status};
R = struct('seed',seed,'mode',o.mode,'n',numel(T),'done',sum(strcmp(st,'DONE')),'failed',sum(strcmp(st,'FAILED')),'open',sum(ismember(st,{'QUEUED','ACTIVE','RUNNING','HELD'})), ...
    'tEnd',t,'viol',viol,'violList',{violList},'worstPair',worstPair,'maxPar',maxPar,'parFrac',parT/sum(parT),'maxQueue',max([0 [T.assigned]-[T.created]]),'makespan',t-20);
% consistency
bad = 0;
for j = 1:F.M
    c = F.cartons(j);
    if any(strcmp(c.state,{'STORED','DOCK'}))
        if c.slot < 1 || ~strcmp(F.slots(c.slot).state,'OCCUPIED') || F.slots(c.slot).item ~= j, bad = bad + 1; end
    elseif strcmp(c.state,'STATION')
        s = c.stationSlot; if s(1) < 1 || ~strcmp(F.stations(s(1)).state{s(2)},'OCC') || F.stations(s(1)).item(s(2)) ~= j, bad = bad + 1; end
    end
end
R.faceViol = faceViol; R.bad = bad; R.locksLeft = size(F.locks,1);
end

function [m,arr] = localDrive(m,g,v,dt)
d = g(:).' - m.pose(1:2); n = norm(d); arr = false;
if n < 0.05, arr = true; return, end
s = min(n,v*dt); m.pose(1:2) = m.pose(1:2) + d/n*s; m.pose(3) = atan2(d(2),d(1)); if norm(g(:).' - m.pose(1:2)) < 0.05, arr = true; end
end

function localDiag(F,M,t)
printf('--- DIAG t=%.0f ---\n',t);
for k = 1:numel(M), r = F.robots(k); printf('  %-6s state=%2d task=%d avail=%d pos=(%.0f,%.0f)\n',r.name,M(k).state,r.task,F.robotAvailable(k),M(k).pose(1),M(k).pose(2)); end
L = F.locks; printf('  locks: '); for q = 1:size(L,1), printf('%s=%s ',L{q,1},F.robots(L{q,2}).name); end, printf('\n');
for ti = 1:numel(F.tasks)
    T = F.tasks(ti); if ~strcmp(T.status,'QUEUED'), continue, end
    sk = F.bayKeys(T.src); why = '';
    if ~F.lockFree(sk,0), why = [why 'SRC-LOCK ']; end
    if T.dstKind == 2
        sset = T.dstStation; if sset == 0, sset = 1:numel(F.stations); end
        free = 0; for st = sset, for q = find(strcmp(F.stations(st).state,'FREE')), if F.stFree(st,q,0), free = free + 1; end, end, end
        if free == 0, why = [why 'NO-FREE-PLACE ']; end
    end
    printf('  queued task %d %s %s pinned=%d -> %s\n',T.id,T.type,F.slots(T.src).id,T.pinned,why);
end
end
