function R = WorldSim(varargin)
%WORLDSIM  Headless "digital twin" of the Phase 3 warehouse for traffic tests (Octave / MATLAB, no Simulink, no Unreal).
%   Uses the REAL Phase3Fleet (tasks, locks, planning token, traffic) and mock robots that behave like the real controller:
%     * 4-connected A* on the inflated occupancy grid (like plannerAStarGrid, 'DiagonalSearch','off'), optional any-angle string pulling,
%     * plan around the other robots (positions + committed routes), wide-berth first, fallbacks,
%     * ROTATE / DRIVE navigation with the controller's constants, fleet speed scale / hold,
%     * arm work durations (scan, extend, grasp, retract, place, release, stow) taken from real logs.
%   Result R: collisions, min separation, stop / wait statistics, route lengths, throughput.
%
%   R = WorldSim('tEnd',600,'seed',1,'pull',true,'verbose',0,'script',{})
%   script: cell array of {t, 'p3 command text'} entries run through Phase3Fleet methods (see localCommand)
p = inputParser;
addParameter(p,'tEnd',600); addParameter(p,'seed',1); addParameter(p,'pull',true); addParameter(p,'verbose',0);
addParameter(p,'nRobots',6); addParameter(p,'auto',true); addParameter(p,'perRobot',3); addParameter(p,'autoPut',true); addParameter(p,'script',{}); addParameter(p,'stationCap',0); addParameter(p,'speed',1.55); addParameter(p,'speedCarry',1.15);
addParameter(p,'dock',14); addParameter(p,'tag',''); addParameter(p,'probe',false); addParameter(p,'faultPlace',[]); addParameter(p,'faultPick',[]); addParameter(p,'stallTask',[]); addParameter(p,'faultAlways',[]); addParameter(p,'planner','mstar'); addParameter(p,'mg',4); addParameter(p,'mn',10000); addParameter(p,'big',false);      % test: tasks whose robot faults when it starts to place / to grasp (first time only)
parse(p,varargin{:}); o = p.Results;
setenv('P3_ROOT',fullfile(tempdir,['p3world' o.tag]));
here = fileparts(mfilename('fullpath')); addpath(fullfile(here,'..')); addpath(here);
rand('twister',o.seed);
if o.big, C = helperFakeConfig(30,10,54,o.seed); else, C = helperFakeConfig(); end, C.NavSpeed = o.speed; C.NavSpeedCarry = o.speedCarry; C.SafetyRadius = 0.45; C.BaseRadius = 0.34; C.Clearance = 0.30; C.CarryExtraClearance = 0.45;
N = o.nRobots; WH = C.WH;
F = Phase3Fleet(C,N,'answer','auto','dwell',20,'verbose',o.verbose,'planner',o.planner); F.map2D = false; F.autoPutaway = o.autoPut; F.mTmax = 12; F.mMaxGroup = o.mg; F.mMaxNodes = o.mn;
if o.stationCap > 0, for q_ = 1:numel(F.stations), for f_ = {'state','item','since','by'}, v_ = F.stations(q_).(f_{1}); F.stations(q_).(f_{1}) = v_(1:o.stationCap); end, end, end      % test option: smaller stations -> auto mode reaches its goal sooner
if o.dock > 0, F.dockAdd(o.dock,true); end
if o.auto, F.setAuto(true,20,60,0); F.autoPerRobot = o.perRobot; end
G = localGrids(WH,C);
F.gridFree = @(fx,fy,carry) localGridFree(G,fx,fy,carry);
F.freeFcn = @(xy) ~G.free(min(G.ny,max(1,floor(xy(2)/G.res) + 1)),min(G.nx,max(1,floor((xy(1) + G.X/2)/G.res) + 1)));
if o.probe, R = struct('G',G,'plan',@(start,goal,carry,cmd) localPlan(G,start,goal,carry,1,[],cmd,false,o.pull),'one',@(start,goal,carry,cmd,bp,br) localPlanOne(G,start,goal,carry,cmd,false,o.pull,bp,br),'astar',@(B,s,g) localAStar(G,B,s,g),'stamp',@(B,s,cmd,bp,br) localStamp(B,G,s,cmd,bp,br)); return, end
dt = 0.1; t = 0;
M = struct('pose',{},'state',{},'wp',{},'wpi',{},'goal',{},'mode',{},'job',{},'seq',{},'status',{},'t0',{},'carry',{},'planWait',{},'planSeq',{},'grant',{}, ...
    'rp',{},'trail',{},'stopT',{},'legLen',{},'legStart',{},'legShort',{},'v',{},'needPlan',{},'replanFlag',{},'path',{},'evSeen',{},'dstSeen',{},'evActive',{},'evPt',{},'evT',{},'fault',{},'recSeen',{},'fired',{},'abortSeen',{},'stalled',{},'mSeen',{});
for k = 1:N
    h = C.P3.robots(k).home;
    M(k) = struct('pose',[h 0],'state',0,'wp',zeros(0,2),'wpi',1,'goal',h,'mode','ROTATE','job',[],'seq',0,'status','idle','t0',0,'carry',false,'planWait',false,'planSeq',0,'grant',false, ...
        'rp',0,'trail',h,'stopT',0,'legLen',0,'legStart',[h],'legShort',0,'v',0,'needPlan',false,'replanFlag',false,'path',h,'evSeen',0,'dstSeen',0,'evActive',false,'evPt',[0 0],'evT',0,'fault',false,'recSeen',0,'fired',[],'abortSeen',0,'stalled',false,'mSeen',0);
end
S = struct('shelfHit',0,'minShelf',inf,'pendWait',0,'minSep',inf,'minSepCarry',inf,'col',0,'colCarry',0,'stops',[],'legRatio',[],'planTime',0,'plans',0,'maxStop',0,'stopLong',0,'stopVeryLong',0,'events',{{}},'trace',zeros(0,11),'planLog',{{}});
stopRun = zeros(1,N); shelfNow = false(1,N); colNow = false; colNowC = false; script = o.script; si = 1;
tStart = tic;
while t < o.tEnd && ~F.stopFlag
    t = t + dt;
    while si <= numel(script) && script{si}{1} <= t, localCommand(F,script{si}{2}); si = si + 1; end
    P = zeros(N,2);
    for k = 1:N
        m = M(k); P(k,:) = m.pose(1:2);
        wpRep = zeros(0,2);
        if any(m.state == [1 5 7]) && ~m.needPlan && m.wpi <= size(m.wp,1), wpRep = m.wp(m.wpi:min(end,m.wpi+39),:); end
        rep = struct('t',t,'pose',m.pose,'state',m.state,'carry',m.carry,'wp',wpRep,'goal',m.goal,'jobSeq',m.seq,'jobStatus',m.status,'scanReq',false,'fault',m.fault, ...
            'planReq',m.planWait,'planSeq',m.planSeq);
        cmd = F.exchange(k,rep);
        if cmd.planGrant, m.grant = true; end
        if cmd.finish && m.state == 0 && isempty(cmd.job) && isempty(m.wp), m.state = 12; m.v = 0; end      % the real controller drives home and parks; the test world just parks
        % new job
        if ~isempty(cmd.job) && cmd.job.seq > m.seq && any(m.state == [0 7])
            m.job = cmd.job; m.seq = cmd.job.seq; m.status = 'running'; m.state = 1; m.goal = m.job.src.staging; m.t0 = t; m.needPlan = true; m.replanFlag = false; m.carry = false;
        end
        if m.fault && isfield(cmd,'recover') && cmd.recover > m.recSeen       % fleet recovery (mock of the controller: FAULT_HOLD -> retract + new destination / stow + home)
            m.recSeen = cmd.recover; m.fault = false; m.t0 = t;
            if m.carry, m.job.dst = cmd.dest; m.state = 4; else, m.state = 9; end
        end
        if isfield(cmd,'abort') && cmd.abort > m.abortSeen         % fleet watchdog (mock of the controller: FAULT in an arm state)
            m.abortSeen = cmd.abort;
            if any(m.state == [10 2 3 4 6]) && ~m.fault, m.fault = true; m.state = 11; m.v = 0; m.stalled = false; end
        end
        if isfield(cmd,'mroute') && cmd.mroute.seq > m.mSeen         % M*: new collision-free route from the fleet
            m.mSeen = cmd.mroute.seq;
            if any(m.state == [1 5 7]) && ~m.needPlan && ~m.planWait && ~m.evActive && ~isempty(cmd.mroute.wp), m.wp = [m.pose(1:2); cmd.mroute.wp]; m.wpi = 2; end
        end
        if cmd.replanSeq ~= m.rp && any(m.state == [1 5 7])
            m.rp = cmd.replanSeq; m.needPlan = true; m.replanFlag = true;
        end
        if isfield(cmd,'dstUpd') && cmd.dstUpd.seq > m.dstSeen && ~isempty(cmd.dstUpd.dst)
            m.dstSeen = cmd.dstUpd.seq; if ~isempty(m.job) && m.state == 1 && ~m.carry, m.job.dst = cmd.dstUpd.dst; end
        end
        if numel(cmd.evade) >= 3 && cmd.evade(3) ~= m.evSeen
            m.evSeen = cmd.evade(3);
            if any(m.state == [1 5 7]) && all(isfinite(cmd.evade(1:2))), m.evActive = true; m.evPt = cmd.evade(1:2); m.wp = [m.pose(1:2); m.evPt]; m.wpi = 2; m.needPlan = false; m.planWait = false; m.mode = 'ROTATE'; m.evT = t + 15;
                if norm(m.evPt - m.pose(1:2)) > 3      % a long way (waiting point): planned route around the shelves like the controller does, never a straight line
                    try, wpx = localPlan(G,m.pose(1:2),m.evPt,m.carry > 0,1,[],cmd,false,o.pull); if size(wpx,1) >= 2, m.wp = [m.pose(1:2); wpx(2:end,:)]; end, catch, end
                    L = sum(sqrt(sum(diff(m.wp).^2,2))); m.evT = t + max(15,8 + 1.6*L/G.v);
                end
            end
        end
        % planning (one robot at a time: the fleet grants the slot)
        if m.needPlan
            if ~m.grant, m.planWait = true;
            else
                m.planWait = false; m.grant = false; tp = tic;
                [wp,info] = localPlan(G,m.pose(1:2),m.goal,m.carry,m.state,m.job,cmd,m.replanFlag,o.pull);
                S.planTime = S.planTime + toc(tp); S.plans = S.plans + 1; S.planLog{end+1} = struct('t',t,'k',k,'wp',wp,'others',cmd.others,'routes',cmd.routes,'info',info);
                m.wp = wp; m.wpi = min(2,size(wp,1)); m.needPlan = false; m.replanFlag = false; m.planSeq = m.planSeq + 1; m.mode = 'ROTATE'; m.path = [m.path; m.pose(1:2)];
                m.legStart = m.pose(1:2); m.legShort = info.direct; m.legLen = 0;
            end
        end
        sp = cmd.scale; hold = cmd.hold;
        [m,S] = localStep(m,S,C,dt,t,sp,hold,cmd,k);
        if ~isempty(m.job) && ~m.fault
            tk = m.job.task;
            if (m.state == 6 && any(o.faultPlace == tk) && ~any(m.fired == tk)) || (m.state == 3 && any(o.faultPick == tk) && ~any(m.fired == tk))
                m.fired(end+1) = tk; m.fault = true; m.state = 11; m.v = 0;
            end
            if m.state == 6 && any(o.faultAlways == tk), m.fault = true; m.state = 11; m.v = 0; end
            if m.state == 2 && any(o.stallTask == tk) && ~any(m.fired == -tk), m.stalled = true; m.t0 = t; end
        end
        if m.stalled && m.state == 2, m.t0 = t; end      % frozen in the extend state until the watchdog aborts
        if m.stalled && m.state ~= 2, m.stalled = false; end
        M(k) = m;
        if abs(t*5 - round(t*5)) < 1e-6, S.trace(end+1,:) = [t k m.pose(1:2) m.state m.carry sp hold m.v m.wpi size(m.wp,1)]; end
        % stop statistics (robot should be driving but is not)
        driving = any(m.state == [1 5 7]) && ~m.needPlan && ~m.planWait && norm(m.goal - m.pose(1:2)) > 1.0;
        pendNow = false; try, pendNow = F.robots(k).pendHold; catch, end
        if pendNow, S.pendWait = S.pendWait + dt; elseif driving && m.v < 0.05 && strcmp(m.mode,'DRIVE') || (any(m.state == [1 5 7]) && (cmd.scale < 0.05 || cmd.hold) && norm(m.goal - m.pose(1:2)) > 1.0)
            stopRun(k) = stopRun(k) + dt;
        else
            if stopRun(k) > 2.0, S.stops(end+1,:) = [k t stopRun(k) m.pose(1:2) m.state m.carry]; end %#ok<AGROW>
            if stopRun(k) > 2.0, S.stopLong = S.stopLong + 1; end
            if stopRun(k) > 5.0, S.stopVeryLong = S.stopVeryLong + 1; end
            S.maxStop = max(S.maxStop,stopRun(k)); stopRun(k) = 0;
        end
    end
    % separation
    low = inf; lowC = inf; pairC = [0 0];
    for i = 1:N-1
        for j = i+1:N
            d = norm(M(i).pose(1:2) - M(j).pose(1:2)); low = min(low,d);
            arm = @(m) any(m.state == [10 2 3 4 6 8 9 11]); dc = d - 0.6*(M(i).carry + M(j).carry) - 0.5*(arm(M(i)) + arm(M(j))); if dc < lowC, pairC = [i j]; end, lowC = min(lowC,dc);
        end
    end
    % robot vs shelf: clearance of every chassis centre to the shelf rectangles (a chassis circle inside a shelf = the robot drives through the rack)
    for k = 1:N
        pk = M(k).pose(1:2); clMin = inf; qMin = 0;
        for q = 1:size(C.WH.shelves,1)
            sh = C.WH.shelves(q,:); dx = max(abs(pk(1) - sh(1)) - sh(4)/2,0); dy = max(abs(pk(2) - sh(2)) - sh(3)/2,0); cl = hypot(dx,dy) - 0.45;
            if cl < clMin, clMin = cl; qMin = q; end
        end
        S.minShelf = min(S.minShelf,clMin);
        if clMin < -0.02 && ~shelfNow(k), shelfNow(k) = true; S.shelfHit = S.shelfHit + 1; S.events{end+1} = sprintf('t=%.1f SHELF-CONTACT r%d %.2f m inside shelf %d at (%.1f,%.1f) state %d',t,k,-clMin,qMin,pk(1),pk(2),M(k).state);
        elseif clMin > 0.05, shelfNow(k) = false; end
    end
    S.minSep = min(S.minSep,low); S.minSepCarry = min(S.minSepCarry,lowC);
    if low < 0.90 && ~colNow, colNow = true; S.col = S.col + 1; S.events{end+1} = sprintf('t=%.1f CONTACT d=%.2f',t,low); elseif low > 1.0, colNow = false; end
    if lowC < 0.90 && ~colNowC, colNowC = true; S.colCarry = S.colCarry + 1; S.events{end+1} = sprintf('t=%.1f CARRY-CONTACT r%d(c%d) r%d(c%d) d=%.2f at (%.1f,%.1f) st=%d/%d',t,pairC(1),M(pairC(1)).carry,pairC(2),M(pairC(2)).carry,lowC,M(pairC(1)).pose(1),M(pairC(1)).pose(2),M(pairC(1)).state,M(pairC(2)).state); elseif lowC > 1.0, colNowC = false; end
    if ~isempty(F.questions) && mod(round(t),5) == 0, F.reply('auto'); end
end
for k = 1:N, if stopRun(k) > 2.0, S.stops(end+1,:) = [k t stopRun(k) M(k).pose(1:2) M(k).state M(k).carry]; end, end %#ok<AGROW>
T = F.tasks; done = sum(strcmp({T.status},'DONE'));
R = struct('F',F,'shelfHit',S.shelfHit,'minShelf',S.minShelf,'pendWait',S.pendWait,'evlog',{F.evlog},'mode',F.mode,'pullOvers',F.pullOvers,'mstat',F.mstat,'collisions',S.col,'collisionsCarry',S.colCarry,'minSep',S.minSep,'minSepCarry',S.minSepCarry,'tasksDone',done,'tasksFailed',sum(strcmp({T.status},'FAILED')), ...
    'stopsOver2s',S.stopLong,'stopsOver5s',S.stopVeryLong,'maxStop',S.maxStop,'plans',S.plans,'planTime',S.planTime,'replans',F.replans,'cycles',F.deadlocks,'throughput',done/(o.tEnd/60), ...
    'events',{S.events},'mstat',F.mstat,'trace',S.trace,'planLog',{S.planLog},'stops',S.stops,'fleet',F,'wall',toc(tStart));
end

% ------------------------------------------------------------------------------------------------------------------------------ world model
function f = localGridFree(G,fx,fy,carry)
B = G.free; if carry, B = G.carry; end
i = min(G.ny,max(1,floor(fy/G.res) + 1)); j = min(G.nx,max(1,floor((fx + G.X/2)/G.res) + 1)); f = ~B(sub2ind(size(B),i,j));
end
function G = localGrids(WH,C)
G.res = 0.25; G.X = WH.X; G.Y = WH.Y; G.nx = ceil(WH.X/G.res); G.ny = ceil(WH.Y/G.res);
raw = false(G.ny,G.nx);
raw(1,:) = true; raw(end,:) = true; raw(:,1) = true; raw(:,end) = true;
for k = 1:size(WH.shelves,1), raw = localRect(raw,G,WH.shelves(k,1),WH.shelves(k,2),WH.shelves(k,4),WH.shelves(k,3)); end
for k = 1:size(WH.tables,1), raw = localRect(raw,G,WH.tables(k,1),WH.tables(k,2),WH.tables(k,3),WH.tables(k,4)); end
if isfield(WH,'stands') && isfield(WH,'standSize'), for k = 1:size(WH.stands,1), raw = localRect(raw,G,WH.stands(k,1),WH.stands(k,2),WH.standSize,WH.standSize); end, end
G.raw = raw;
G.free = localInflate(raw,G,C.BaseRadius + C.Clearance);
G.carry = localInflate(raw,G,C.BaseRadius + C.Clearance + C.CarryExtraClearance);
G.wideExtra = 0.9; G.v = C.NavSpeed; G.vCarry = C.NavSpeedCarry;
G.freeW = localInflate(G.free,G,0.9); G.carryW = localInflate(G.carry,G,0.9);
end
function B = localRect(B,G,cx,cy,w,h)
x0 = (cx - w/2 + G.X/2)/G.res; x1 = (cx + w/2 + G.X/2)/G.res; y0 = (cy - h/2)/G.res; y1 = (cy + h/2)/G.res;
j0 = max(1,floor(x0)+1); j1 = min(size(B,2),ceil(x1)); i0 = max(1,floor(y0)+1); i1 = min(size(B,1),ceil(y1));
B(i0:i1,j0:j1) = true;
end
function Bi = localInflate(B,G,r)
n = ceil(r/G.res); [dx,dy] = meshgrid(-n:n,-n:n); K = (dx.^2 + dy.^2) <= (r/G.res)^2;
[ii,jj] = find(B); Bi = B;
for q = 1:numel(ii)
    for s = find(K(:)).'
        a = ii(q) + dy(s); b = jj(q) + dx(s);
        if a >= 1 && a <= size(B,1) && b >= 1 && b <= size(B,2), Bi(a,b) = true; end
    end
end
end
function [i,j] = localCell(G,xy), j = min(G.nx,max(1,floor((xy(1) + G.X/2)/G.res) + 1)); i = min(G.ny,max(1,floor(xy(2)/G.res) + 1)); end
function xy = localXY(G,i,j), xy = [(j - 0.5)*G.res - G.X/2, (i - 0.5)*G.res]; end

% ------------------------------------------------------------------------------------------------------------------------------ planning like the controller
function [wp,info] = localPlan(G,start,goal,carrying,state,job,cmd,replan,pull)
% geometry (gate point at a station) + the SHARED plan-time negotiation (fleetPlanRoute) used by the controller as well
gate = goal; wpTail = zeros(0,2);
if state == 5 && ~isempty(job) && job.dst.kind == 2
    gate = goal - 1.6*[cos(job.dst.yaw) sin(job.dst.yaw)]; wpTail = goal;
end
v = G.v; if carrying, v = G.vCarry; end
fcn = @(bp,br) localPlanOne(G,start,gate,carrying,cmd,replan,pull,bp,br);
[wp1,pinfo] = fleetPlanRoute(fcn,start,gate,v,cmd.routes,cmd.planPar,carrying);
if isempty(wp1), wp1 = [start; gate]; end
wp = fleetSimplify([wp1; wpTail]);
info = pinfo; info.direct = norm(goal - start);
end
function wp = localPlanOne(G,start,goal,carrying,cmd,replan,pull,bp,br)
base = G.free; if carrying, base = G.carry; end
wp = [];
if ~replan
    if carrying, Bw = G.carryW; else, Bw = G.freeW; end
    Bs = localStamp(Bw,G,start,cmd,bp,br,carrying); wp = localAStar(G,Bs,start,goal);
    if size(wp,1) < 2 || (size(wp,1) == 2 && norm(start - goal) > 3), wp = []; elseif pull, wp = localStringPull(G,Bs,wp); end
end
if isempty(wp)
    Bs = localStamp(base,G,start,cmd,bp,br,carrying); wp = localAStar(G,Bs,start,goal);
    if size(wp,1) < 2 || (size(wp,1) == 2 && norm(start - goal) > 3), wp = []; elseif pull, wp = localStringPull(G,Bs,wp); end
end
if isempty(wp) && isempty(bp)
    c2 = cmd; c2.stampPos = max(0.95,0.85*cmd.stampPos);
    Bs = localStamp(base,G,start,c2,bp,br,carrying); wp = localAStar(G,Bs,start,goal);
    if size(wp,1) < 2 || (size(wp,1) == 2 && norm(start - goal) > 3), wp = []; elseif pull, wp = localStringPull(G,Bs,wp); end
    if isempty(wp), wp = localAStar(G,base,start,goal); if pull, wp = localStringPull(G,base,wp); end, end
end
end
function B = localStamp(B,G,start,cmd,bp,br,carrying)
if nargin < 7, carrying = false; end
oxy = cmd.others; if size(oxy,2) < 3, oxy(:,3) = 0; end
S = [oxy(:,1:2); bp]; Rr = [cmd.stampPos + oxy(:,3) + cmd.planPar.carryExtra*carrying; repmat(br,size(bp,1),1)];
for q = 1:size(S,1)
    c = S(q,:); r = min(Rr(q),max(0.3,norm(c - start) - 0.12));
    n = ceil(r/G.res); [ci,cj] = localCell(G,c);
    for a = -n:n, for b = -n:n
        if a^2 + b^2 <= (r/G.res)^2, ii = ci + a; jj = cj + b; if ii >= 1 && ii <= G.ny && jj >= 1 && jj <= G.nx, B(ii,jj) = true; end, end
    end, end
end
end
function wp = localAStar(G,B,start,goal)
[si,sj] = localCell(G,start); [gi,gj] = localCell(G,goal);
[si,sj] = localNearestFree(B,si,sj); [gi,gj] = localNearestFree(B,gi,gj);
if isempty(si) || isempty(gi), wp = []; return, end
ny = G.ny; nx = G.nx; found = false;
binf = [tempname() '.bin']; outf = [tempname() '.txt'];
fid = fopen(binf,'wb'); fwrite(fid,int32([ny nx si sj gi gj]),'int32'); fwrite(fid,uint8(B(:)),'uint8'); fclose(fid);
system(sprintf('"%s" %s %s',fullfile(fileparts(mfilename('fullpath')),'fastastar'),binf,outf));
d = dir(outf); if d.bytes > 0, pp = load(outf); else, pp = []; end, delete(binf); delete(outf); found = ~isempty(pp);
if ~found, wp = [start; goal]; return, end
path = zeros(size(pp,1),2); for q = 1:size(pp,1), path(q,:) = localXY(G,pp(q,1),pp(q,2)); end

if norm(path(1,:) - start) > 0.08, path = [start; path]; else, path(1,:) = start; end
if norm(path(end,:) - goal) > 0.08, path = [path; goal]; else, path(end,:) = goal; end
% keep corners only (helperPlanPathAStar.localKeepCorners)
if size(path,1) >= 3
    dl = diff(path,1,1); keep = [true; abs(dl(1:end-1,1).*dl(2:end,2) - dl(1:end-1,2).*dl(2:end,1)) > 1e-8; true]; path = path(keep,:);
end
wp = path;
end
function [i,j] = localNearestFree(B,i,j)
if ~B(i,j), return, end
for r = 1:12
    for a = -r:r, for b = -r:r
        if max(abs(a),abs(b)) ~= r, continue, end
        ii = i + a; jj = j + b; if ii >= 1 && jj >= 1 && ii <= size(B,1) && jj <= size(B,2) && ~B(ii,jj), i = ii; j = jj; return, end
    end, end
end
i = []; j = [];
end
function wp = localStringPull(G,B,wp)
% any-angle shortcut: from each point jump to the farthest later point that can be reached in a straight line over free cells
if size(wp,1) < 3, return, end
out = wp(1,:); a = 1; n = size(wp,1);
while a < n
    b = n;
    while b > a + 1 && ~localLos(G,B,wp(a,:),wp(b,:)), b = b - 1; end
    out(end+1,:) = wp(b,:); a = b; %#ok<AGROW>
end
wp = out;
end
function ok = localLos(G,B,p,q)
d = norm(q - p); n = max(2,ceil(d/(G.res*0.5))); ok = true;
for u = 0:n
    s = p + (q - p)*u/n; [i,j] = localCell(G,s); if B(i,j), ok = false; return, end
end
end
function wp = localSimplify(wp)
if size(wp,1) < 3, return, end
keep = true(size(wp,1),1); last = 1;
for i = 2:size(wp,1)-1
    a = wp(i,:) - wp(last,:); b = wp(i+1,:) - wp(i,:); ang = abs(atan2(a(1)*b(2) - a(2)*b(1),a(1)*b(1) + a(2)*b(2)));
    if ang < 12*pi/180, keep(i) = false; else, last = i; end
end
wp = wp(keep,:);
end

% ------------------------------------------------------------------------------------------------------------------------------ robot step
function [m,S] = localStep(m,S,C,dt,t,sp,hold,cmd,k)
st = m.state; m.v = 0;
switch st
    case {1,5,7}
        if m.evActive
            if norm(m.evPt - m.pose(1:2)) < 0.35 || t > m.evT, m.evActive = false; m.needPlan = true; m.replanFlag = false; return, end
        end
        if m.needPlan || m.planWait, return, end
        wp = m.wp; N = size(wp,1); carrying = m.carry; rs = 0.45; if carrying, rs = 0.60; end
        while m.wpi < N && norm(wp(m.wpi,:) - m.pose(1:2)) < rs, m.wpi = m.wpi + 1; end
        gl = m.goal; if m.evActive, gl = m.evPt; end
        final = m.wpi == N; target = wp(m.wpi,:); if final, target = gl; end
        gd = norm(gl - m.pose(1:2)); tol = 0.20; if st == 5, tol = 0.04; end
        if gd < tol && final && ~m.evActive
            m = localArrive(m,t); return
        end
        dist = norm(target - m.pose(1:2)); e = localWrap(atan2(target(2) - m.pose(2),target(1) - m.pose(1)) - m.pose(3));
        if strcmp(m.mode,'ROTATE'), if abs(e) < 0.10, m.mode = 'DRIVE'; end, elseif abs(e) > 0.60, m.mode = 'ROTATE'; end
        if strcmp(m.mode,'ROTATE')
            w = max(-1.1,min(1.1,1.6*e)); if abs(w) < 0.15, w = 0.15*sign(e); end; v = 0;
        else
            vmax = C.NavSpeed; if carrying, vmax = C.NavSpeedCarry; end
            v = min(vmax,1.0*gd);
            if ~final && dist < 1.2 && m.wpi < N
                nxt = wp(m.wpi + 1,:); a1 = atan2(target(2) - m.pose(2),target(1) - m.pose(1)); a2 = atan2(nxt(2) - target(2),nxt(1) - target(1)); psi = abs(localWrap(a2 - a1));
                v = v*(1 - 0.6*min(1,psi/(pi/2))*min(1,(1.2 - dist)/0.6));
            end
            v = max(v,0.04)*max(0,cos(e)); w = max(-1.1,min(1.1,1.8*e));
        end
        if hold, v = 0; w = 0; elseif sp < 1, v = v*sp; end
        if st == 5 || st == 1 || st == 7, end
        m.pose(1) = m.pose(1) + v*cos(m.pose(3))*dt; m.pose(2) = m.pose(2) + v*sin(m.pose(3))*dt; m.pose(3) = localWrap(m.pose(3) + w*dt); m.v = v;
        m.legLen = m.legLen + v*dt;
    case 10, if t - m.t0 > 5.0, m.state = 2; m.t0 = t; end
    case 2,  if t - m.t0 > 5.0, m.state = 3; m.t0 = t; end
    case 3,  if t - m.t0 > 9.0, m.state = 4; m.t0 = t; m.carry = true; end
    case 4,  if t - m.t0 > 12.0, m.state = 5; m.goal = m.job.dst.approach; m.t0 = t; m.needPlan = true; m.replanFlag = false; end
    case 6,  if t - m.t0 > 8.0, m.state = 8; m.t0 = t; end
    case 8,  if t - m.t0 > 2.0, m.state = 9; m.status = 'done'; m.carry = false; m.t0 = t; end
    case 9,  if t - m.t0 > 12.0, m.state = 7; m.goal = m.job.src.staging*0; m.t0 = t; m.needPlan = true; m.replanFlag = false; end
end
if m.state == 7 && ~isempty(m.job) && all(m.goal == 0), m.goal = localHome(m); end
end
function h = localHome(m), h = m.trail(1,:); end
function m = localArrive(m,t)
switch m.state
    case 1, m.state = 10; m.t0 = t; m.wp = zeros(0,2); m.mode = 'ROTATE';
    case 5, m.state = 6; m.t0 = t; m.wp = zeros(0,2); m.mode = 'ROTATE';
    case 7, m.state = 0; m.wp = zeros(0,2); m.mode = 'ROTATE'; m.status = 'idle';
end
end
function a = localWrap(a), a = mod(a + pi,2*pi) - pi; end

% ------------------------------------------------------------------------------------------------------------------------------ scripted commands
function localCommand(F,txt)
f = strsplit(strtrim(txt),' ');
switch f{1}
    case 'end', F.finishAll();
    case 'dock', F.dockAdd(str2double(f{2}),true);
    case 'add', F.addBoxAt(f{2},0);
    case 'addst', F.addBoxStation(str2double(f{2}),0);
    case 'order', F.addOrder(str2double(f{2}),str2double(f{3}),true);
    case 'qput', F.queuePutaway(0,'');
    case 'urgent', F.urgentJob(f{2},str2double(f{3}));
    case 'move', F.moveJob(f{2},f{3},str2double(f{4}),0);
    case 'wms', fprintf('%s',F.wmsText(''));
    case 'qputslot', F.queuePutaway(str2double(f{2}),f{3});
    case 'putaway', F.manualTask(str2double(f{2}),'putaway',str2double(f{3}),f{4},0);
    case 'retrieve', F.manualTask(str2double(f{2}),'retrieve',0,f{4},str2double(f{5}));
end
end
