function [paths,info] = fleetSTPlan(F,res,x0,starts,goals,order,dsafe,statics,lim)
%FLEETSTPLAN  Time-stamped multirobot planning WITH WAITING: prioritised space-time A* on the same coarse grid as fleetMStar.
%
%   Used as the timed second level of the M* coordination: when the joint M* search gives up (node / time limit) or when a robot has to decide between
%   waiting for a working robot and driving around it.  Robots are planned one after the other in priority order (highest first).  A robot may move to any of
%   its 8 neighbour cells or WAIT, one step = lim.dt seconds; it must keep the centre distance dsafe(i,k) to every robot planned before it (continuous
%   check along every step) and the distance statics(s).req(i) to every static disc during its time window [t0,t1) (a robot that is working at a rack is
%   an obstacle until it is finished - the planner decides whether waiting for that or a detour is faster, the cost of both is time).
%   A pair that starts closer than its limit may not get closer than it already is (it is already inside the bubble).
%
%   F        ny x nx x n logical   F(:,:,i) = true where the CENTRE of robot i may be (inflated by the robot size)
%   res, x0  grid cell size, x offset (cell centre of column j = (j-0.5)*res - x0, row i = (i-0.5)*res)
%   starts, goals  n x 2 [m]
%   order    1 x n                 planning order, highest priority first
%   dsafe    n x n                 distance to keep between two moving robots
%   statics  struct array          .xy [1x2]  .req [n x 1]  .t0 [steps]  .t1 [steps, inf = for ever]
%   lim      dt [s] 0.5, maxSteps 160, maxNodes 60000 (per robot), tMax [s] 2.5 (all robots together), stayCost 1.0
%   paths{i} (K+1) x 2             position after every step (waiting = repeated rows), all robots padded to the same length; [] when robot i failed
%   info     ok (all robots planned), okR (per robot), reason, nodes, time, steps, dt
n = size(starts,1); [ny,nx,~] = size(F);
info = struct('ok',false,'okR',false(n,1),'reason','','nodes',0,'time',0,'steps',0,'dt',0.5);
paths = cell(n,1); t0c = tic;
if nargin < 8 || isempty(statics), statics = struct('xy',{},'req',{},'t0',{},'t1',{}); end
if nargin < 9 || isempty(lim), lim = struct(); end
if ~isfield(lim,'dt'), lim.dt = 0.5; end
if ~isfield(lim,'maxSteps'), lim.maxSteps = 160; end
if ~isfield(lim,'maxNodes'), lim.maxNodes = 60000; end
if ~isfield(lim,'tMax'), lim.tMax = 2.5; end
if ~isfield(lim,'stayCost'), lim.stayCost = 1.0; end
if ~isfield(lim,'eps'), lim.eps = 2.5; end
info.dt = lim.dt; T = lim.maxSteps;
[ci,cj] = ind2sub([ny nx],(1:ny*nx).'); XY = [(cj - 0.5)*res - x0, (ci - 0.5)*res];
mvs = [0 0; 1 0; -1 0; 0 1; 0 -1; 1 1; 1 -1; -1 1; -1 -1];
RES = cell(n,1);               % reserved positions of the robots planned so far: (T+1) x 2
placed = false(n,1);
hstep = 1/(res*sqrt(2));
wA = lim.eps;     % weighted A* (bounded sub-optimal, much faster): the route is at most eps times the optimum
for oi = 1:numel(order)
    i = order(oi);
    if toc(t0c) > lim.tMax, info.reason = 'time limit'; break, end
    Fi = F(:,:,i);
    sc = localSnap(Fi,starts(i,:),res,x0); gc = localSnap(Fi,goals(i,:),res,x0);
    if sc == 0 || gc == 0, continue, end
    D = localField(Fi,gc,res); if ~isfinite(D(sc)), continue, end
    % limits for this robot (a pair that starts inside the bubble may not get closer)
    prev = find(placed).'; np = numel(prev);
    dlim = zeros(1,np); for q = 1:np, k = prev(q); dlim(q) = min(dsafe(i,k),0.92*norm(XY(sc,:) - RES{k}(1,:))); end
    ns = numel(statics); slim = zeros(1,ns);
    for q = 1:ns, slim(q) = min(statics(q).req(i),0.92*norm(XY(sc,:) - statics(q).xy)); end
    % the goal may only be taken when nobody planned before passes through it later
    lastBad = 0;
    for q = 1:np, k = prev(q); dd = sqrt(sum((RES{k} - XY(gc,:)).^2,2)); b = find(dd < dlim(q),1,'last'); if ~isempty(b), lastBad = max(lastBad,b - 1); end, end
    for q = 1:ns
        if statics(q).t1 > lastBad, if norm(XY(gc,:) - statics(q).xy) < slim(q), lastBad = max(lastBad,min(T,statics(q).t1)); end, end
    end
    % ---- A* in (cell, step); g = step (time), uniform cost per step
    cap = lim.maxNodes; NC = zeros(cap,1,'int32'); NT = zeros(cap,1,'int32'); NP = zeros(cap,1,'int32'); cnt = 1; NC(1) = sc; NT(1) = 0; NP(1) = 0;
    seen = zeros(ny*nx,T + 1,'int32'); seen(sc,1) = 1;
    heap = struct('k',zeros(1024,1),'id',zeros(1024,1),'n',0); heap = localPush(heap,wA*hstep*D(sc),1);
    foundN = 0; why = '';
    while heap.n > 0
        [heap,v] = localPop(heap); c = double(NC(v)); t = double(NT(v));
        if c == gc && t >= lastBad, foundN = v; break, end
        if t >= T, continue, end
        if mod(cnt,64) == 0 && toc(t0c) > lim.tMax, why = 'time limit'; break, end
        [r0,c0] = ind2sub([ny nx],c);
        rr = r0 + mvs(:,1); cc = c0 + mvs(:,2);
        ok = rr >= 1 & rr <= ny & cc >= 1 & cc <= nx; ok(1) = true;
        idx = zeros(9,1); idx(ok) = (cc(ok) - 1)*ny + rr(ok);
        ok(ok) = Fi(idx(ok)) & isfinite(D(idx(ok))); ok(1) = true;
        % no corner cutting on diagonals
        for m = 6:9
            if ok(m) && ~(Fi(r0 + mvs(m,1),c0) && Fi(r0,c0 + mvs(m,2))), ok(m) = false; end
        end
        cand = idx(ok); P1 = XY(cand,:); P0 = XY(c,:);
        good = true(numel(cand),1);
        for q = 1:np
            k = prev(q); a = RES{k}(t + 1,:); b = RES{k}(t + 2,:);
            r0v = P0 - a; r1 = P1 - b; dd = r1 - r0v; den = sum(dd.^2,2); al = zeros(numel(cand),1); nz = den > 1e-12;
            al(nz) = min(1,max(0,-(dd(nz,:)*r0v.')./den(nz)));
            dm = sqrt(sum((r0v + al.*dd).^2,2)); good = good & dm >= dlim(q);
        end
        for q = 1:ns
            if t >= statics(q).t1 || t + 1 < statics(q).t0, continue, end
            dm = min(sqrt(sum((P1 - statics(q).xy).^2,2)),norm(P0 - statics(q).xy)*ones(numel(cand),1));
            mid = sqrt(sum(((P1 + P0)/2 - statics(q).xy).^2,2)); dm = min(dm,mid);
            good = good & dm >= slim(q);
        end
        cand = cand(good);
        for q = 1:numel(cand)
            u = cand(q);
            if seen(u,t + 2) ~= 0, continue, end
            if cnt >= cap, why = 'node limit'; break, end
            cnt = cnt + 1; NC(cnt) = u; NT(cnt) = t + 1; NP(cnt) = v; seen(u,t + 2) = cnt;
            heap = localPush(heap,(t + 1) + wA*hstep*D(u),cnt);
        end
        if ~isempty(why), break, end
    end
    info.nodes = info.nodes + cnt;
    if foundN == 0
        if isempty(info.reason), if isempty(why), info.reason = sprintf('robot %d: no timed path',i); else, info.reason = why; end, end
        continue
    end
    chain = foundN; while NP(chain(1)) > 0, chain = [double(NP(chain(1))) chain]; end %#ok<AGROW>
    cells = double(NC(chain)); Pi = XY(cells,:);
    R = [Pi; repmat(Pi(end,:),T + 2 - size(Pi,1),1)];
    RES{i} = R; placed(i) = true; paths{i} = Pi; info.okR(i) = true;
end
info.time = toc(t0c);
info.ok = all(info.okR);
if ~any(info.okR), if isempty(info.reason), info.reason = 'no timed path'; end, return, end
K = max(cellfun(@(p) size(p,1),paths(info.okR)));
for i = 1:n, if info.okR(i), paths{i} = [paths{i}; repmat(paths{i}(end,:),K - size(paths{i},1),1)]; else, paths{i} = []; end, end
info.steps = K - 1;
if info.ok, info.reason = 'ok'; end
end

function c = localSnap(Fi,xy,res,x0)
[ny,nx] = size(Fi); j = min(nx,max(1,floor((xy(1) + x0)/res) + 1)); i = min(ny,max(1,floor(xy(2)/res) + 1));
if Fi(i,j), c = sub2ind([ny nx],i,j); return, end
c = 0;
for r = 1:8
    best = inf;
    for a = -r:r
        for b = -r:r
            if max(abs(a),abs(b)) ~= r, continue, end
            ii = i + a; jj = j + b;
            if ii >= 1 && jj >= 1 && ii <= ny && jj <= nx && Fi(ii,jj) && hypot(a,b) < best, best = hypot(a,b); c = sub2ind([ny nx],ii,jj); end
        end
    end
    if c > 0, return, end
end
end

function D = localField(Fi,g,res)
[ny,nx] = size(Fi); D = inf(ny,nx); D(g) = 0;
mv = [1 0 res; -1 0 res; 0 1 res; 0 -1 res; 1 1 res*sqrt(2); 1 -1 res*sqrt(2); -1 1 res*sqrt(2); -1 -1 res*sqrt(2)];
Fp = false(ny+2,nx+2); Fp(2:end-1,2:end-1) = Fi; ok = cell(8,1);
for m = 1:8
    di = mv(m,1); dj = mv(m,2); okm = Fp(2+di:ny+1+di,2+dj:nx+1+dj);
    if di ~= 0 && dj ~= 0, okm = okm & Fp(2+di:ny+1+di,2:nx+1) & Fp(2:ny+1,2+dj:nx+1+dj); end
    ok{m} = okm & Fi;
end
for it = 1:(2*(ny + nx))
    Pd = inf(ny+2,nx+2); Pd(2:end-1,2:end-1) = D; Dn = D;
    for m = 1:8
        di = mv(m,1); dj = mv(m,2); cand = Pd(2+di:ny+1+di,2+dj:nx+1+dj) + mv(m,3); cand(~ok{m}) = inf; Dn = min(Dn,cand);
    end
    if isequal(Dn,D), break, end
    D = Dn;
end
D = D(:);
end

function heap = localPush(heap,k,id)
heap.n = heap.n + 1; if heap.n > numel(heap.k), heap.k(end+numel(heap.k)) = 0; heap.id(end+numel(heap.id)) = 0; end
i = heap.n; heap.k(i) = k; heap.id(i) = id;
while i > 1
    p = floor(i/2); if heap.k(p) <= heap.k(i), break, end
    tk = heap.k(p); ti = heap.id(p); heap.k(p) = heap.k(i); heap.id(p) = heap.id(i); heap.k(i) = tk; heap.id(i) = ti; i = p;
end
end
function [heap,id] = localPop(heap)
id = heap.id(1); heap.k(1) = heap.k(heap.n); heap.id(1) = heap.id(heap.n); heap.n = heap.n - 1; i = 1;
while true
    l = 2*i; r = l + 1; s = i;
    if l <= heap.n && heap.k(l) < heap.k(s), s = l; end
    if r <= heap.n && heap.k(r) < heap.k(s), s = r; end
    if s == i, break, end
    tk = heap.k(s); ti = heap.id(s); heap.k(s) = heap.k(i); heap.id(s) = heap.id(i); heap.k(i) = tk; heap.id(i) = ti; i = s;
end
end
