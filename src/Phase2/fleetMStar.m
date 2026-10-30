function [paths,info] = fleetMStar(F,res,x0,starts,goals,w,dsafe,lim)
%FLEETMSTAR  M* multirobot path planning (Wagner & Choset, "Subdimensional expansion for multirobot path planning", AIJ 2015) on an 8-connected grid.
%
%   Every robot has an INDIVIDUAL OPTIMAL POLICY (shortest path to its own goal, a cost-to-go field).  The search runs in the JOINT configuration space of all
%   robots, but a robot that is not (yet) in a collision set may only follow its policy (one successor); only the robots of a collision set get all their
%   neighbours (subdimensional expansion).  When a successor configuration collides, the colliding robots are added to the collision set of the node and
%   the set is propagated back to all predecessors (back-propagation), which are re-opened.  If no robots ever collide, the result is the set of
%   independent shortest paths; robots are coupled only where and as long as their paths really interfere.  Search = A* on the sum of weighted costs
%   (admissible heuristic = sum of weighted cost-to-go), so the coupled solution is optimal for the weighted sum of path costs.
%
%   F      ny x nx x n logical     F(:,:,i) = true where the CENTRE of robot i may be (already inflated by the robot size)
%   res    cell size [m]           x0: cell centre of column j = (j-0.5)*res - x0, of row i = (i-0.5)*res
%   starts, goals  n x 2 [m]       w  n x 1 cost weights (higher = this robot's path is kept shorter, the others give way)
%   dsafe  n x n [m]               centre distance the robots must keep while they move (continuous check along each step)
%   lim    struct (optional)       maxNodes, maxGroup, tMax [s], stayCost [m]
%   paths{i}  Kx2 [m]              one row per synchronous step (robot waits = repeated rows); empty when ok = false
%   info                           ok, reason, nodes, expanded, maxGroup (largest collision set), coupled, steps, time, cost, groups (final collision sets seen)
n = size(starts,1); [ny,nx,~] = size(F);
info = struct('ok',false,'reason','','nodes',0,'expanded',0,'maxGroup',1,'coupled',false,'steps',0,'time',0,'cost',0,'groups',{{}},'deviated',false(size(starts,1),1),'cells',{{}});
paths = cell(n,1); t0 = tic;
if nargin < 8 || isempty(lim), lim = struct(); end
if ~isfield(lim,'maxNodes'), lim.maxNodes = 6000; end
if ~isfield(lim,'maxGroup'), lim.maxGroup = 4; end
if ~isfield(lim,'tMax'), lim.tMax = 8; end
if ~isfield(lim,'stayCost'), lim.stayCost = res; end
w = w(:);
% ---- start / goal cells, individual policies
sc = zeros(n,1); gc = zeros(n,1); D = cell(n,1); NXT = cell(n,1);
for i = 1:n
    sc(i) = localSnap(F(:,:,i),starts(i,:),res,x0); gc(i) = localSnap(F(:,:,i),goals(i,:),res,x0);
    if sc(i) == 0 || gc(i) == 0, info.reason = sprintf('robot %d: no free cell near its start / goal',i); info.time = toc(t0); return, end
    [D{i},NXT{i}] = localField(F(:,:,i),gc(i),res);
    if ~isfinite(D{i}(sc(i))), info.reason = sprintf('robot %d: goal not reachable',i); info.time = toc(t0); return, end
end
% cell -> coordinates
[ci,cj] = ind2sub([ny nx],(1:ny*nx).'); XY = [(cj - 0.5)*res - x0, (ci - 0.5)*res];
% separation: a pair that starts closer than dsafe may not get closer than it already is (it is already inside the bubble)
d0 = zeros(n); for a = 1:n, for b = 1:n, d0(a,b) = norm(XY(sc(a),:) - XY(sc(b),:)); end, end
dsafe = min(dsafe,0.92*d0); dsafe(1:n+1:end) = 0;
[pa,pb] = find(triu(true(n),1)); npair = numel(pa); dpair = dsafe(sub2ind([n n],pa,pb));
% ---- node storage
cap = lim.maxNodes; CFG = zeros(cap,n); G = inf(cap,1); PAR = zeros(cap,1); PART = zeros(cap,n); CLOSED = false(cap,1); BACK = cell(cap,1);
cnt = 1;
CFG(1,:) = sc.'; G(1) = 0; BACK{1} = zeros(1,0);
HTN = 65536; HT = zeros(HTN,1); hw = (1315423911 - 2654435761*(0:n-1)); hw = mod(abs(hw),1048573) + 17; hw = hw(:);
HT(mod(mod(CFG(1,:)*hw,2^31),HTN) + 1) = 1;
heap = struct('k',zeros(1024,1),'id',zeros(1024,1),'n',0);
h1 = localH(CFG(1,:),D,w); heap = localPush(heap,h1,1);
goalCfg = gc.'; found = 0;
while heap.n > 0
    [heap,v] = localPop(heap);
    if CLOSED(v), continue, end          % stale heap entry (the node was closed already with the same or better value)
    CLOSED(v) = true;
    if all(CFG(v,:) == goalCfg), found = v; break, end
    if toc(t0) > lim.tMax, info.reason = 'time limit'; break, end
    info.expanded = info.expanded + 1;
    cfg = CFG(v,:); lab = PART(v,:);
    % ---- limited neighbour set: followers take ONE policy step, the robots of a collision set take all their moves
    nbr = cfg; stay = (cfg == goalCfg);
    for i = 1:n
        if lab(i) == 0 && ~stay(i), nbr(i) = NXT{i}(cfg(i)); end
    end
    mv = find(lab > 0);
    if isempty(mv)
        cand = nbr;
    else
        L = cell(1,numel(mv));
        for q = 1:numel(mv), L{q} = localMoves(F(:,:,mv(q)),cfg(mv(q)),D{mv(q)},ny,nx); end
        cand = localProduct(nbr,mv,L);
    end
    K = size(cand,1);
    % ---- collisions along every step (continuous, relative motion)
    P0 = XY(cfg,:); P1 = zeros(K,2,n); for i = 1:n, P1(:,:,i) = XY(cand(:,i),:); end
    coll = false(K,npair);
    for q = 1:npair
        a = pa(q); b = pb(q); r0 = P0(a,:) - P0(b,:); r1 = P1(:,:,a) - P1(:,:,b); dd = r1 - r0; den = sum(dd.^2,2);
        al = zeros(K,1); nz = den > 1e-12; al(nz) = min(1,max(0,-(dd(nz,:)*r0.')./den(nz)));
        dm = sqrt(sum((r0 + al.*dd).^2,2)); coll(:,q) = dm < dpair(q);
    end
    bad = any(coll,2);
    if any(bad)
        edge = any(coll(bad,:),1); [lab2,ch] = localMerge(PART(v,:),pa(edge),pb(edge),n);
        if ch
            [PART,CLOSED,heap,BACK] = localBack(v,lab2,PART,CLOSED,heap,BACK,CFG,G,D,w,n);
            info.maxGroup = max(info.maxGroup,max(accumarray(lab2(lab2>0).',1))); info.coupled = true;
            if info.maxGroup > lim.maxGroup, info.reason = sprintf('collision set larger than %d robots',lim.maxGroup); break, end
            info.groups{end+1} = find(lab2 > 0); %#ok<AGROW>
        end
    end
    cand = cand(~bad,:); K = size(cand,1);
    if K > 0
        % ---- cost of every step (weighted path length; waiting at the goal is free), vectorised
        Pn = zeros(K,2,n); dist = zeros(K,n);
        for i = 1:n, Pn(:,:,i) = XY(cand(:,i),:); dist(:,i) = sqrt(sum((Pn(:,:,i) - P0(i,:)).^2,2)); end
        moved = cand ~= cfg; atGoal = repmat(cfg == goalCfg,K,1);
        cst = (dist.*moved + lim.stayCost*(~moved & ~atGoal))*w;
        % ---- look the successors up in the hash table (linear probing, verified against the stored configuration)
        pos = mod(mod(cand*hw,2^31),HTN) + 1; u = zeros(K,1); todo = true(K,1);
        while any(todo)
            ids = HT(pos(todo)); tj = find(todo);
            hit = false(size(tj)); emp = ids == 0;
            if any(~emp), nz = find(~emp); hit(nz) = all(CFG(ids(nz),:) == cand(tj(nz),:),2); end
            u(tj(hit)) = ids(hit); todo(tj(hit | emp)) = false;
            cont = ~hit & ~emp; pos(tj(cont)) = mod(pos(tj(cont)),HTN) + 1;
            % the ones that hit an empty slot are new nodes: remember their slot, insert below
            newIdx = tj(emp);
            for q = 1:numel(newIdx)
                kq = newIdx(q);
                if cnt >= cap, info.reason = 'node limit'; break, end
                cnt = cnt + 1; CFG(cnt,:) = cand(kq,:); BACK{cnt} = zeros(1,0); HT(pos(kq)) = cnt; u(kq) = cnt;
            end
            if ~isempty(info.reason), break, end
        end
        if ~isempty(info.reason), break, end
        for q = 1:K, BACK{u(q)}(end+1) = v; end
        known = find(any(PART(u,:) > 0,2));
        for q = known.'
            [lab3,ch3] = localMergeLab(PART(v,:),PART(u(q),:),n);
            if ch3
                [PART,CLOSED,heap,BACK] = localBack(v,lab3,PART,CLOSED,heap,BACK,CFG,G,D,w,n);
                info.maxGroup = max(info.maxGroup,max(accumarray(lab3(lab3>0).',1))); info.coupled = true;
            end
        end
        gnew = G(v) + cst; better = find(gnew < G(u) - 1e-9);
        for q = better.'
            uu = u(q); G(uu) = gnew(q); PAR(uu) = v; CLOSED(uu) = false; heap = localPush(heap,G(uu) + localH(CFG(uu,:),D,w),uu);
        end
    end
    if ~isempty(info.reason), break, end
    if info.maxGroup > lim.maxGroup, info.reason = sprintf('collision set larger than %d robots',lim.maxGroup); break, end
end
info.nodes = cnt; info.time = toc(t0);
if found == 0
    if isempty(info.reason), info.reason = 'no joint path'; end
    return
end
chain = found; while PAR(chain(1)) > 0, chain = [PAR(chain(1)) chain]; end %#ok<AGROW>
for i = 1:n
    paths{i} = XY(CFG(chain,i),:);
    pc = CFG(chain,i); pc = pc([true; diff(pc) ~= 0]); pol = sc(i); c = sc(i);          % robot i deviates from its individual optimal policy when its cell sequence differs (waiting does not count)
    while c ~= gc(i) && numel(pol) < ny*nx, c = NXT{i}(c); pol(end+1,1) = c; end %#ok<AGROW>
    info.deviated(i) = ~isequal(pc(:),pol(:));
end
info.ok = true; info.steps = numel(chain) - 1; info.cost = G(found); info.reason = 'ok';
end

% ------------------------------------------------------------------------------------------------------------------------------------------------
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

function [D,NXT] = localField(Fi,g,res)
% cost-to-go field (8-connected, no corner cutting) by wavefront relaxation; NXT = best neighbour cell of every cell (the individual optimal policy)
[ny,nx] = size(Fi); D = inf(ny,nx); D(g) = 0;
mv = [1 0 res; -1 0 res; 0 1 res; 0 -1 res; 1 1 res*sqrt(2); 1 -1 res*sqrt(2); -1 1 res*sqrt(2); -1 -1 res*sqrt(2)];
Fp = false(ny+2,nx+2); Fp(2:end-1,2:end-1) = Fi;
ok = cell(8,1);
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
Pd = inf(ny+2,nx+2); Pd(2:end-1,2:end-1) = D; best = inf(ny,nx); NXT = reshape(int32(1:ny*nx),ny,nx);
[II,JJ] = ndgrid(1:ny,1:nx);
for m = 1:8
    di = mv(m,1); dj = mv(m,2); cand = Pd(2+di:ny+1+di,2+dj:nx+1+dj) + mv(m,3); cand(~ok{m}) = inf;
    better = cand < best - 1e-12; best(better) = cand(better);
    tgt = (JJ + dj - 1)*ny + (II + di); NXT(better) = int32(tgt(better));
end
NXT(g) = int32(g); NXT(~isfinite(D)) = int32(find(~isfinite(D)));
NXT = double(NXT(:));
end

function L = localMoves(Fi,c,Di,ny,nx)
% stay + 8 neighbours of cell c for a robot of a collision set (no corner cutting, only cells from which the goal is reachable)
[i,j] = ind2sub([ny nx],c); L = c;
for di = -1:1
    for dj = -1:1
        if di == 0 && dj == 0, continue, end
        ii = i + di; jj = j + dj;
        if ii < 1 || jj < 1 || ii > ny || jj > nx || ~Fi(ii,jj) || ~isfinite(Di(ii,jj)), continue, end
        if di ~= 0 && dj ~= 0 && ~(Fi(ii,j) && Fi(i,jj)), continue, end
        L(end+1) = sub2ind([ny nx],ii,jj); %#ok<AGROW>
    end
end
end

function cand = localProduct(base,mv,L)
cand = base; nm = numel(mv);
for q = 1:nm
    lq = L{q}(:); m = size(cand,1); cand = repmat(cand,numel(lq),1); cand(:,mv(q)) = kron(lq,ones(m,1));
end
end

function k = localKey(cfg), k = char(typecast(int32(cfg(:).'),'uint8')); end

function h = localH(cfg,D,w)
h = 0; for i = 1:numel(cfg), h = h + w(i)*D{i}(cfg(i)); end
end

function [lab,ch] = localMerge(lab0,ea,eb,n)
% collision sets as component labels (0 = robot not in any set): merge the pair edges (ea,eb) into the sets lab0
A = false(n); for i = 1:n, for j = 1:n, if lab0(i) > 0 && lab0(i) == lab0(j), A(i,j) = true; end, end, end
for q = 1:numel(ea), A(ea(q),eb(q)) = true; A(eb(q),ea(q)) = true; end
lab = localComp(A,n); ch = ~isequal(lab,lab0);
end
function [lab,ch] = localMergeLab(lab0,lab1,n)
A = false(n); for i = 1:n, for j = 1:n, if i ~= j && ((lab0(i) > 0 && lab0(i) == lab0(j)) || (lab1(i) > 0 && lab1(i) == lab1(j))), A(i,j) = true; end, end, end
lab = localComp(A,n); ch = ~isequal(lab,lab0);
end
function lab = localComp(A,n)
lab = zeros(1,n); nl = 0; seen = false(1,n);
for s = 1:n
    if seen(s), continue, end
    comp = s; seen(s) = true; q = s;
    while ~isempty(q), u = q(1); q(1) = []; nb = find(A(u,:) & ~seen); seen(nb) = true; comp = [comp nb]; q = [q nb]; end %#ok<AGROW>
    if numel(comp) >= 2, nl = nl + 1; lab(comp) = nl; end
end
end

function [PART,CLOSED,heap,BACK] = localBack(v,lab,PART,CLOSED,heap,BACK,CFG,G,D,w,n)
% back-propagation: node v gets the collision set lab; every predecessor of v receives it as well; a closed node whose set grew is re-opened
stack = {v,lab};
while ~isempty(stack)
    u = stack{1}; add = stack{2}; stack(1:2) = [];
    [nl,ch] = localMergeLab(PART(u,:),add,n);
    if ~ch, continue, end
    PART(u,:) = nl;
    if CLOSED(u), CLOSED(u) = false; heap = localPush(heap,G(u) + localH(CFG(u,:),D,w),u); end       % re-opened with its own g + h
    pr = BACK{u};
    for q = 1:numel(pr), stack(end+1:end+2) = {pr(q),nl}; end %#ok<AGROW>
end
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
