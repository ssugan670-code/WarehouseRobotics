function [wp,info] = fleetPlanRoute(planFcn,start,goal,v,R,P,myCarry)
%FLEETPLANROUTE  Plan-time route negotiation of one robot (shared by the controller and by the headless test world).
%   The robot plans the SHORTEST route first, then checks it against the committed routes of all robots that planned earlier
%   (R rows: [x y eta id carry]; eta = seconds from now at which that robot reaches the point).  The check is done in SPACE AND TIME:
%   a conflict exists only where both robots are near the same point at (nearly) the same time.  If the other robot has passed before
%   I arrive, or arrives long after I left, there is no conflict and the shortest route is kept.  Otherwise the conflicting part of the
%   other route is blocked and a second route is planned; it is accepted only when it is not much longer (no big loops).  If no
%   acceptable alternative exists the shortest route is kept and the traffic layer adapts the SPEED (slow down early, never stop late).
%   planFcn(blockPts,blockRad) returns Nx2 waypoints or [].
if nargin < 7, myCarry = false; end
info = struct('conflicts',0,'stretch',1,'alt',false,'tries',0,'len',0);
wp = planFcn(zeros(0,2),0);
if isempty(wp), return, end
L0 = pathLen(wp); info.len = L0;
if isempty(R) || size(wp,1) < 2, return, end
[bp,nc,arm] = findConflicts(wp,v,R,P,myCarry); info.conflicts = nc;
if nc == 0, return, end
allBp = zeros(0,2); best = wp;
for it = 1:P.maxTries
    allBp = [allBp; bp]; info.tries = it; %#ok<AGROW>
    w1 = planFcn(allBp,P.blockRad + P.carryExtra*double(myCarry) + P.armExtra*double(arm));      % same clearance the conflict test asks for (a working robot's arm counts)
    if isempty(w1) || size(w1,1) < 2, break, end
    L1 = pathLen(w1);
    if L1 > P.maxStretch*L0 + P.maxExtra, break, end
    [bp1,nc1,arm1] = findConflicts(w1,v,R,P,myCarry); arm = arm || arm1;
    if nc1 < nc || nc1 == 0
        best = w1; nc = nc1; bp = bp1; info.alt = true; info.stretch = L1/max(L0,0.1); info.len = L1; info.conflicts = nc1;
        if nc1 == 0, break, end
    else
        bp = bp1;
    end
end
wp = best;
end

function L = pathLen(w)
L = sum(sqrt(sum(diff(w,1,1).^2,2)));
end

function [bp,nc,arm] = findConflicts(wp,v,R,P,myCarry)
% sample my route every 0.5 m, time = distance / speed; compare with the other robots' time-stamped points
seg = sqrt(sum(diff(wp,1,1).^2,2)); s = [0; cumsum(seg)]; n = max(2,ceil(s(end)/0.5)+1); ss = linspace(0,s(end),n).';
Q = [interp1(s,wp(:,1),ss) interp1(s,wp(:,2),ss)]; tq = ss/max(v,0.1);
bp = zeros(0,2); nc = 0; arm = false;
Rc = P.Rc + P.carryExtra*(double(myCarry) + R(:,5).') + P.armExtra*R(:,6).';           % separation needed per other point
for a = 1:n
    if ss(a) < 1.2, continue, end                                 % the start itself cannot be moved
    d = sqrt((R(:,1).' - Q(a,1)).^2 + (R(:,2).' - Q(a,2)).^2);
    hit = d < Rc & abs(tq(a) - R(:,3).') < P.tWin;
    if any(hit), nc = nc + 1; bp = [bp; R(hit(:),1:2)]; arm = arm || any(R(hit(:),6) > 0); end %#ok<AGROW>
end
if ~isempty(bp), bp = unique(round(bp*20)/20,'rows'); end
end
