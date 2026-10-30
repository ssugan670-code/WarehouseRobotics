function [stop,why,dMin] = p3SafetyField(est,robXY,unkXY,carry,p)
%P3SAFETYFIELD  Last-resort protective field of the lidar layer (the planner keeps the robots far apart; this only fires when that failed).
%   est    [x y yaw] estimated pose of this robot              robXY  n-by-2 fused tracks that are other robots (centres)
%   unkXY  m-by-2 lidar clusters that are NOT in the map and NOT a robot (unexpected obstacle)         carry  true while a carton is carried
%   p      .coneDeg (35)  .dRobot (0.85 centre distance = chassis contact 0.68 + margin)  .dUnk (1.45)  .rSelf (1.05 own arm / carried box, ignored)
%   stop   true = hold the base (the controller multiplies its drive command by 0)
stop = false; why = ''; dMin = inf;
if ~isfield(p,'coneDeg'), p.coneDeg = 35; end
for q = 1:2
    if q == 1, A = robXY; lim = p.dRobot; tag = 'robot'; else, A = unkXY; lim = p.dUnk; tag = 'obstacle'; end
    for i = 1:size(A,1)
        d = A(i,:) - est(1:2); r = hypot(d(1),d(2)); b = abs(atan2(sin(atan2(d(2),d(1))-est(3)),cos(atan2(d(2),d(1))-est(3))));
        if q == 2 && r < p.rSelf, continue, end
        if r < dMin, dMin = r; end
        if r < lim && b <= deg2rad(p.coneDeg), stop = true; why = tag; end
    end
end
end
