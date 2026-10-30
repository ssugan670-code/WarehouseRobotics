function r = p3RaycastScan(G,res,off,pose,angles,maxRange,discs)
%P3RAYCASTSCAN  Simulated 2-D lidar: ray-march through the static grid (and optional discs = [x y radius] of other robots).
%   Used (a) as the lidar when the Unreal lidar block is not available and (b) as the reference in the p3sense checks.
%   r(i) = range of beam i (angles are relative to the robot heading), Inf = no return within maxRange.
step = 1/res; n = numel(angles); r = inf(1,n);
if nargin<7, discs = zeros(0,3); end
s = (step:step:maxRange).'; ns = numel(s);
for i = 1:n
    a = pose(3) + angles(i); xy = [pose(1) + s*cos(a), pose(2) + s*sin(a)];
    hit = find(p3GridHit(G,res,off,xy),1,'first');
    if ~isempty(hit), r(i) = s(hit); end
    for j = 1:size(discs,1)                          % ray / circle intersection
        d = discs(j,1:2) - pose(1:2); t = d(1)*cos(a) + d(2)*sin(a);
        if t <= 0, continue, end
        h2 = discs(j,3)^2 - (d(1)^2 + d(2)^2 - t^2);
        if h2 >= 0, tt = t - sqrt(h2); if tt > 0 && tt < r(i) && tt <= maxRange, r(i) = tt; end, end
    end
end
end
