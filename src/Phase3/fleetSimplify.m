function wp = fleetSimplify(wp,tol)
%FLEETSIMPLIFY  Douglas-Peucker: drop waypoints that lie within tol [m] of the straight line between their neighbours (never cuts a detour around a robot).
if nargin < 2, tol = 0.12; end
n = size(wp,1); if n < 3, return, end
keep = false(n,1); keep([1 n]) = true; stack = [1 n];
while ~isempty(stack)
    a = stack(end,1); b = stack(end,2); stack(end,:) = [];
    if b <= a + 1, continue, end
    d = wp(b,:) - wp(a,:); L = norm(d);
    if L < 1e-9, dist = sqrt(sum((wp(a+1:b-1,:) - wp(a,:)).^2,2)); else, dist = abs(d(1)*(wp(a+1:b-1,2) - wp(a,2)) - d(2)*(wp(a+1:b-1,1) - wp(a,1)))/L; end
    [m,ix] = max(dist);
    if m > tol, c = a + ix; keep(c) = true; stack(end+1,:) = [a c]; stack(end+1,:) = [c b]; end %#ok<AGROW>
end
wp = wp(keep,:);
end
