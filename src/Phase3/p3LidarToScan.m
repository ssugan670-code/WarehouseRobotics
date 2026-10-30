function [ranges,angles] = p3LidarToScan(pc,nBeams,minR,maxR,mirror,yawOff,sensOff)
%P3LIDARTOSCAN  Unreal lidar point cloud (any of M-by-N-by-3, n-by-3, 3-by-n; sensor frame) -> 2-D scan with nBeams equally spaced beams over 360 deg.
%   Points are flattened to the sensor's horizontal plane (all channels, nearest return per beam wins), invalid points (NaN / Inf / exactly 0,0,0) are
%   dropped.  mirror = true flips y (left- vs right-handed sensor frame), yawOff rotates the scan [rad] - both are chosen by Phase3Sense at start-up by
%   matching the first scans against the map (the Unreal sensor frame convention is not assumed).
if nargin<5, mirror = false; end
if nargin<6, yawOff = 0; end
if nargin<7, sensOff = [0 0]; end      % where the sensor sits on the robot [x y] (robot frame): the scan is expressed around the robot centre
angles = linspace(-pi,pi,nBeams+1); angles(end) = []; ranges = inf(1,nBeams);
P = double(pc);
if ndims(P) == 3, P = reshape(P,[],size(P,3)); end
if size(P,2) ~= 3 && size(P,1) == 3, P = P.'; end
if size(P,2) < 3 || isempty(P), return, end
P = P(:,1:3); bad0 = ~all(isfinite(P),2) | all(P == 0,2); P = P(~bad0,:);
if isempty(P), return, end
z0 = median(P(:,3)); P = P(abs(P(:,3) - z0) <= 0.25,:);      % floor / ceiling returns of tilted channels are dropped (relative to the scan plane, so a frame with an offset z still works)
if isempty(P), return, end
y = P(:,2); if mirror, y = -y; end
a = atan2(y,P(:,1)) + yawOff; a = atan2(sin(a),cos(a)); r = hypot(P(:,1),y);
k = r >= minR & r <= maxR; a = a(k); r = r(k);
if any(sensOff ~= 0), xr = r.*cos(a) + sensOff(1); yr = r.*sin(a) + sensOff(2); a = atan2(yr,xr); r = hypot(xr,yr); end      % sensor is not at the robot centre: express the hits around the centre
b = min(nBeams,max(1,round((a + pi)/(2*pi)*nBeams) + 1)); b(b > nBeams) = 1;
for i = 1:numel(r), if r(i) < ranges(b(i)), ranges(b(i)) = r(i); end, end
end
