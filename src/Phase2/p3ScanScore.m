function s = p3ScanScore(Ginfl,res,off,pose,ranges,angles)
%P3SCANSCORE  Fraction of the scan end points that land on (slightly inflated) occupied map cells when the robot is at pose [x y yaw] (world).
%   1 = the scan fits the map, ~0.1-0.3 = it does not.  Used to pick the lidar frame convention and to check the sensor at start-up.
k = isfinite(ranges); if nnz(k) < 5, s = 0; return, end
a = pose(3) + angles(k); xy = [pose(1) + ranges(k).*cos(a); pose(2) + ranges(k).*sin(a)].';
s = mean(p3GridHit(Ginfl,res,off,xy));
end
