function [uv,zf,cam] = p3CamProject(P,lift,cx,cy)
%P3CAMPROJECT  Pinhole projection of base-frame points (n-by-3, robot frame: x forward, y left, z up, origin on the floor under the arm base) into the
%   picture of the lift-carriage camera.  Same camera model as fleetVisionCheck / the rendered Simulation 3D Camera (helperVisionCameraMount).
%   uv = [column row] in pixels, zf = depth along the optical axis [m].
if nargin<3, cx = 320; cy = 240; end
m = helperVisionCameraMount(); f = m(5);
cam = [0.10+m(1) m(2) 0.47+m(3)+lift]; pit = deg2rad(m(4));
fwd = [cos(pit) 0 -sin(pit)]; right = [0 -1 0]; down = [-sin(pit) 0 -cos(pit)];     % right = fwd x z, down = fwd x right
v = P - cam;
zf = v*fwd.'; uv = [cx + f*(v*right.')./zf, cy + f*(v*down.')./zf];
end
