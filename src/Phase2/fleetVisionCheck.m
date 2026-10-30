function [hit,info,known] = fleetVisionCheck(img,pose,lift,slotXYZ,C)
%FLEETVISIONCHECK  Does the robot camera picture show a carton (carton) where the slot database says one is?
%   img     480x640x3 uint8 from the robot camera       pose  [x y yaw] of that robot       lift  carriage height [m]
%   slotXYZ expected carton centre (right-handed world)   C     config (image size, focal length)
% The expected carton is projected into the picture with the same camera model that places the camera (helperVisionCameraMount);
% the check looks for pixels of the carton colour (C.P3.cartonColour, hue gate) inside a window around that pixel.
%   hit   = enough carton-coloured pixels in the window          known = the slot is inside the picture (otherwise nothing can be said)
hit = false; known = false; info = struct('uv',[NaN NaN],'pixels',0,'window',[0 0 0 0]);
m = helperVisionCameraMount(); f = m(5); cx = 320; cy = 240;
yaw = pose(3); R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
cxy = pose(1:2).' + R*[0.10+m(1); m(2)]; cam = [cxy.' 0.47+m(3)+lift];
pit = deg2rad(m(4));
fwd = [cos(yaw)*cos(pit) sin(yaw)*cos(pit) -sin(pit)];
right = cross(fwd,[0 0 1]); right = right/norm(right);
down = cross(fwd,right); down = down/norm(down);
v = slotXYZ(:).' - cam; zf = dot(v,fwd);
if zf < 0.3, return, end
u = cx + f*dot(v,right)/zf; w = cy + f*dot(v,down)/zf;
info.uv = [u w];
half = max(40,min(200,f*(0.40 + 0.25)/zf));      % carton half size 0.4 m + 0.25 m tolerance for the pose belief (AMCL) and the camera delay
if u < 0 || u > 640 || w < 0 || w > 480, return, end
known = true;
inside = (min(640,u+half) - max(0,u-half))*(min(480,w+half) - max(0,w-half))/(4*half^2);
info.inside = inside;      % fraction of the search window that is really in the picture: a slot at the edge of the picture (or cut off at the top) cannot be judged
c0 = max(1,round(u-half)); c1 = min(size(img,2),round(u+half)); r0 = max(1,round(w-half)); r1 = min(size(img,1),round(w+half));
info.window = [c0 r0 c1 r1];
sub = double(img(r0:r1,c0:c1,:))/255;
hsv = rgb2hsv(sub);
col = [0.50 0.30 0.12]; try, col = C.P3.cartonColour(:).'; catch, end       % the gate follows the carton colour of the config (any colour works)
mask = p3CamMask(hsv,col);                                    % same gate as the locator (shadowed cartons count, the orange stand does not)
info.pixels = nnz(mask);
expect = (f*0.45/zf)*(f*0.36/zf);      % pixels of the carton face in the picture at this distance
hit = info.pixels >= max(40,0.12*expect);
end
