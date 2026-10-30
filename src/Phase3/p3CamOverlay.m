function im = p3CamOverlay(im,ov)
%P3CAMOVERLAY  Detection overlay on a camera picture (same drawing as the saved cam_*.png): green = detected carton outline,
%   magenta cross = where the database says the carton is, yellow rectangle = search window.  Shown for 3 s after the last detection frame.
if isempty(ov) || ~isstruct(ov) || (now - ov.t)*86400 > 3, return, end
hc = [0 255 0];
try, if ~isempty(ov.hull), im = localLine(im,ov.hull,hc); end, catch, end
u = round(ov.uv);
if numel(u) >= 2 && all(isfinite(u(1:2)))
    im = localLine(im,[u(1)-8 u(2); u(1)+8 u(2)],[255 0 255]); im = localLine(im,[u(1) u(2)-8; u(1) u(2)+8],[255 0 255]);
end
w = ov.win;
if numel(w) >= 4 && w(3) > 0, im = localLine(im,[w(1) w(2); w(3) w(2); w(3) w(4); w(1) w(4); w(1) w(2)],[255 255 0]); end
end
function im = localLine(im,pts,col)
for i = 1:size(pts,1)-1
    n = max(2,ceil(max(abs(pts(i+1,:)-pts(i,:)))));
    x = round(linspace(pts(i,1),pts(i+1,1),n)); y = round(linspace(pts(i,2),pts(i+1,2),n));
    k = x >= 1 & x <= size(im,2) & y >= 1 & y <= size(im,1);
    for c = 1:3, ch = im(:,:,c); ch(sub2ind(size(ch),y(k),x(k))) = col(c); im(:,:,c) = ch; end
end
end
