function [found,rel,info] = p3CamLocate(img,expRel,yawRel,dims,lift,colour,opt)
%P3CAMLOCATE  Where is the carton?  Model-based pose estimation from ONE camera picture (Computer Vision Toolbox: cameraIntrinsics + world2img for the camera model,
%   vision.BlobAnalysis for the carton blob - through p3CamCVT, with a self-check and a plain fallback when the toolbox / licence is missing).
%   img      480x640x3 uint8 picture of the robot camera
%   expRel   carton centre the warehouse database expects, in the ROBOT frame [x y z]  (shelf slot -> robot frame with the robot's pose estimate)
%   yawRel   carton yaw relative to the robot [rad]      dims = [dx dy dz] carton size [m]      lift = carriage height [m]
%   colour   carton colour [r g b] 0..1 (config)         opt: .minIoU (0.45) .search (0.18) .minPix (150)
% 1. colour segmentation (HSV gate) in a window around where the carton should be
% 2. the carton's silhouette (convex hull of the corners of its VISIBLE side faces - the lit top face is white, not carton coloured, and would pull the fit towards the stand / floor by ~0.2 m) is slid over the picture in the ground plane (dx,dy), coarse then fine, and scored
%    by intersection-over-union with the colour mask (integral image: O(width) per candidate).  Pixels BELOW the silhouette are ignored (stand / floor).
% 3. the best offset is the carton's measured position:  rel = expRel + [dx dy 0].   z and yaw stay at the database values (shelf level / stand are known).
% found = false when the window is outside the picture, too few carton-coloured pixels, or the best IoU is below minIoU (carton absent / hidden).
if nargin<7, opt = struct(); end
if ~isfield(opt,'minIoU'), opt.minIoU = 0.45; end
if ~isfield(opt,'search'), opt.search = 0.18; end
if ~isfield(opt,'minPix'), opt.minPix = 150; end
found = false; rel = expRel(:).'; info = struct('score',0,'dx',0,'dy',0,'pixels',0,'reason','','hull',zeros(0,2),'win',[0 0 0 0],'expUV',[NaN NaN],'blobArea',0,'blobBox',[0 0 0 0],'blobIoU',0);
H = size(img,1); W = size(img,2); cx = W/2; cy = H/2;
proj = @(P) p3CamCVT('project',P,lift,cx,cy);
[uvc,zf] = proj(expRel(:).'); info.expUV = uvc;
if zf < 0.3 || zf > 4.5, info.reason = 'range'; return, end
sx = opt.search; ex = [-sx sx];
cs = localCorners(expRel,yawRel,dims);
m_ = helperVisionCameraMount(); camPos = [0.10+m_(1) m_(2) 0.47+m_(3)+lift];
csT = localSideFaces(expRel,yawRel,dims,camPos);          % what the camera really sees of the carton: the side faces turned towards it (the top face is glare-white, not carton coloured)
ext = []; for a = ex, for b = ex, uvx = proj(cs + [a b 0]); ext = [ext; uvx]; end, end %#ok<AGROW>
c0 = max(1,floor(min(ext(:,1)))-6); c1 = min(W,ceil(max(ext(:,1)))+6); r0 = max(1,floor(min(ext(:,2)))-6); r1 = min(H,ceil(max(ext(:,2)))+6);
if c1-c0 < 20 || r1-r0 < 20, info.reason = 'outside'; return, end
info.win = [c0 r0 c1 r1];
sub = double(img(r0:r1,c0:c1,:))/255; hsv = localRgb2Hsv(sub);
mask = p3CamMask(hsv,colour);                                              % shared gate (hue + saturation band + low value floor: shadowed cartons, stand excluded)
info.pixels = nnz(mask);
if info.pixels < opt.minPix, info.reason = 'noColour'; return, end
[blobA,blobB] = p3CamCVT('blobs',mask);                                   % carton-coloured blobs (vision.BlobAnalysis); the largest one is the carton
if ~isempty(blobA), info.blobArea = blobA(1); info.blobBox = blobB(1,:) + [c0-1 r0-1 0 0]; end
Cc = cumsum(double(mask),1); Rs = cumsum(sum(double(mask),2));
Hh = size(mask,1); Ww = size(mask,2);
best = -1; bdx = 0; bdy = 0;
for stage = 1:2
    if stage == 1, gx = -sx:0.03:sx; gy = gx; else, gx = bdx+(-0.03:0.006:0.03); gy = bdy+(-0.03:0.006:0.03); end
    nx_ = numel(gx); ny_ = numel(gy); nT = size(csT,1); Pall = zeros(nT*nx_*ny_,3); q = 0;
    for ix = 1:nx_
        for iy = 1:ny_
            Pall(nT*q+(1:nT),:) = csT + [gx(ix) gy(iy) 0]; q = q+1;      % all candidate silhouettes of this stage go through ONE projection call
        end
    end
    uvAll = proj(Pall); q = 0;
    for ix = 1:nx_
        for iy = 1:ny_
            s = localScore(uvAll(nT*q+(1:nT),:),c0,r0,Cc,Rs,Hh,Ww); q = q+1;
            if s > best, best = s; bdx = gx(ix); bdy = gy(iy); end
        end
    end
end
info.score = best; info.dx = bdx; info.dy = bdy;
uvh = proj(csT + [bdx bdy 0]);
try, k = convhull(uvh(:,1),uvh(:,2)); info.hull = uvh(k,:); catch, info.hull = uvh; end
if info.blobArea > 0      % agreement of the carton blob with the fitted silhouette (bounding boxes)
    hb = [min(uvh(:,1)) min(uvh(:,2)) max(uvh(:,1)) max(uvh(:,2))]; bb = [info.blobBox(1) info.blobBox(2) info.blobBox(1)+info.blobBox(3) info.blobBox(2)+info.blobBox(4)];
    iw = max(0,min(hb(3),bb(3)) - max(hb(1),bb(1))); ih = max(0,min(hb(4),bb(4)) - max(hb(2),bb(2))); ia = iw*ih;
    ua = (hb(3)-hb(1))*(hb(4)-hb(2)) + (bb(3)-bb(1))*(bb(4)-bb(2)) - ia; if ua > 0, info.blobIoU = ia/ua; end
end
if best < opt.minIoU, info.reason = 'lowScore'; return, end
found = true; rel = expRel(:).' + [bdx bdy 0]; info.reason = 'ok';
end

function P = localSideFaces(c,yaw,d,cam)
% corners of the vertical faces of the carton whose outward normal points towards the camera
R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)]; P = zeros(0,3);
nrm = [1 0; -1 0; 0 1; 0 -1];
for k = 1:4
    n = R*nrm(k,:).'; half = 0.5*abs(nrm(k,1))*d(1) + 0.5*abs(nrm(k,2))*d(2);
    fc = [c(1:2).' + n*half; c(3)];
    if dot([n; 0],cam(:) - fc) <= 0, continue, end
    t = [-nrm(k,2) nrm(k,1)]; w = 0.5*(abs(t(1))*d(1) + abs(t(2))*d(2)); tw = R*t.';
    for a = [-1 1], for z = [-1 1]
        p2 = fc(1:2) + a*w*tw; P(end+1,:) = [p2.' fc(3) + z*d(3)/2]; %#ok<AGROW>
    end, end
end
if isempty(P), P = localCorners(c,yaw,d); end
end

function cs = localCorners(c,yaw,d)
% 8 corners of the carton (robot frame)
R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)]; cs = zeros(8,3); q = 0;
for sx = [-1 1], for sy = [-1 1], for sz = [-1 1]
    q = q+1; p = R*[sx*d(1)/2; sy*d(2)/2]; cs(q,:) = [c(1)+p(1) c(2)+p(2) c(3)+sz*d(3)/2];
end, end, end
end

function s = localScore(uv,c0,r0,Cc,Rs,Hh,Ww)
u = uv(:,1)-c0+1; v = uv(:,2)-r0+1;
try, k = convhull(u,v); catch, s = 0; return, end
hx = u(k); hy = v(k); x0 = max(1,ceil(min(hx))); x1 = min(Ww,floor(max(hx)));
if x1 < x0, s = 0; return, end
xs = (x0:x1); yt = inf(size(xs)); yb = -inf(size(xs));
for e = 1:numel(hx)-1
    xa = hx(e); xb = hx(e+1); ya = hy(e); yb_ = hy(e+1);
    if xa == xb, continue, end
    lo = min(xa,xb); hi = max(xa,xb); in = xs >= lo & xs <= hi;
    t = (xs(in)-xa)/(xb-xa); yy = ya + t*(yb_-ya);
    yt(in) = min(yt(in),yy); yb(in) = max(yb(in),yy);
end
ok = isfinite(yt) & isfinite(yb); if ~any(ok), s = 0; return, end
xs = xs(ok); yt = max(1,ceil(yt(ok))); yb = min(Hh,floor(yb(ok)));
v_ = yb >= yt; xs = xs(v_); yt = yt(v_); yb = yb(v_); if isempty(xs), s = 0; return, end
lb = Cc(sub2ind([Hh Ww],yb,xs)); la = zeros(size(xs)); m = yt > 1; la(m) = Cc(sub2ind([Hh Ww],yt(m)-1,xs(m)));
inter = sum(lb - la); area = sum(yb - yt + 1);
rowMax = min(Hh,max(yb)+2); tot = Rs(rowMax);
s = inter/max(1,(area + tot - inter));
end

function hsv = localRgb2Hsv(rgb)
% rgb2hsv without toolboxes (Octave / MATLAB identical)
r = rgb(:,:,1); g = rgb(:,:,2); b = rgb(:,:,3); mx = max(max(r,g),b); mn = min(min(r,g),b); d = mx-mn;
h = zeros(size(r)); nz = d > 1e-9;
m1 = nz & mx==r; h(m1) = mod((g(m1)-b(m1))./d(m1),6);
m2 = nz & mx==g & ~m1; h(m2) = (b(m2)-r(m2))./d(m2)+2;
m3 = nz & ~m1 & ~m2; h(m3) = (r(m3)-g(m3))./d(m3)+4;
s = zeros(size(r)); pz = mx > 0; s(pz) = d(pz)./mx(pz);
hsv = cat(3,h/6,s,mx);
end
