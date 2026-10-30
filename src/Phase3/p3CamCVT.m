function varargout = p3CamCVT(action,varargin)
%P3CAMCVT  Computer Vision Toolbox layer of the carton camera (used by p3CamLocate).
%   [uv,zf]       = p3CamCVT('project',P,lift,cx,cy)   base-frame points (n-by-3) -> picture pixels [column row] and depth zf
%                   through   cameraIntrinsics + world2img   (Computer Vision Toolbox).  Checked against the plain pinhole model (p3CamProject) on the
%                   first call; if the two disagree by more than 1.5 px or any toolbox call fails, the plain model is used and the reason is logged.
%   [area,bbox]   = p3CamCVT('blobs',mask)             carton-coloured blobs of a logical mask through  vision.BlobAnalysis  (area [px], [x y w h]);
%                   falls back to plain connected-component code when the toolbox / licence is missing.
%   st            = p3CamCVT('status')                 struct: projOK, blobOK, projWhy, blobWhy (what really runs)
%   p3CamCVT('reset')                                  forget the self-checks (tests)
% The numbers used are the ones of the rendered camera (helperVisionCameraMount): focal length in pixels, mounting height / pitch on the lift carriage.
persistent S blobObj
if isempty(S), S = localNew(); blobObj = []; end
switch lower(action)
    case 'reset'
        S = localNew(); blobObj = []; varargout = {};
    case 'status'
        if ~S.projChecked, S = localCheckProj(S); end
        if ~S.blobChecked, [S,blobObj] = localCheckBlob(S); end
        varargout{1} = struct('projOK',S.projOK,'blobOK',S.blobOK,'projWhy',S.projWhy,'blobWhy',S.blobWhy,'convention',S.conv);
    case 'project'
        P = varargin{1}; lift = varargin{2}; cx = varargin{3}; cy = varargin{4};
        if ~S.projChecked, S = localCheckProj(S); end
        if S.projOK
            try
                [uv,zf] = localProjectCVT(P,lift,cx,cy,S.conv); varargout = {uv,zf}; return
            catch ME
                S.projOK = false; S.projWhy = ['world2img failed: ' ME.message]; localLog('projection',S.projWhy);
            end
        end
        [uv,zf] = p3CamProject(P,lift,cx,cy); varargout = {uv,zf};
    case 'blobs'
        mask = logical(varargin{1});
        if ~S.blobChecked, [S,blobObj] = localCheckBlob(S); end
        if S.blobOK
            try
                [a,~,b] = step(blobObj,mask);
                varargout = {double(a(:)),double(b)}; return
            catch ME
                S.blobOK = false; S.blobWhy = ['vision.BlobAnalysis failed: ' ME.message]; localLog('blob analysis',S.blobWhy);
            end
        end
        [a,b] = localBlobsPlain(mask); varargout = {a,b};
    otherwise
        error('p3CamCVT: unknown action %s',action);
end
end

function S = localNew()
S = struct('projChecked',false,'projOK',false,'projWhy','not checked','blobChecked',false,'blobOK',false,'blobWhy','not checked','conv',1);
end

function S = localCheckProj(S)
S.projChecked = true; S.projOK = false;
try
    if isempty(which('cameraIntrinsics')) || isempty(which('world2img')) || isempty(which('rigidtform3d'))
        S.projWhy = 'cameraIntrinsics / world2img not available (Computer Vision Toolbox not installed or not licensed) - plain pinhole model used'; localLog('projection',S.projWhy); return
    end
    T = [1.2 0 0.53; 1.0 0.3 0.53; 1.4 -0.2 1.43; 0.9 0 0.38];      % a carton at a stand / at the shelf levels
    bad = inf;
    for conv = [1 2]
        err = 0;
        for lf = [0 0.9]
            a = localProjectCVT(T,lf,320,240,conv); b = p3CamProject(T,lf,320,240); err = max(err,max(abs(a(:) - b(:))));
        end
        bad = err; S.conv = conv; if err < 1.5, break, end
    end
    if bad < 1.5
        S.projOK = true; S.projWhy = sprintf('cameraIntrinsics + world2img active, max difference to the pinhole model %.2f px',bad);
    else
        S.projWhy = sprintf('world2img disagrees with the pinhole model by %.1f px - plain model used',bad);
    end
    localLog('projection',S.projWhy);
catch ME
    S.projOK = false; S.projWhy = ['toolbox call failed: ' ME.message]; localLog('projection',S.projWhy);
end
end

function [uv,zf] = localProjectCVT(P,lift,cx,cy,conv)
m = helperVisionCameraMount(); f = m(5);
cam = [0.10+m(1) m(2) 0.47+m(3)+lift]; pit = deg2rad(m(4));
fwd = [cos(pit) 0 -sin(pit)]; right = [0 -1 0]; down = [-sin(pit) 0 -cos(pit)];
K = cameraIntrinsics([f f],[cx cy],[round(2*cy) round(2*cx)]);          % image 480 x 640: [rows cols]
Rcw = [right(:) down(:) fwd(:)];                                         % camera axes (x right, y down, z forward) in the robot frame
Rwc = Rcw.'; t = -Rwc*cam(:);                                            % world -> camera
if conv == 1, tf = rigidtform3d(Rwc,t.'); else, tf = rigidtform3d(Rcw,cam); end
uv = double(world2img(P,tf,K));
zf = (P - cam)*fwd.';
end

function [S,blobObj] = localCheckBlob(S)
S.blobChecked = true; S.blobOK = false; blobObj = [];
try
    if isempty(which('vision.BlobAnalysis'))
        S.blobWhy = 'vision.BlobAnalysis not available (Computer Vision Toolbox not installed or not licensed) - plain connected-component code used'; localLog('blob analysis',S.blobWhy); return
    end
    blobObj = vision.BlobAnalysis('AreaOutputPort',true,'CentroidOutputPort',true,'BoundingBoxOutputPort',true,'MinimumBlobArea',60,'MaximumCount',25);
    test = false(60,80); test(10:30,20:50) = true; test(40:45,60:70) = true;
    [a,~,b] = step(blobObj,test);
    if numel(a) == 2 && abs(max(a) - 21*31) < 1.5 && any(abs(b(:,3) - 31) < 1.5)
        S.blobOK = true; S.blobWhy = 'vision.BlobAnalysis active';
    else
        S.blobWhy = 'vision.BlobAnalysis gave an unexpected result on the test mask - plain code used';
    end
    localLog('blob analysis',S.blobWhy);
catch ME
    S.blobOK = false; S.blobWhy = ['vision.BlobAnalysis failed: ' ME.message]; localLog('blob analysis',S.blobWhy);
end
end

function localLog(what,msg)
fprintf('[CAM] Computer Vision Toolbox %s: %s\n',what,msg);
end

function [area,bbox] = localBlobsPlain(mask)
% 4-connected components (no toolbox): area and bounding box [x y w h], largest first
[h,w] = size(mask); lab = zeros(h,w); n = 0; area = zeros(0,1); bbox = zeros(0,4);
nb = [1 0; -1 0; 0 1; 0 -1];
for i = 1:h
    for j = 1:w
        if mask(i,j) && lab(i,j) == 0
            n = n+1; stack = [i j]; lab(i,j) = n; cnt = 0; r0 = i; r1 = i; c0 = j; c1 = j;
            while ~isempty(stack)
                p = stack(end,:); stack(end,:) = []; cnt = cnt+1;
                r0 = min(r0,p(1)); r1 = max(r1,p(1)); c0 = min(c0,p(2)); c1 = max(c1,p(2));
                for d = 1:4
                    q = p + nb(d,:);
                    if q(1) >= 1 && q(1) <= h && q(2) >= 1 && q(2) <= w && mask(q(1),q(2)) && lab(q(1),q(2)) == 0, lab(q(1),q(2)) = n; stack(end+1,:) = q; end %#ok<AGROW>
                end
            end
            area(end+1,1) = cnt; bbox(end+1,:) = [c0 r0 c1-c0+1 r1-r0+1]; %#ok<AGROW>
        end
    end
end
[area,k] = sort(area,'descend'); bbox = bbox(k,:);
end
