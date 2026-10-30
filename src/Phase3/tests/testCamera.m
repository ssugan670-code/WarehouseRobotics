function testCamera()
%TESTCAMERA  Regression test of the carton camera on REAL rendered pictures (tests/camframes: two shadowed cartons, one lit carton, two views without a carton in the slot).
%   Old gate: shadowed cartons were not seen at all (noColour) or, after a relaxed gate, found ~170-190 mm off because the white top face was part of the fitted silhouette.
here = fileparts(mfilename('fullpath')); addpath(fullfile(here,'..'));
dims = [0.45 0.50 0.36]; col = [0.50 0.30 0.12]; z = 0.53; opt = struct('minIoU',0.45,'search',0.18,'minPix',150);
cases = {'dark_L1_a.png',[326 239],true; 'dark_L1_b.png',[301 235],true; 'lit_L1.png',[331 236],true; 'absent_a.png',[320 215],false; 'absent_b.png',[320 215],false};
bad = 0;
for k = 1:size(cases,1)
    img = imread(fullfile(here,'camframes',cases{k,1})); img = img(:,:,1:3);
    best = inf; for xx = 0.6:0.02:3, for yy = -1:0.02:1, uv = p3CamProject([xx yy z],0,320,240); e = norm(uv - cases{k,2}); if e < best, best = e; bx = xx; by = yy; end, end, end
    [ok,~,info] = p3CamLocate(img,[bx by z],0,dims,0,col,opt);
    pass = (ok == cases{k,3});
    if cases{k,3}, pass = pass && info.score >= 0.85 && abs(info.dx) < 0.08 && abs(info.dy) < 0.08; end
    fprintf('%-14s expected found=%d -> found=%d IoU %.2f dx %+.0f mm dy %+.0f mm : %s\n',cases{k,1},cases{k,3},ok,info.score,1000*info.dx,1000*info.dy,localTern(pass,'ok','FAIL'));
    bad = bad + ~pass;
end
if bad, error('testCamera: %d case(s) failed',bad); else, fprintf('testCamera: all %d cases ok\n',size(cases,1)); end
end
function s = localTern(c,a,b), if c, s = a; else, s = b; end, end
