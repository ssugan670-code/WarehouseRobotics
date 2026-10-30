function phase2(varargin)
%PHASE2  Start the Phase 2 warehouse: ONE robot (blue) in the Phase 3 warehouse.  Just type   phase2
%   The 3-D window opens, then choose the mode in the MATLAB window:
%       p3 mode auto      the system works by itself: orders and new dock boxes are generated, blue does them (A* route planning), then goes home
%       p3 mode manual    you give the jobs:   p3 blue retrieve 4A-L1-P06 station 3     p3 move 8B-L3-P06 station 1     p3 move D03 1A-L1-P02
%       p3 mode batch     load jobs for blue, nothing moves until  p3 go
%       p3 help           the whole command table;  p3 wms = where is which box
%   All 6 robot home pads are shown, only the blue robot is there.  Routes: A* on the inflated map (the robot's own planner); with one robot there is nothing to coordinate.
% Options (name value pairs, any order, all optional):
%   layout 8 (default, 8 shelves) | layout 6   mode manual/auto   vision on/off   answer ask/auto (what blue does when its target slot is taken)
%   cartons 54 | stored 30 | dock 10 | decor 0 | seed 7 | labels num/full/off | missing [ids] | dwell 25 | verbose 0/1/2 | map on/off (live 2-D map window)
%   perf light/full (light = default: faster 3-D window; full = Unreal lidar, camera every 0.2 s, 2-D map every 0.1 s)   fill max (every shelf slot gets a box, strong PC)
% Example:  phase2 mode auto
here = fileparts(mfilename('fullpath')); if ~isempty(here), cd(here); addpath(here,'-begin'); end
% MathWorks example helpers (helperTargetPose, helperShelfBays, helperPlanPathAStar ...) live in the Phase3 folder: copy missing ones here, else use that folder
if ~exist('helperTargetPose','file') || ~exist('helperShelfBays','file') || ~exist('helperPlanPathAStar','file')
    sib = fullfile(fileparts(here),'Phase3');
    if isfolder(sib)
        d = dir(fullfile(sib,'helper*.*'));
        for q = 1:numel(d)
            if ~d(q).isdir && ~isfile(fullfile(here,d(q).name)), try, copyfile(fullfile(sib,d(q).name),here); catch, end, end
        end
        addpath(sib,'-end'); rehash;
    end
    if ~exist('helperTargetPose','file') || ~exist('helperShelfBays','file') || ~exist('helperPlanPathAStar','file')
        error('Phase:Helpers','helperTargetPose / helperShelfBays / helperPlanPathAStar not found. Copy the helper*.m files of your Phase3 folder into this folder (%s).',here);
    end
end
if nargin>=1 && (strcmpi(varargin{1},'stopped')), localStopped(); return, end
if nargin==2 && strcmpi(varargin{1},'mode')       % phase2 mode auto|manual|batch while the simulation is running = same as  p3 mode ...;  otherwise it STARTS the simulation in that mode
    run_ = false; try, run_ = bdIsLoaded('WarehousePhase2Sim') && strcmp(get_param('WarehousePhase2Sim','SimulationStatus'),'running'); catch, end
    if run_, p3('mode',varargin{2}); return, end
end
o = struct('robots',1,'mode','standby','vision','on','answer','ask','cartons',54,'stored',30,'dock',10,'decor',0,'seed',7,'labels','num','missing',[],'dwell',25,'verbose',1,'map','on','perf','light','layout',8,'planner','mstar','fill','off');
k = 1;
while k <= numel(varargin)
    key = lower(char(varargin{k}));
    if ~isfield(o,key) || k==numel(varargin), error('Phase3:Option','Unknown option or missing value: %s',key); end
    v = varargin{k+1};
    if ischar(v) || isstring(v)
        n = str2double(v); if isfinite(n) && ~any(strcmp(key,{'mode','vision','answer','labels','map','perf','planner','fill'})), v = n; else, v = lower(char(v)); end
    end
    o.(key) = v; k = k+2;
end
if o.robots ~= 1, fprintf('[P2] Phase 2 has ONE robot (blue) - the option robots is ignored.\n'); end
o.robots = 1;
if strcmp(o.fill,'max'), o.stored = Inf; o.cartons = 0; fprintf('[P3] FILL MAX: every shelf slot (except 8 kept free for put-away) gets a real box - the 3-D window needs a strong PC.\n'); else, o.cartons = max(o.cartons, o.stored+o.dock+14); end
% console log of the whole run -> <out folder>\console_log_<time>.txt (send this file when something looks wrong)
try, diary off; pth_=phase3Paths(); if ~isfolder(pth_.out), mkdir(pth_.out); end, if ~isfolder(pth_.logs), mkdir(pth_.logs); end, diary(fullfile(pth_.logs,['console_log_' datestr(now,'yyyymmdd_HHMMSS') '.txt'])); catch, end
% a running model of a previous start is stopped first
try, if bdIsLoaded('WarehousePhase2Sim') && strcmp(get_param('WarehousePhase2Sim','SimulationStatus'),'running'), set_param('WarehousePhase2Sim','SimulationCommand','stop'); fprintf('[P3] stopping the previous run ...\n'); for q=1:30, pause(1); if ~strcmp(get_param('WarehousePhase2Sim','SimulationStatus'),'running'), break, end, end, pause(2); end, catch, end
try, evalin('base','clear Phase3Fleet Phase3Cfg'); catch, end      % the next start builds its own fleet object
P = phase3Paths(); if ~isfolder(P.out), mkdir(P.out); end
makePhase3Layout(false,o.layout);
C = WarehousePhase3Config('cartons',o.cartons,'stored',o.stored,'dock',o.dock,'decor',o.decor,'seed',o.seed,'missing',o.missing,'labels',o.labels);
F = Phase3Fleet(C,o.robots,'answer',o.answer,'dwell',o.dwell,'vision',o.vision,'verbose',o.verbose,'planner',o.planner);
if strcmp(o.perf,'light')      % laptop-friendly: the sensing code is the same, only the sensor rates / the Unreal lidar blocks are reduced
    C.Sense.LidarMode = 'raycast'; C.Sense.LidarPeriod = 0.20; C.Sense.CamPeriod = 0.40; C.Sense.CamFrames = 4;
end
assignin('base','Phase3Cfg',C); assignin('base','Phase3Fleet',F);
if any(strcmp(o.mode,{'auto','manual','batch'})), F.setMode(o.mode); else, F.mode = 'standby'; end      % standby = waiting for  p3 mode ...  (shown on the 2-D map)
F.map2D = strcmp(o.map,'on'); % the 2-D map opens by itself with the first fleet report, i.e. when the simulation really starts (clocks start with it)
mdl = buildFleetSim(o.robots);
set_param(mdl,'StopTime','inf','ReturnWorkspaceOutputs','off');
set_param(mdl,'SimulationCommand','start');
fprintf('\n[P3] %d robot(s): %s.  Simulation starting - wait for the 3-D window.\n',o.robots,strjoin({C.P3.robots(1:o.robots).name},', '));
fprintf('\n[P3] CHOOSE THE MODE (type in this window):\n');
fprintf('       p3 mode auto       the system works by itself (emergency job:  p3 urgent <slot> [station n])\n');
fprintf('       p3 mode manual     you give the jobs, they run at once in parallel   (p3 wms = where is which box)\n');
fprintf('       p3 mode batch      test single robots: jobs per robot, start with  p3 go\n');
fprintf('       p3 help            short command table\n');
fprintf('       outputs: %s\n\n',P.out);
end

function localStopped()
try
    F = evalin('base','Phase3Fleet'); F.metricsWrite(true);
    fprintf('[P3] simulation stopped. Metrics written to %s\n',F.outDir);
    diary off

catch
end
end
