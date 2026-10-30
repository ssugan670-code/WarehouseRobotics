function phase1(varargin)
%PHASE1  Phase 1 - the warehouse environment only.  Just type   phase1
%   Opens the 3-D window with the warehouse: floor and walls, 8 shelves (3 levels, boxes on them), 3 packing stations, the staging area with its
%   wooden stands and boxes, the position numbers on the floor and the 6 robot home pads (blue red green black pink yellow).  There is NO robot and nothing
%   can be operated - this phase is for looking at the environment.
%   In the 3-D window:  hold the right mouse button and move the mouse to look around, W A S D to fly, Q / E down / up.
%   phase1 stop                 stops the simulation (or press Stop in the Simulink window)
% Options (name value pairs, any order), all optional:
%   layout 8 (default, 8 shelves) | layout 6    labels num (default, position numbers on the floor) | labels full | labels off
%   stored 30 | dock 10 | seed 7                 number of boxes on the shelves / in the staging area, seed of the random choice of slots
%   Example:   phase1 layout 6 labels off
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
mdl = 'WarehouseEnvSim';
if nargin>=1 && strcmpi(varargin{1},'stop')
    try, if bdIsLoaded(mdl), set_param(mdl,'SimulationCommand','stop'); fprintf('[P1] simulation stopped.\n'); end, catch, end
    return
end
o = struct('layout',8,'labels','num','stored',30,'dock',10,'seed',7);
k = 1;
while k <= numel(varargin)
    key = lower(char(varargin{k}));
    if ~isfield(o,key) || k==numel(varargin), error('Phase1:Option','Unknown option or missing value: %s   (options: layout labels stored dock seed)',key); end
    v = varargin{k+1};
    if ischar(v) || isstring(v)
        n = str2double(v); if isfinite(n) && ~strcmp(key,'labels'), v = n; else, v = lower(char(v)); end
    end
    o.(key) = v; k = k+2;
end
% a running model of a previous start is stopped first
try
    if bdIsLoaded(mdl) && strcmp(get_param(mdl,'SimulationStatus'),'running')
        set_param(mdl,'SimulationCommand','stop'); fprintf('[P1] stopping the previous run ...\n');
        for q=1:30, pause(1); if ~strcmp(get_param(mdl,'SimulationStatus'),'running'), break, end, end, pause(2);
    end
catch
end
try, evalin('base','clear Phase3Fleet Phase3Cfg'); catch, end
P = phase3Paths(); if ~isfolder(P.out), mkdir(P.out); end
makePhase3Layout(false,o.layout);
C = WarehousePhase3Config('cartons',max(54,o.stored+o.dock+14),'stored',o.stored,'dock',o.dock,'decor',0,'seed',o.seed,'missing',[],'labels',o.labels);
C.UnrealPhysics = false;                 % boxes simply stand where they are put
assignin('base','Phase3Cfg',C);
if evalin('base','exist(''world'',''var'')'), try, close(evalin('base','world')); catch, end; try, delete(evalin('base','world')); catch, end; evalin('base','clear world'); end
if bdIsLoaded(mdl), close_system(mdl,0); end
modelFile = fullfile(pwd,[mdl '.slx']); if exist(modelFile,'file'), delete(modelFile); end
load_system('sim3dlib');
new_system(mdl);
set_param(mdl,'Solver','ode4','FixedStep',num2str(C.SampleTime),'StopTime','inf');
try, set_param(mdl,'EnablePacing','on'); set_param(mdl,'PacingRate','1'); catch, end
scn = localAdd(mdl,'Simulation 3D Scene Configuration','Scene Configuration',[300 100 520 160]);
set_param(scn,'ProjectFormat','Default Scenes','SceneDesc','Empty scene','Priority','0');
env = localAdd(mdl,'Simulation 3D Actor','Phase1 Warehouse',[300 220 460 310]);
set_param(env,'ActorName','WarehouseRoot','ParentName','Scene Origin','Operation','Create at setup','InitScriptText','setupEnvScene(Actor,World);','Priority','1');
set_param(mdl,'SimulationCommand','update');
save_system(mdl,modelFile); open_system(mdl);
set_param(mdl,'SimulationCommand','start');
nBox = numel(C.P3.cartons);
fprintf('\n[P1] Phase 1: the warehouse environment (%d shelves, 3 packing stations, staging area, 6 robot home pads, %d boxes in the pool).  No robot.\n',size(C.WH.shelves,1),nBox);
fprintf('[P1] Wait for the 3-D window.  Look around: hold the right mouse button + move the mouse, W A S D to fly, Q / E down / up.\n');
fprintf('[P1] Stop with   phase1 stop   (or the Stop button of the Simulink window).\n\n');
end

function block = localAdd(model,blockName,newName,position)
found = find_system('sim3dlib','SearchDepth',1,'FollowLinks','on','Name',blockName);
assert(~isempty(found),'Phase1:BlockNotFound','Could not find "%s" in sim3dlib (Automated Driving Toolbox Interface for Unreal Engine 4 Projects needed).',blockName);
block = [model '/' newName]; add_block(found{1},block,'Position',position);
end
