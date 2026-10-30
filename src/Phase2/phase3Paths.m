function P = phase3Paths()
%PHASE3PATHS  One place for the Phase 2 folders.  Environment variable P3_ROOT overrides the project root (used by the headless tests).
root = getenv('P3_ROOT');
if isempty(root), root = 'C:\WarehouseRobotics'; end
P.root   = root;
P.data   = fullfile(root,'data','Phase2');
P.layout = fullfile(P.data,'warehouse_layout.mat');   % Phase 2 copy
P.out    = fullfile(P.data,'out');                    % slot snapshots, metrics, event logs
P.metrics = fullfile(P.out,'Metrics');  P.tasks = fullfile(P.out,'Tasks');  P.events = fullfile(P.out,'Events');      % every kind of output file has its own folder inside data\Phase3\out
P.summary = fullfile(P.out,'Summary');  P.logs  = fullfile(P.out,'Logs');   P.slots  = fullfile(P.out,'Slots');
P.sense = fullfile(P.out,'Sense');  P.cam = fullfile(P.out,'Camera');  P.diag = fullfile(P.out,'Diagnostics');  P.videos = fullfile(P.out,'Videos');      % sense csv / camera pictures / lidar+p3sense diagnostics / camera videos: one folder each
P.debug  = fullfile(root,'debug');
end
