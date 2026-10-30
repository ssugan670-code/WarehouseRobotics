function WH = makePhase3Layout(force,variant)
%MAKEPHASE3LAYOUT  Phase 3 warehouse layout (version 5: variant 8 or 6 shelves) -> data\Phase3\warehouse_layout.mat
%   Floor 58 x 75 m, 6 shelves (both sides A / B in use), 3 packing stations (north), staging area = 2 dark mats with 10 wooden
%   stands (south, middle), 6 robot bays (south wall, 3 at each end).  Positions measured from the layout picture of the project owner (Oct 15).
%   Safe to run again: a file that already is version 2 is left alone (makePhase3Layout(true) rewrites it).
%   The original data\warehouse_layout.mat (Phase 1/2) is never touched.
if nargin<1 || isempty(force), force=false; end
if nargin<2 || isempty(variant), variant=8; end      % 8 = 8-shelf layout (default, the owner's preferred), 6 = 6-shelf layout (design picture of Oct 15)
variant = 6 + 2*(variant>=7);
P = phase3Paths();
if ~isfolder(P.data), mkdir(P.data); end
WH = [];
if isfile(P.layout) && ~force
    S = load(P.layout,'WH'); WH = S.WH;
    if isfield(WH,'layoutVersion') && WH.layoutVersion>=6 && isfield(WH,'nShelves') && WH.nShelves==variant, return, end
end
if variant == 8
    W = struct();
    W.X = 66; W.Y = 75; W.Z = 9;       % 66 m wide (was 58): 8.5-9.5 m of free floor beside the outer faces of shelf 1 (west) and shelf 8 (east) - three robots work side by side there and others still pass
    W.shelves = [-22.5 38 30 2 1; -10.5 44 19 2 1; 1 44 19 2 1; 12.5 47.5 13 2 0; -10.5 23 13 2 0; 1 23 13 2 0; 12.5 26 13 2 0; 23.5 37 30 2 1];
    W.nShelves = 8; W.shelfH = 5.5; W.shelfLevels = 3;
    W.tables = [18.5 68 8 4.5;  0.5 68 8 4.5; -17.5 68 8 4.5];
    W.offCx = -200; W.offCy = 0; W.offW = 1; W.offD = 1;
    W.robotXY = [-20 7.3; -20 3.6; -20 11; -24.5 3.6; -24.5 7.3; -24.5 11];
    W.nRobots = 6; W.mapRes = 2; W.navMap = [];
    W.mats = [20.65 3.65 12.7 4.3; 6.5 3.65 12.8 4.3; -7.75 3.65 11.5 4.3];
    W.stands = [-11 3.7 3; -8.5 3.7 3; -6 3.7 3; -3.5 3.7 3; 1.5 3.7 2; 4 3.7 2; 6.5 3.7 2; 9 3.7 2; 11.5 3.7 2; 15.5 3.7 1; 18 3.7 1; 20.5 3.7 1; 23 3.7 1; 25.5 3.7 1];
    W.standSize = 0.7; W.standHeight = 0.35; W.padDiameter = 1.0;
    W.phase3 = true; W.layoutVersion = 6;
    WH = W; %#ok<NASGU>
    save(P.layout,'WH'); WH = W;
    fprintf('[LAYOUT] Warehouse layout (8 shelves) written: %s (3 stations, %d staging stands, 6 robot bays)\n',P.layout,size(W.stands,1));
    return
end
W = struct();
W.X = 58; W.Y = 75; W.Z = 9;
%            cx     cy   length width  flag
W.shelves = [-15.1  23    13     2     0;     % 1  (short column, south row)
               1.3  23    13     2     0;     % 2
              17.3  23    13     2     0;     % 3
              17.3  44    19     2     1;     % 4  (long column, north row)
               1.3  44    19     2     1;     % 5
             -15.1  44    19     2     1];    % 6
W.nShelves = 6; W.shelfH = 5.5; W.shelfLevels = 3;
W.tables = [ 18.5 68 8 4.5;  0.5 68 8 4.5; -17.5 68 8 4.5];        % 3 packing stations: [cx cy widthX depthY]
W.offCx = -200; W.offCy = 0; W.offW = 1; W.offD = 1;               % office block removed from this layout (kept far outside the floor so old code finds the fields)
% robot bays (south wall, two groups of three, staging area between them): blue red green black pink yellow
W.robotXY = [22.55 3.7; 18.9 3.7; 26.2 3.7; -25.0 3.7; -21.35 3.7; -17.7 3.7];
W.nRobots = 6; W.mapRes = 2; W.navMap = [];
% staging area (receiving dock): 2 dark mats [cx cy lengthX widthY], 10 wooden stands (the cartons stand on them), 5 per mat, pitch 2.5 m
W.mats = [10.1 3.65 12.8 4.3;  -6.8 3.65 12.8 4.3];
sx = {[5 7.5 10 12.5 15], [-11.5 -9 -6.5 -4 -1.5]};
W.stands = zeros(0,3);
for m = 1:numel(sx), for q = 1:numel(sx{m}), W.stands(end+1,:) = [sx{m}(q) 3.7 m]; end, end    %#ok<AGROW>  [x y mat]
W.standSize = 0.7; W.standHeight = 0.35; W.padDiameter = 1.0;
[~,so] = sortrows(W.stands(:,1)); W.stands = W.stands(so,:);      % numbered in one line from west to east: D01 (west) ... D10 (east)
W.phase3 = true; W.layoutVersion = 6;
WH = W; %#ok<NASGU>
save(P.layout,'WH');
WH = W;
fprintf('[LAYOUT] Warehouse layout (6 shelves) written: %s (6 shelves, 3 stations, %d staging stands, 6 robot bays)\n',P.layout,size(W.stands,1));
end
