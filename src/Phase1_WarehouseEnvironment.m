%% Phase1_WarehouseEnvironment.m
% Warehouse Robotics Simulation — Phase 1 (v4 - Clean Layout)
%
% Layout (compass: North=far, South=near, East=right, West=left):
%   - 2 large Tables (North area)
%   - Office with door (South-West corner)
%   - 7 Shelves all facing EAST (open East side):
%       3 LONG : West wall, East upper, East lower
%       4 MEDIUM: 2 center-upper pair, 2 center-lower pair
%   - 5 Kinova Movo robots on charging pads (South center)
%
% USAGE: WH = Phase1_WarehouseEnvironment();

function WH = Phase1_WarehouseEnvironment()

cd('C:\WarehouseRobotics');
addpath(genpath('C:\WarehouseRobotics'));

fprintf('==============================================\n');
fprintf('  Warehouse Robotics Simulation - Phase 1\n');
fprintf('==============================================\n');

WH = defineParams();
WH.navMap = buildNavMap(WH);
fprintf('[Phase1] Nav map: %dx%d cells\n', ...
    WH.navMap.GridSize(1), WH.navMap.GridSize(2));

fprintf('[Phase1] Creating 3D world...\n');
world = sim3d.World('Scene','Empty scene','StopTime',9999);

fprintf('[Phase1] Building warehouse...\n');
buildFloor(world, WH);
buildWalls(world, WH);
buildAllShelves(world, WH);
buildTables(world, WH);
buildOffice(world, WH);
buildSafetyLines(world, WH);
buildChargingZone(world, WH);
buildRobots(world, WH);

createViewport(world, Translation=[0,-42,60], Rotation=[0,0.80,0]);

if ~isfolder('data'), mkdir('data'); end
navMap = WH.navMap; %#ok<NASGU>
save('data\warehouse_layout.mat','WH','navMap');

fprintf('\n==============================================\n');
fprintf('  PHASE 1 COMPLETE!\n');
fprintf('  Warehouse : %.0fm x %.0fm\n', WH.X, WH.Y);
fprintf('  Shelves   : 7  |  Robots : 5\n');
fprintf('==============================================\n\n');

view(world);
assignin('base','world',world);
end


%% ── PARAMETERS ────────────────────────────────────────────────────────────
function WH = defineParams()

% Warehouse size (meters)
WH.X = 56;   % E-W width  (X: -28 to +28) — larger boundary
WH.Y = 72;   % N-S depth  (Y: 0 = South wall, 72 = North wall) — larger boundary
WH.Z = 9;    % height

% 7 shelf definitions: [cx, cy, length(Y), width(X), isLong]
% All shelves face EAST (open face on +X side)
WH.shelves = [
%   cx      cy     lenY   widX  long
   -18.0,  30.0,  28.0,  2.0,  1;   % S1 West wall  (long)
   +18.0,  42.0,  16.0,  2.0,  1;   % S2 East upper (long)
   +18.0,  20.0,  18.0,  2.0,  1;   % S3 East lower (long)
    -3.0,  38.0,  11.0,  1.8,  0;   % S4 Center-upper A (medium)
    +7.0,  38.0,  11.0,  1.8,  0;   % S5 Center-upper B (medium)
    -3.0,  18.0,  11.0,  1.8,  0;   % S6 Center-lower A (medium)
    +7.0,  18.0,  11.0,  1.8,  0;   % S7 Center-lower B (medium)
];
WH.nShelves  = 7;
WH.shelfH    = 5.5;   % shelf height
WH.shelfLevels = 3;   % levels per shelf

% 2 delivery tables (North area) — bigger size, pushed further North
WH.tables = [
   -8.0, 58.0, 8.0, 4.5;   % [cx, cy, W, D] Table 1 — was [6.0,3.0] at Y=51
   +6.0, 58.0, 8.0, 4.5;   % Table 2
];

% Office (South-West corner)
WH.offCx = -13.0; WH.offCy = 6.0;
WH.offW  =  10.0; WH.offD  = 9.0;

% Robot charging pads (South-center)
WH.robotXY = [
    -3.0,  3.0;
     0.0,  3.0;
    +3.0,  3.0;
    -1.5,  6.5;
    +1.5,  6.5;
];
WH.nRobots = 5;

WH.mapRes = 2;
end


%% ── NAV MAP ───────────────────────────────────────────────────────────────
function navMap = buildNavMap(WH)
mW = round(WH.X * WH.mapRes);
mH = round(WH.Y * WH.mapRes);
G  = zeros(mH, mW, 'logical');
im = 0.5;
wb = round(0.3*WH.mapRes);
G(1:wb,:)=1; G(end-wb+1:end,:)=1;
G(:,1:wb)=1; G(:,end-wb+1:end)=1;

    function G = obs(G,xc,yc,w,d)
        xs=xc+WH.X/2;
        r1=max(1,round((yc-d/2-im)*WH.mapRes));
        r2=min(mH,round((yc+d/2+im)*WH.mapRes));
        c1=max(1,round((xs-w/2-im)*WH.mapRes));
        c2=min(mW,round((xs+w/2+im)*WH.mapRes));
        G(r1:r2,c1:c2)=1;
    end

for s=1:WH.nShelves
    G=obs(G,WH.shelves(s,1),WH.shelves(s,2),WH.shelves(s,4),WH.shelves(s,3));
end
for t=1:size(WH.tables,1)
    G=obs(G,WH.tables(t,1),WH.tables(t,2),WH.tables(t,3),WH.tables(t,4));
end
G=obs(G,WH.offCx,WH.offCy,WH.offW,WH.offD);
navMap=binaryOccupancyMap(double(G),WH.mapRes);
end


%% ── FLOOR ─────────────────────────────────────────────────────────────────
function buildFloor(world, WH)
a=sim3d.Actor(ActorName='Floor'); add(world,a);
createShape(a,'box',[WH.X,WH.Y,0.15]);
a.Translation=[0,WH.Y/2,-0.075];
a.Color=[0.80,0.80,0.78];
end


%% ── WALLS ─────────────────────────────────────────────────────────────────
function buildWalls(world, WH)
wt=0.4; h=WH.Z; c=[0.74,0.74,0.72];
defs={'WallS',[0,0,h/2],      [WH.X+0.8,wt,h];
      'WallN',[0,WH.Y,h/2],   [WH.X+0.8,wt,h];
      'WallW',[-WH.X/2,WH.Y/2,h/2],[wt,WH.Y,h];
      'WallE',[+WH.X/2,WH.Y/2,h/2],[wt,WH.Y,h]};
for i=1:4
    a=sim3d.Actor(ActorName=defs{i,1}); add(world,a);
    createShape(a,'box',defs{i,3});
    a.Translation=defs{i,2}; a.Color=c;
end
end


%% ── SINGLE SHELF BUILDER ──────────────────────────────────────────────────
function buildShelf(world, sid, cx, cy, lenY, widX, height, nLev)

seedV = mod(abs(round(cx*17+cy*11+sid*5)), 2^31-1);
rng(seedV);

upC  = [0.18,0.24,0.38];
bmC  = [0.22,0.30,0.46];
brdC = [0.65,0.55,0.38];
yelC = [0.95,0.85,0.10];
boxCols = [0.88,0.78,0.58; 0.82,0.72,0.50;
           0.92,0.84,0.64; 0.78,0.68,0.46;
           0.86,0.76,0.54];

postW = 0.14;
levZ  = linspace(1.0, height-0.5, nLev);

a=sim3d.Actor(ActorName=sprintf('S%d_PostS',sid)); add(world,a);
createShape(a,'box',[widX, postW, height]);
a.Translation=[cx, cy-lenY/2+postW/2, height/2]; a.Color=upC;

b=sim3d.Actor(ActorName=sprintf('S%d_PostN',sid)); add(world,b);
createShape(b,'box',[widX, postW, height]);
b.Translation=[cx, cy+lenY/2-postW/2, height/2]; b.Color=upC;

c=sim3d.Actor(ActorName=sprintf('S%d_WPost',sid)); add(world,c);
createShape(c,'box',[0.10, postW, height]);
c.Translation=[cx-widX/2+0.05, cy-lenY/2+postW/2, height/2]; c.Color=upC;
c2=sim3d.Actor(ActorName=sprintf('S%d_WPost2',sid)); add(world,c2);
createShape(c2,'box',[0.10, postW, height]);
c2.Translation=[cx-widX/2+0.05, cy+lenY/2-postW/2, height/2]; c2.Color=upC;

persistent sqBoxCount
if isempty(sqBoxCount), sqBoxCount = 0; end

for lv=1:nLev
    nm=sprintf('S%d_BmF%d',sid,lv);
    d=sim3d.Actor(ActorName=nm); add(world,d);
    createShape(d,'box',[widX,0.07,0.07]);
    d.Translation=[cx, cy-lenY/2+postW+0.04, levZ(lv)-0.04]; d.Color=bmC;

    nm2=sprintf('S%d_BmB%d',sid,lv);
    e=sim3d.Actor(ActorName=nm2); add(world,e);
    createShape(e,'box',[widX,0.07,0.07]);
    e.Translation=[cx, cy+lenY/2-postW-0.04, levZ(lv)-0.04]; e.Color=bmC;

    nm3=sprintf('S%d_Brd%d',sid,lv);
    f=sim3d.Actor(ActorName=nm3); add(world,f);
    createShape(f,'box',[widX-0.10, lenY-postW*2, 0.07]);
    f.Translation=[cx, cy, levZ(lv)]; f.Color=brdC;

    nBoxes = randi([4,7]);
    yPositions = linspace(cy-lenY/2+0.5, cy+lenY/2-0.5, nBoxes);

    for b2=1:nBoxes
        nm4=sprintf('S%d_L%d_Bx%d',sid,lv,b2);
        bx=sim3d.Actor(ActorName=nm4); add(world,bx);

        if sqBoxCount < 10
            % SQUARE box — bigger size, bottom/mid levels get BIG squares
            if lv < nLev
                sqSide = 0.65 + rand()*0.20;   % bottom & mid levels (0.65-0.85m)
            else
                sqSide = 0.40 + rand()*0.10;   % top level (0.40-0.50m)
            end
            createShape(bx,'box',[sqSide, sqSide, sqSide]);
            bx.Translation=[cx+0.05, yPositions(b2), levZ(lv)+0.07+sqSide/2];
            sqBoxCount = sqBoxCount + 1;
        else
            bw=0.48+rand()*0.18;
            bh=0.32+rand()*0.16;
            createShape(bx,'box',[widX*0.62, bw, bh]);
            bx.Translation=[cx+0.05, yPositions(b2), levZ(lv)+0.07+bh/2];
        end
        bx.Color=boxCols(randi(5),:);
    end   % closes for b2
end   % closes for lv

g=sim3d.Actor(ActorName=sprintf('S%d_Border',sid)); add(world,g);
createShape(g,'box',[widX+1.2, lenY+1.2, 0.03]);
g.Translation=[cx, cy, 0.015]; g.Color=yelC;

end   % closes buildShelf function


%% ── BUILD ALL 7 SHELVES ───────────────────────────────────────────────────
function buildAllShelves(world, WH)
for s=1:WH.nShelves
    cx=WH.shelves(s,1); cy=WH.shelves(s,2);
    ln=WH.shelves(s,3); wd=WH.shelves(s,4);
    buildShelf(world,s,cx,cy,ln,wd,WH.shelfH,WH.shelfLevels);
    fprintf('[Phase1]   Shelf %d built.\n',s);
end
end


%% ── DELIVERY TABLES ───────────────────────────────────────────────────────
function buildTables(world, WH)
topC  =[0.82,0.72,0.55];
legC  =[0.22,0.22,0.25];
edgeC =[0.15,0.55,0.25];
yelC  =[0.95,0.85,0.10];
tH=0.90;

for t=1:size(WH.tables,1)
    tx=WH.tables(t,1); ty=WH.tables(t,2);
    tW=WH.tables(t,3); tD=WH.tables(t,4);

    % Table top
    nm=sprintf('Table%d_Top',t);
    a=sim3d.Actor(ActorName=nm); add(world,a);
    createShape(a,'box',[tW,tD,0.07]);
    a.Translation=[tx,ty,tH]; a.Color=topC;

    % Green front edge strip
    nm=sprintf('Table%d_Edge',t);
    e=sim3d.Actor(ActorName=nm); add(world,e);
    createShape(e,'box',[tW,0.07,0.12]);
    e.Translation=[tx,ty-tD/2,tH+0.06]; e.Color=edgeC;

    % 4 legs
    legIdx=0;
    for lx=[-tW/2+0.15, tW/2-0.15]
        for ly=[-tD/2+0.15, tD/2-0.15]
            legIdx=legIdx+1;
            lnm=sprintf('Table%d_Leg%d',t,legIdx);
            l=sim3d.Actor(ActorName=lnm); add(world,l);
            createShape(l,'box',[0.09,0.09,tH-0.07]);
            l.Translation=[tx+lx,ty+ly,(tH-0.07)/2]; l.Color=legC;
        end
    end

    % Yellow safety border
    nm=sprintf('Table%d_Border',t);
    b=sim3d.Actor(ActorName=nm); add(world,b);
    createShape(b,'box',[tW+1.4,tD+1.4,0.03]);
    b.Translation=[tx,ty,0.015]; b.Color=yelC;
end
end


%% ── OFFICE ────────────────────────────────────────────────────────────────
function buildOffice(world, WH)
ox=WH.offCx; oy=WH.offCy;
ow=WH.offW;  od=WH.offD;
wH=4.6; wT=0.18;
wallC=[0.80,0.82,0.84];
floorC=[0.88,0.88,0.86];
% Office floor (slightly different color)
f=sim3d.Actor(ActorName='OffFloor'); add(world,f);
createShape(f,'box',[ow,od,0.02]);
f.Translation=[ox,oy,0.01]; f.Color=floorC;
% 4 walls (with door gap in South wall)
% South wall — solid (no door)
sw=sim3d.Actor(ActorName='OffWallS'); add(world,sw);
createShape(sw,'box',[ow,wT,wH]);
sw.Translation=[ox,oy-od/2,wH/2]; sw.Color=wallC;
% North wall — door gap here (door faces North/outward into warehouse)
doorW=2.4;
pieceW=(ow-doorW)/2;
sw1=sim3d.Actor(ActorName='OffWallNL'); add(world,sw1);
createShape(sw1,'box',[pieceW,wT,wH]);
sw1.Translation=[ox-doorW/2-pieceW/2, oy+od/2, wH/2]; sw1.Color=wallC;
sw2=sim3d.Actor(ActorName='OffWallNR'); add(world,sw2);
createShape(sw2,'box',[pieceW,wT,wH]);
sw2.Translation=[ox+doorW/2+pieceW/2, oy+od/2, wH/2]; sw2.Color=wallC;
df=sim3d.Actor(ActorName='OffDoorTop'); add(world,df);
createShape(df,'box',[doorW,wT,wH*0.25]);
df.Translation=[ox, oy+od/2, wH*0.875]; df.Color=wallC;
% East wall
ew=sim3d.Actor(ActorName='OffWallE'); add(world,ew);
createShape(ew,'box',[wT,od,wH]);
ew.Translation=[ox+ow/2,oy,wH/2]; ew.Color=wallC;
% West wall
ww=sim3d.Actor(ActorName='OffWallW'); add(world,ww);
createShape(ww,'box',[wT,od,wH]);
ww.Translation=[ox-ow/2,oy,wH/2]; ww.Color=wallC;
% 2 desks inside
deskC=[0.82,0.72,0.52];
deskPos=[-2.5,2.5];
for d=1:2
    nm=sprintf('OffDesk%d',d);
    dk=sim3d.Actor(ActorName=nm); add(world,dk);
    createShape(dk,'box',[1.5,0.70,0.05]);
    dk.Translation=[ox+deskPos(d),oy+1.5,0.74]; dk.Color=deskC;
% Monitor
    mnm=sprintf('OffMon%d',d);
    m=sim3d.Actor(ActorName=mnm); add(world,m);
    createShape(m,'box',[0.40,0.04,0.28]);
    m.Translation=[ox+deskPos(d),oy+1.2,0.94]; m.Color=[0.10,0.10,0.12];
end
end


%% ── SAFETY LINES ──────────────────────────────────────────────────────────
function buildSafetyLines(world, WH)
yelC=[0.95,0.85,0.10];
orgC=[0.95,0.42,0.10];

% Horizontal aisle safety lines
yLines=[14,28,42];
for i=1:numel(yLines)
    nm=sprintf('SafeH%d',i);
    a=sim3d.Actor(ActorName=nm); add(world,a);
    createShape(a,'box',[WH.X,0.14,0.02]);
    a.Translation=[0,yLines(i),0.02]; a.Color=yelC;
end

% Vertical center aisle line
vl=sim3d.Actor(ActorName='SafeV'); add(world,vl);
createShape(vl,'box',[0.14,WH.Y,0.02]);
vl.Translation=[-10,WH.Y/2,0.02]; vl.Color=yelC;

% Table delivery zone line
for t=1:size(WH.tables,1)
    nm=sprintf('DelivLine%d',t);
    dl=sim3d.Actor(ActorName=nm); add(world,dl);
    createShape(dl,'box',[WH.tables(t,3)+2,0.14,0.02]);
    dl.Translation=[WH.tables(t,1), ...
        WH.tables(t,2)-WH.tables(t,4)/2-0.8, 0.02];
    dl.Color=orgC;
end

% Fire extinguisher
fe=sim3d.Actor(ActorName='FireExt'); add(world,fe);
createShape(fe,'cylinder',[0.12,0.12,0.55]);
fe.Translation=[+2,28.5,0.28]; fe.Color=[0.85,0.10,0.10];

% Safety bollards (near center)
bollC=[0.95,0.78,0.05];
bpos=[-5,0,5];
for i=1:3
    nm=sprintf('Boll%d',i);
    bl=sim3d.Actor(ActorName=nm); add(world,bl);
    createShape(bl,'cylinder',[0.15,0.15,0.90]);
    bl.Translation=[bpos(i),14.5,0.45]; bl.Color=bollC;
end
end


%% ── CHARGING ZONE ─────────────────────────────────────────────────────────
function buildChargingZone(world, WH)
% Zone floor marker
zf=sim3d.Actor(ActorName='ChargZone'); add(world,zf);
createShape(zf,'box',[13,8,0.02]);
zf.Translation=[0,5,0.01]; zf.Color=[0.68,0.68,0.70];

padC=[0.80,0.10,0.10];
for i=1:WH.nRobots
    px=WH.robotXY(i,1); py=WH.robotXY(i,2);
    nm=sprintf('CPad%d',i);
    a=sim3d.Actor(ActorName=nm); add(world,a);
    createShape(a,'cylinder',[0.85,0.85,0.06]);
    a.Translation=[px,py,0.03]; a.Color=padC;
end
end


%% ── KINOVA MOVO ROBOTS (retracted idle pose, extendable arms) ─────────────
function buildRobots(world, WH)
% Body height fixed = shelfH/2 = 2.75m (unchanged)
% Arms start RETRACTED (small/idle) — use setArmExtension() to extend later

baseC  = [0.08, 0.08, 0.10];
torsoC = [0.13, 0.13, 0.17];
armC   = [0.11, 0.11, 0.15];
gripC  = [0.26, 0.26, 0.30];
snsrC  = [0.05, 0.48, 0.92];

% Retracted (idle) arm segment lengths — small, arm folded near shoulder
retractUpperLen = 0.35;
retractForeLen  = 0.28;

for i = 1:WH.nRobots
    px = WH.robotXY(i,1);
    py = WH.robotXY(i,2);
    t  = sprintf('Ro%d', i);

    % ── Mobile base ──
    a = sim3d.Actor(ActorName=sprintf('%sBas',t)); add(world,a);
    createShape(a,'cylinder',[1.20, 1.20, 0.46]);
    a.Translation = [px, py, 0.23]; a.Color = baseC;

    b = sim3d.Actor(ActorName=sprintf('%sBRn',t)); add(world,b);
    createShape(b,'cylinder',[1.25, 1.25, 0.08]);
    b.Translation = [px, py, 0.44]; b.Color = snsrC;

    % ── Torso: 0.46 → 2.20 ──
    c = sim3d.Actor(ActorName=sprintf('%sTrs',t)); add(world,c);
    createShape(c,'box',[0.66, 0.58, 1.74]);
    c.Translation = [px, py, 1.33]; c.Color = torsoC;

    % ── Shoulder: 2.20 → 2.36 (SHOULDER_Z = 2.28 pivot for arm) ──
    d = sim3d.Actor(ActorName=sprintf('%sShl',t)); add(world,d);
    createShape(d,'box',[1.72, 0.59, 0.16]);
    d.Translation = [px, py, 2.28]; d.Color = baseC;

    % ── LEFT ARM — RETRACTED idle pose (folded near shoulder) ──
    e = sim3d.Actor(ActorName=sprintf('%sLAU',t)); add(world,e);
    createShape(e,'box',[0.20, 0.20, retractUpperLen]);
    e.Translation = [px-0.75, py, 2.36+retractUpperLen/2]; e.Color = armC;

    f = sim3d.Actor(ActorName=sprintf('%sLAF',t)); add(world,f);
    createShape(f,'box',[0.17, 0.17, retractForeLen]);
    f.Translation = [px-0.75, py, 2.36+retractUpperLen+retractForeLen/2]; f.Color = armC;

    g = sim3d.Actor(ActorName=sprintf('%sLGr',t)); add(world,g);
    createShape(g,'box',[0.28, 0.18, 0.20]);
    g.Translation = [px-0.75, py, 2.36+retractUpperLen+retractForeLen+0.10]; g.Color = gripC;

    % ── RIGHT ARM — RETRACTED idle pose ──
    h = sim3d.Actor(ActorName=sprintf('%sRAU',t)); add(world,h);
    createShape(h,'box',[0.20, 0.20, retractUpperLen]);
    h.Translation = [px+0.75, py, 2.36+retractUpperLen/2]; h.Color = armC;

    j = sim3d.Actor(ActorName=sprintf('%sRAF',t)); add(world,j);
    createShape(j,'box',[0.17, 0.17, retractForeLen]);
    j.Translation = [px+0.75, py, 2.36+retractUpperLen+retractForeLen/2]; j.Color = armC;

    k = sim3d.Actor(ActorName=sprintf('%sRGr',t)); add(world,k);
    createShape(k,'box',[0.28, 0.18, 0.20]);
    k.Translation = [px+0.75, py, 2.36+retractUpperLen+retractForeLen+0.10]; k.Color = gripC;

    % ── Head dome: 2.36 → 2.66 ──
    hd = sim3d.Actor(ActorName=sprintf('%sHed',t)); add(world,hd);
    createShape(hd,'cylinder',[0.34, 0.34, 0.30]);
    hd.Translation = [px, py, 2.51]; hd.Color = torsoC;

    % ── LIDAR ring: 2.66 → 2.75 (exact robot top) ──
    lr = sim3d.Actor(ActorName=sprintf('%sLdr',t)); add(world,lr);
    createShape(lr,'cylinder',[0.38, 0.38, 0.09]);
    lr.Translation = [px, py, 2.705]; lr.Color = snsrC;
    a.Mobility=sim3d.utils.MobilityTypes.Movable; b.Mobility=sim3d.utils.MobilityTypes.Movable;
    c.Mobility=sim3d.utils.MobilityTypes.Movable; d.Mobility=sim3d.utils.MobilityTypes.Movable;
    e.Mobility=sim3d.utils.MobilityTypes.Movable; f.Mobility=sim3d.utils.MobilityTypes.Movable;
    g.Mobility=sim3d.utils.MobilityTypes.Movable; h.Mobility=sim3d.utils.MobilityTypes.Movable;
    j.Mobility=sim3d.utils.MobilityTypes.Movable; k.Mobility=sim3d.utils.MobilityTypes.Movable;
    hd.Mobility=sim3d.utils.MobilityTypes.Movable; lr.Mobility=sim3d.utils.MobilityTypes.Movable;

end
end


%% ── ARM EXTEND/RETRACT HELPER (call during Phase 2 task execution) ────────
function setArmExtension(world, WH, robotIdx, side, extendFrac)
% extendFrac: 0.0 = fully retracted (idle) | 1.0 = fully extended (top shelf reach)
% side: 'L' or 'R'
%
% USAGE (Phase 2):
%   setArmExtension(world, WH, 3, 'L', 1.0);   % Robot 3 left arm reach top shelf
%   setArmExtension(world, WH, 3, 'L', 0.0);   % retract back to idle

px = WH.robotXY(robotIdx,1);
py = WH.robotXY(robotIdx,2);
t  = sprintf('Ro%d', robotIdx);
xOff = 0.75 * (side=='R') - 0.75*(side=='L');

SHOULDER_Z = 2.36;
retractUpperLen = 0.35;  retractForeLen = 0.28;
extendUpperLen  = 1.55;  extendForeLen  = 1.30;
gripH = 0.20;

upperLen = retractUpperLen + extendFrac*(extendUpperLen-retractUpperLen);
foreLen  = retractForeLen  + extendFrac*(extendForeLen -retractForeLen);

au = world.Actors.(sprintf('%sLAU',t)); if side=='R', au = world.Actors.(sprintf('%sRAU',t)); end
af = world.Actors.(sprintf('%sLAF',t)); if side=='R', af = world.Actors.(sprintf('%sRAF',t)); end
gr = world.Actors.(sprintf('%sLGr',t)); if side=='R', gr = world.Actors.(sprintf('%sRGr',t)); end

au.Scale = [1, 1, upperLen/retractUpperLen];
au.Translation = [px+xOff, py, SHOULDER_Z+upperLen/2];

af.Scale = [1, 1, foreLen/retractForeLen];
af.Translation = [px+xOff, py, SHOULDER_Z+upperLen+foreLen/2];

gr.Translation = [px+xOff, py, SHOULDER_Z+upperLen+foreLen+gripH/2];
end