function setupFleetScene(Actor,World)
%SETUPFLEETSCENE  Phase 3 warehouse (layout v2): floor, walls, 8 shelves (A/B sides, static stock on both), 3 packing stations,
%   staging area (3 dark mats, 14 orange wooden stands), 6 robot bays, slot position labels, carton pool, robot paint marks.
C = evalin('base','Phase3Cfg'); WH = C.WH; P3 = C.P3; F = evalin('base','Phase3Fleet'); N = F.N;
localBox(World,Actor,'WarehouseFloor',[0 WH.Y/2 -.075],[WH.X WH.Y .15],[.72 .72 .70]);
wall=[.31 .34 .40]; z=WH.Z/2;
localBox(World,Actor,'WallSouth',[0 0 z],[WH.X+.4 .35 WH.Z],wall);
localBox(World,Actor,'WallNorth',[0 WH.Y z],[WH.X+.4 .35 WH.Z],wall);
localBox(World,Actor,'WallWest',[-WH.X/2 WH.Y/2 z],[.35 WH.Y WH.Z],wall);
localBox(World,Actor,'WallEast',[WH.X/2 WH.Y/2 z],[.35 WH.Y WH.Z],wall);
static = false(numel(P3.slots),1); static(P3.static) = true;
for k=1:size(WH.shelves,1)
    inShelf = find([P3.slots.shelf]==k);
    localShelf(World,Actor,k,WH.shelves(k,:),P3.slots(inShelf),static(inShelf),P3.decor);
end
for k=1:size(WH.tables,1), localTable(World,Actor,k,WH.tables(k,:)); end
localShelfLabels(World,Actor,WH);
if ~strcmp(P3.labelMode,'off'), localPositionLabels(World,Actor,WH,P3); end
% staging area: dark mats, orange wooden stands (the cartons stand on them), number plates D01..D14 on the aisle face of every stand
for m=1:size(WH.mats,1), localBox(World,Actor,sprintf('StagingMat%d',m),[WH.mats(m,1:2) 0.012],[WH.mats(m,3) WH.mats(m,4) 0.024],[0.30 0.30 0.32]); end
for i=1:size(WH.stands,1)
    sx=WH.stands(i,1); sy=WH.stands(i,2); h=WH.standHeight; sz=WH.standSize;
    localBox(World,Actor,sprintf('Stand%02d',i),[sx sy h/2],[sz sz h],[0.95 0.48 0.05]);
    if ~strcmp(P3.labelMode,'off')
        localBox(World,Actor,sprintf('StandPlate%02d',i),[sx sy+sz/2+0.004 0.17],[0.30 0.008 0.13],[1 1 1]);
        txt=sprintf('d%02d',i); nd=numel(txt); dw=0.075;
        for di=1:nd
            localGlyph(World,Actor,sprintf('StandNum%02d_%d',i,di),txt(di),[sx sy+sz/2+0.0105 0.17],[-1 0],[0 0],(di-(nd+1)/2)*dw*1.25,dw*0.8,0.10,'wall');
        end
    end
end
for k=1:numel(P3.robots)
    r=P3.robots(k); localCylinder(World,Actor,sprintf('HomePad%d',k),[r.home .025],[WH.padDiameter WH.padDiameter .05],r.colour);
end
% a tiny dummy actor named TargetBox: the Robot block's (unused) native-grasp input wants an actor of that name; it is parked outside the walls
a=sim3d.Actor(ActorName='TargetBox'); add(World,a,Actor); createShape(a,'box',[0.05 0.05 0.05]); a.Translation=localRhrToUnreal([-40 -5 0.03]); a.Color=[0.5 0.5 0.5];
% carton pool (cartons that are not in the warehouse yet wait outside the south wall)
dims=C.TargetDimensions;
for j=1:P3.nCartons
    xyz=P3.cartons(j).xyz;
    box=sim3d.Actor(ActorName=sprintf('Carton%02d',j)); add(World,box,Actor);
    createShape(box,'box',dims); box.Mobility=sim3d.utils.MobilityTypes.Movable;
    if isfield(C,'UnrealPhysics') && C.UnrealPhysics
        props={'Physics',true;'Gravity',true;'Mass',4;'Friction',0.7;'Restitution',0.05;'PreciseContacts',true};
        for kk=1:size(props,1), try, box.(props{kk,1})=props{kk,2}; catch, end, end
    end
    box.Translation=localRhrToUnreal(xyz); box.Color=P3.cartonColour;
    try, box.Rotation=[0 0 -P3.cartons(j).yaw]; catch, end      % Unreal yaw = -(right-handed yaw)
end
% robot identification: a painted patch on the lower back = 3 thin slats that follow the curve of the chassis (moved by Transform Set blocks)
mr=0.338; mz=0.20; th=[-0.2618 0 0.2618];
if isfield(C,'MarkerRadius'), mr=C.MarkerRadius; end
if isfield(C,'MarkerZ'), mz=C.MarkerZ; end
for k=1:N
    r=P3.robots(k);
    for q=1:3
        m=sim3d.Actor(ActorName=sprintf('RobotMarker%d_%d',k,q)); add(World,m,Actor);
        createShape(m,'box',[0.012 0.09 0.20]); m.Mobility=sim3d.utils.MobilityTypes.Movable;
        try, m.Physics=false; m.Gravity=false; catch, end
        f=pi+th(q); m.Translation=localRhrToUnreal([r.home(1)+mr*cos(f) r.home(2)+mr*sin(f) mz]); m.Color=r.colour;
        try, m.Rotation=[0 0 -f]; catch, end
    end
end
end

function localShelf(World,parent,sid,d,slots,static,decor)
% Shelf with open panels and 3 decks.  Both faces are service faces: A = west, B = east.  A face's stock is exactly the static-stock slots of the
% database (the other free slots stay empty for the robots); positions that are not in the database get random decor.
cx=d(1); cy=d(2); lenY=d(3); widX=d(4); G=helperShelfBays(d);
blue=[.14 .27 .55]; deckC=[.50 .51 .53]; plate=[.72 .68 .12];
localBox(World,parent,sprintf('S%dFloor',sid),[cx cy .006],[widX+1 lenY+1 .012],plate);
for i=1:numel(G.panelY), localBox(World,parent,sprintf('S%dPanel%d',sid,i),[cx G.panelY(i) G.H/2],[widX G.panelT G.H],blue); end
for level=1:3, localBox(World,parent,sprintf('S%dDeck%d',sid,level),[cx cy G.deckTop(level)-G.deckT/2],[widX-.02 lenY G.deckT],deckC); end
decorP=min(0.55,1.2*decor); rs=RandStream('mt19937ar','Seed',100+sid); pal=[.76 .60 .38;.86 .73 .53;.93 .92 .88;.58 .68 .78]; pick=[1 1 2 2 3 4];
nK=numel(G.slotY); inDb=false(2,3,nK); isSt=false(2,3,nK);
for i=1:numel(slots)
    kk=find(abs(G.slotY-slots(i).xyz(2))<1e-6,1); if isempty(kk), continue, end
    sd=1+(slots(i).n(1)<0); inDb(sd,slots(i).level,kk)=true; isSt(sd,slots(i).level,kk)=static(i);
end
for sd=1:2
    sgn=3-2*sd;                          % +1 = B (east), -1 = A (west)
    for level=1:3
        for slot=1:nK
            r1=rand(rs); r2=rand(rs); r3=rand(rs); r4=rand(rs); r5=rand(rs); r6=rand(rs);
            if ~G.slotClear(slot), continue, end
            if inDb(sd,level,slot), if ~isSt(sd,level,slot), continue, end
            elseif r1>=decorP, continue, end
            bx=.40+.15*r2; by=.50+.18*r3; bh=.28+.16*r4; inset=G.rowInset+.15*r5; yc=G.slotY(slot)+.06*(r6-.5);
            xc=cx+sgn*(widX/2-inset-bx/2);
            localBox(World,parent,sprintf('S%dB%d_%d_%d',sid,sd,level,slot),[xc yc G.deckTop(level)+bh/2],[bx by bh],pal(pick(1+mod(floor(r2*100),numel(pick))),:));
        end
    end
end
end

function localPositionLabels(World,parent,WH,P3)
% small number of every position (P01, P02 ... counted along the shelf, the same on all three levels) on the aisle floor in front of the shelf, both sides.
% digits are upright for someone standing in the aisle and looking at the shelf.  labelMode 'full' adds the letter P (more actors), 'num' = digits only.
done=containers.Map('KeyType','char','ValueType','logical');
for i=1:numel(P3.slots)
    s=P3.slots(i); if ~strcmp(s.role,'store'), continue, end
    key=sprintf('%s_%d',s.gname,s.pos); if isKey(done,key), continue, end
    done(key)=true;
    n=s.n; faceP=s.xyz(1)+n(1)*0.275;           % face x of this side
    c=[faceP+n(1)*0.78 s.xyz(2) 0.006];         % just outside the yellow floor plate (it reaches 0.5 m off the face), so the numbers stay visible
    uv=-n; rv=[uv(2) -uv(1)];                   % up = toward the shelf; right = right-hand side when looking along up
    txt=sprintf('%02d',s.pos); if strcmp(P3.labelMode,'full'), txt=['P' txt]; end %#ok<AGROW>
    nd=numel(txt); dw=0.15;
    for di=1:nd
        localGlyph(World,parent,sprintf('Pos_%s_%d_%d',s.gname,s.pos,di),txt(di),c,rv,uv,(di-(nd+1)/2)*dw*1.25,dw*0.8,0.26,'floor');
    end
end
end

function localGlyph(World,parent,name,ch,c,rv,uv,u0,w,h,kind)
% seven-segment character.  rv = unit "right" vector (x y), uv = unit "up" vector on the floor ('floor') ; for 'wall' the up direction is +z.
if ch>='0' && ch<='9', pat=localPat(ch-'0'); else, pat=localPatCh(ch); end
t=0.16*h; if t>0.30*w, t=0.30*w; end
pos=[0 h/2-t/2 w t; -w/2+t/2 h/4 t h/2; w/2-t/2 h/4 t h/2; 0 0 w t; -w/2+t/2 -h/4 t h/2; w/2-t/2 -h/4 t h/2; 0 -h/2+t/2 w t];
for q=1:7
    if ~pat(q), continue, end
    pu=u0+pos(q,1); pv=pos(q,2); sw=pos(q,3); sh=pos(q,4);
    if strcmp(kind,'floor')
        ctr=[c(1)+rv(1)*pu+uv(1)*pv  c(2)+rv(2)*pu+uv(2)*pv  c(3)];
        dim=[abs(rv(1))*sw+abs(uv(1))*sh  abs(rv(2))*sw+abs(uv(2))*sh  0.008];
    else
        ctr=[c(1)+rv(1)*pu  c(2)+rv(2)*pu  c(3)+pv];
        dim=[abs(rv(1))*sw+abs(rv(2))*0.006  abs(rv(2))*sw+abs(rv(1))*0.006  sh];
    end
    localLabelBox(World,parent,sprintf('%s_s%d',name,q),ctr,dim,[0.05 0.05 0.08]);
end
end
function p=localPat(n)
seg=[1 1 1 0 1 1 1; 0 0 1 0 0 1 0; 1 0 1 1 1 0 1; 1 0 1 1 0 1 1; 0 1 1 1 0 1 0; 1 1 0 1 0 1 1; 1 1 0 1 1 1 1; 1 0 1 0 0 1 0; 1 1 1 1 1 1 1; 1 1 1 1 0 1 1];
p=seg(n+1,:);
end
function p=localPatCh(ch)
switch ch
    case 'A', p=[1 1 1 1 1 1 0];
    case 'b', p=[0 1 0 1 1 1 1];
    case 'd', p=[0 0 1 1 1 1 1];
    case 'P', p=[1 1 1 1 1 0 0];
    otherwise, p=[0 0 0 1 0 0 0];
end
end

function localTable(World,parent,id,d)
cx=d(1); cy=d(2); w=d(3); depth=d(4); top=.82;
localBox(World,parent,sprintf('Table%dTop',id),[cx cy top],[w depth .10],[.46 .31 .16]);
for ix=[-1 1]
    for iy=[-1 1]
        localBox(World,parent,sprintf('Table%dLeg%d%d',id,ix+2,iy+2), ...
            [cx+ix*(w/2-.16) cy+iy*(depth/2-.16) .36],[.13 .13 .72],[.12 .12 .14]);
    end
end
end
function localBox(World,parent,name,t,dim,c)
a=sim3d.Actor(ActorName=regexprep(name,'[^A-Za-z0-9_]','_')); add(World,a,parent); createShape(a,'box',dim); a.Translation=localRhrToUnreal(t); a.Color=c;
end
function localCylinder(World,parent,name,t,dim,c)
a=sim3d.Actor(ActorName=regexprep(name,'[^A-Za-z0-9_]','_')); add(World,a,parent); createShape(a,'cylinder',dim); a.Translation=localRhrToUnreal(t); a.Color=c;
end
function unreal=localRhrToUnreal(rhr)
% sim3d.Actor poses use Unreal coordinates. Navigation and Simulation 3D
% Robot use the Robotics System Toolbox right-handed world: (x,-y,z).
unreal=[rhr(1) -rhr(2) rhr(3)];
end

function localShelfLabels(World,parent,WH)
% Shelf number plates (shelf number = row of WH.shelves, the number the logs and command window use):
%   * white plate + black digits on the OUTER face of BOTH end panels (south and north), robust to where panelY sits
%   * large digits painted on the floor beside BOTH long sides (front and back aisle), readable from the top camera
for k=1:size(WH.shelves,1)
    d=WH.shelves(k,:); g=helperShelfBays(d);
    s=min(0.55,0.45*d(4)); zc=0.65*g.H; txt=sprintf('%d',k); nd=numel(txt);
    for side=[-1 1]                         % -1 = south end panel, +1 = north end panel
        sn='S'; if side>0, sn='N'; end
        y=g.panelY(1+(side>0)*(numel(g.panelY)-1)); pt=0.05;     % plate depth 5 cm: touches the panel whether panelY is its centre or inner face
        yc=y+side*(g.panelT/2+pt/2); yf=yc+side*(pt/2+0.006);
        localLabelBox(World,parent,sprintf('Shelf%dPlate%s',k,sn),[d(1) yc zc],[s pt s],[1 1 1]);
        % side letters and level marks on the end panel (seen from outside: A = west face on the left, B = east face on the right; L1..L3 at the height of each level)
        rvv=[-side 0]; yw=y+side*(g.panelT/2+0.006); zL=g.deckTop+0.18; offA=0.20*d(4); offL=0.39*d(4);
        localPix(World,parent,sprintf('Shelf%dLetA%s',k,sn),'A',[d(1)-offA yw zL(2)],rvv,[0 0],0.06,'wall',[1 1 1]);
        localPix(World,parent,sprintf('Shelf%dLetB%s',k,sn),'B',[d(1)+offA yw zL(2)],rvv,[0 0],0.06,'wall',[1 1 1]);
        for lv=1:3
            localPix(World,parent,sprintf('Shelf%dLvA%d%s',k,lv,sn),sprintf('L%d',lv),[d(1)-offL yw zL(lv)],rvv,[0 0],0.035,'wall',[1 1 1]);
            localPix(World,parent,sprintf('Shelf%dLvB%d%s',k,lv,sn),sprintf('L%d',lv),[d(1)+offL yw zL(lv)],rvv,[0 0],0.035,'wall',[1 1 1]);
        end
        dw=s*0.8/nd;
        for di=1:nd
            u0=(di-(nd+1)/2)*dw;            % viewer coordinates: right = +x at the south panel, -x at the north panel
            localDigit(World,parent,sprintf('Shelf%dD%s_%d',k,sn,di),str2double(txt(di)),[d(1) yf zc],'wall',side,u0,dw*0.8,s*0.7);
        end
    end
    fs=0.9; dw=fs*0.55;                     % floor digits, centred along the shelf, 1.55 m off each long face (outside the yellow plate and the position numbers)
    for sd=[-1 1]
        sn='F'; if sd>0, sn='B'; end
        cx=d(1)+sd*(d(4)/2+1.55);
        for di=1:nd
            u0=(di-(nd+1)/2)*dw*1.25;
            localDigit(World,parent,sprintf('Shelf%dFloor%s_%d',k,sn,di),str2double(txt(di)),[cx d(2) 0.006],'floor',1,u0,dw,fs);
        end
        % side letter (capital, small) just below the number: west aisle = A, east aisle = B
        letter='B'; if sd<0, letter='A'; end
        localPix(World,parent,sprintf('Shelf%dSide%s',k,sn),letter,[cx d(2)-1.1 0.006],[1 0],[0 1],0.09,'floor',[0.05 0.05 0.08]);
    end
end
end
function localPix(World,parent,name,str,c,rv,uv,cs,kind,col)
% text from a 3x5 pixel font (A B L 1 2 3), vertical runs merged into boxes.  c = centre, rv = right vector (x y), uv = up vector (floor kind); wall kind: up = +z
F=struct('A',[0 1 0;1 0 1;1 1 1;1 0 1;1 0 1],'B',[1 1 0;1 0 1;1 1 0;1 0 1;1 1 0],'L',[1 0 0;1 0 0;1 0 0;1 0 0;1 1 1], ...
    'x1',[0 1 0;1 1 0;0 1 0;0 1 0;1 1 1],'x2',[1 1 1;0 0 1;1 1 1;1 0 0;1 1 1],'x3',[1 1 1;0 0 1;1 1 1;0 0 1;1 1 1]);
nc=numel(str); tot=4*nc-1;
for q=1:nc
    ch=str(q); if ch>='0' && ch<='9', ch=['x' ch]; end
    pat=F.(ch);
    for cc=1:3
        r=1;
        while r<=5
            if ~pat(r,cc), r=r+1; continue, end
            r2=r; while r2<5 && pat(r2+1,cc), r2=r2+1; end
            pu=((q-1)*4+(cc-1)-(tot-1)/2)*cs; pv=(3-(r+r2)/2)*cs; hh=(r2-r+1)*cs;
            if strcmp(kind,'wall')
                ctr=[c(1)+rv(1)*pu c(2) c(3)+pv]; dim=[cs 0.012 hh];
            else
                ctr=[c(1)+rv(1)*pu+uv(1)*pv c(2)+rv(2)*pu+uv(2)*pv 0.006]; dim=[cs hh 0.012];
            end
            localLabelBox(World,parent,sprintf('%s_c%d_p%d_%d',name,q,cc,r),ctr,dim,col);
            r=r2+1;
        end
    end
end
end
function localDigit(World,parent,name,n,c,kind,side,u0,w,h)
seg=[1 1 1 0 1 1 1; 0 0 1 0 0 1 0; 1 0 1 1 1 0 1; 1 0 1 1 0 1 1; 0 1 1 1 0 1 0; 1 1 0 1 0 1 1; 1 1 0 1 1 1 1; 1 0 1 0 0 1 0; 1 1 1 1 1 1 1; 1 1 1 1 0 1 1];
on=seg(n+1,:); t=0.14*h;
% segments a(top) b(upper-left) c(upper-right) d(middle) e(lower-left) f(lower-right) g(bottom): [u v width height]
pos=[0 h/2-t/2 w t; -w/2+t/2 h/4 t h/2; w/2-t/2 h/4 t h/2; 0 0 w t; -w/2+t/2 -h/4 t h/2; w/2-t/2 -h/4 t h/2; 0 -h/2+t/2 w t];
for q=1:7
    if ~on(q), continue, end
    u=u0+pos(q,1); v=pos(q,2); sw=pos(q,3); sh=pos(q,4);
    if strcmp(kind,'wall')
        localLabelBox(World,parent,sprintf('%s_s%d',name,q),[c(1)-side*u c(2) c(3)+v],[sw 0.012 sh],[0.05 0.05 0.08]);
    else
        localLabelBox(World,parent,sprintf('%s_s%d',name,q),[c(1)+u c(2)+v 0.006],[sw sh 0.012],[0.05 0.05 0.08]);
    end
end
end
function localLabelBox(World,parent,name,t,dim,col)
a=sim3d.Actor(ActorName=regexprep(name,'[^A-Za-z0-9_]','_')); add(World,a,parent);
createShape(a,'box',dim);                     % [x y z] extents
a.Translation=[t(1) -t(2) t(3)]; a.Color=col;
end
