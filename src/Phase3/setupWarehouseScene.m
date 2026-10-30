function setupWarehouseScene(Actor,World)
%SETUPWAREHOUSESCENE Create the static warehouse in its centred-X world frame.
P3=phase3Paths(); S=load(P3.layout,'WH'); WH=S.WH;
cfg=WarehouseRobotPhase2Config();
localBox(World,Actor,'WarehouseFloor',[0 WH.Y/2 -.075],[WH.X WH.Y .15],[.72 .72 .70]);
wall=[.31 .34 .40]; z=WH.Z/2;
localBox(World,Actor,'WallSouth',[0 0 z],[WH.X+.4 .35 WH.Z],wall);
localBox(World,Actor,'WallNorth',[0 WH.Y z],[WH.X+.4 .35 WH.Z],wall);
localBox(World,Actor,'WallWest',[-WH.X/2 WH.Y/2 z],[.35 WH.Y WH.Z],wall);
localBox(World,Actor,'WallEast',[WH.X/2 WH.Y/2 z],[.35 WH.Y WH.Z],wall);
% Each shelf that hosts one of the three cartons keeps that carton's slot free of stock; the others get the usual fill.
shelfTarget=cell(size(WH.shelves,1),1); for k=1:size(WH.shelves,1), shelfTarget{k}=cfg.Target; end
for b=1:numel(cfg.Boxes), shelfTarget{cfg.Boxes(b).shelf}=struct('shelf',cfg.Boxes(b).shelf,'level',cfg.Boxes(b).level,'slot',[]); end
for k=1:size(WH.shelves,1)
    buildShelf(World,Actor,k,WH.shelves(k,:),shelfTarget{k},cfg.ShelfFill);
end
for k=1:size(WH.tables,1), localTable(World,Actor,k,WH.tables(k,:)); end
if ~isfield(cfg,'ShelfLabels') || cfg.ShelfLabels, localShelfLabels(World,Actor,WH); end   % shelf number plates on both end panels of every shelf
localCylinder(World,Actor,'HomePad',[cfg.HomeXY .025],[.64 .64 .05],[.12 .55 .18]);

% The three cartons come from the same shelf geometry as the panels (stock-free slot).  Box 1 stays magenta.
names={'TargetBox','TargetBox2','TargetBox3'};
for b=1:numel(cfg.Boxes)
    xyz=cfg.Boxes(b).xyz;
    if b==1, xyz=xyz+[reshape(cfg.Test.SpawnOffset,1,2) 0]; end
    if any(b==cfg.Test.MissingBox), xyz=[cfg.Test.MissingXY 0.5*cfg.Boxes(b).dims(3)+0.001]; end   % test hook: carton not in its slot
    box=sim3d.Actor(ActorName=names{b}); add(World,box,Actor);
    createShape(box,'box',cfg.Boxes(b).dims);
    box.Mobility=sim3d.utils.MobilityTypes.Movable;
    if isfield(cfg,'UnrealPhysics') && cfg.UnrealPhysics
        % Unreal rigid-body physics for the carton (gravity, contact with deck/table, friction). Each property is
        % guarded: an unsupported name only prints a warning and the controller's own contact solver still protects the box.
        props={'Physics',true;'Gravity',true;'Mass',4;'Friction',0.7;'Restitution',0.05;'PreciseContacts',true};
        for kk=1:size(props,1)
            try, box.(props{kk,1})=props{kk,2}; catch ME, if b==1, fprintf('[PHYS] property %s not set: %s\n',props{kk,1},ME.message); end, end
        end
    end
    box.Translation=localRhrToUnreal(xyz); box.Color=cfg.BoxColors(b,:);
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
a=sim3d.Actor(ActorName=name); add(World,a,parent); createShape(a,'box',dim); a.Translation=localRhrToUnreal(t); a.Color=c;
end
function localCylinder(World,parent,name,t,dim,c)
a=sim3d.Actor(ActorName=name); add(World,a,parent); createShape(a,'cylinder',dim); a.Translation=localRhrToUnreal(t); a.Color=c;
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
        y=g.panelY(1+(side>0)); pt=0.05;     % plate depth 5 cm: touches the panel whether panelY is its centre or inner face
        yc=y+side*(g.panelT/2+pt/2); yf=yc+side*(pt/2+0.006);
        localLabelBox(World,parent,sprintf('Shelf%dPlate%s',k,sn),[d(1) yc zc],[s pt s],[1 1 1]);
        dw=s*0.8/nd;
        for di=1:nd
            u0=(di-(nd+1)/2)*dw;            % viewer coordinates: right = +x at the south panel, -x at the north panel
            localDigit(World,parent,sprintf('Shelf%dD%s_%d',k,sn,di),str2double(txt(di)),[d(1) yf zc],'wall',side,u0,dw*0.8,s*0.7);
        end
    end
    fs=0.9; dw=fs*0.55;                     % floor digits, centred along the shelf, 0.9 m off each long face
    for sd=[-1 1]
        sn='F'; if sd>0, sn='B'; end
        cx=d(1)+sd*(d(4)/2+0.9);
        for di=1:nd
            u0=(di-(nd+1)/2)*dw*1.25;
            localDigit(World,parent,sprintf('Shelf%dFloor%s_%d',k,sn,di),str2double(txt(di)),[cx d(2) 0.006],'floor',1,u0,dw,fs);
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
        localLabelBox(World,parent,sprintf('%s_s%d',name,q),[c(1)+u c(2)+v c(3)],[sw sh 0.012],[0.05 0.05 0.08]);
    end
end
end
function localLabelBox(World,parent,name,t,dim,col)
a=sim3d.Actor(ActorName=name); add(World,a,parent);
createShape(a,'box',dim);                     % [x y z] extents
a.Translation=[t(1) -t(2) t(3)]; a.Color=col;
end
