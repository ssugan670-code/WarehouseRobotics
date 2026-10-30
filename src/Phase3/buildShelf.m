function buildShelf(World,parent,sid,d,target,fill)
%BUILDSHELF Open three-level panel shelf with sparse accessible stock.
if nargin<6 || isempty(fill), fill=.55; end
cx=d(1); cy=d(2); lenY=d(3); widX=d(4); G=helperShelfBays(d);
blue=[.14 .27 .55]; deckC=[.50 .51 .53]; plate=[.72 .68 .12];
localBox(World,parent,sprintf('S%dFloor',sid),[cx cy .006],[widX+1 lenY+1 .012],plate);
for i=1:numel(G.panelY)
    localBox(World,parent,sprintf('S%dPanel%d',sid,i),[cx G.panelY(i) G.H/2],[widX G.panelT G.H],blue);
end
for level=1:3
    localBox(World,parent,sprintf('S%dDeck%d',sid,level),[cx cy G.deckTop(level)-G.deckT/2],[widX-.02 lenY G.deckT],deckC);
end
rng(100+sid); pal=[.76 .60 .38;.86 .73 .53;.93 .92 .88;.58 .68 .78]; pick=[1 1 2 2 3 4];
for side=[-1 1]
    for level=1:3
        for slot=1:numel(G.slotY)
            if ~G.slotClear(slot) || localIsTarget(target,sid,side,level,slot,G) || rand>=fill, continue; end
            bx=.40+.15*rand; by=.50+.18*rand; bh=.28+.16*rand; inset=G.rowInset+.15*rand;
            xc=cx+side*(widX/2-inset-bx/2); yc=G.slotY(slot)+.06*(rand-.5);
            localBox(World,parent,sprintf('S%dB%d_%d_%d',sid,side>0,level,slot), ...
                [xc yc G.deckTop(level)+bh/2],[bx by bh],pal(pick(randi(numel(pick))),:));
        end
    end
end
end
function tf=localIsTarget(T,sid,side,level,slot,G)
tf=false;
if isempty(T) || side~=1 || T.shelf~=sid || T.level~=level, return; end
k=helperTargetSlot(G,T.slot);
tf=slot==k;
end
function localBox(World,parent,name,t,dim,c)
a=sim3d.Actor(ActorName=name); add(World,a,parent); createShape(a,'box',dim);
% Shelf geometry is specified in the navigation/right-handed warehouse frame.
% sim3d.Actor is an Unreal actor, whose matching world pose is (x,-y,z).
a.Translation=[t(1) -t(2) t(3)]; a.Color=c;
end
