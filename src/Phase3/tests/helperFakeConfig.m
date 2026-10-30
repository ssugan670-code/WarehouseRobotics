function C=helperFakeConfig(nStored,nDock,nPool,seed)
% helperFakeConfig()  small test world (12 boxes);  helperFakeConfig(30,10,54,seed) = like the real start: 30 stored, 10 on the stands, 54 boxes, no static stock
big = nargin >= 3; if ~big, seed = 7; end
WH=makePhase3Layout(true);
names={'blue','red','green','black','pink','yellow'}; cols=rand(6,3);
dims=[0.45 0.50 0.36];
slots=p3BuildSlots(WH,dims,0.34,0.35,0.53,@(xy) true);
rand('twister',seed); isStore=strcmp({slots.role},'store'); static=find(isStore & rand(1,numel(slots))<0.45*(~big));
free=find(isStore); free=setdiff(free,static); stored=free([5 30 60 100]); dockIdx=find(strcmp({slots.role},'dock'));
M=12; nS=4; nD=4; if big, pp=free(randperm(numel(free))); stored=pp(1:nStored); M=nPool; nS=nStored; nD=nDock; end
cartons=struct('id',{},'slot',{},'state',{},'xyz',{},'park',{},'yaw',{});
for j=1:M
  park=[-3+0.9*mod(j-1,4) 30+0.9*floor((j-1)/4) -3]; slot=0; st='RESERVE'; xyz=park; yw=0;
  if j<=nS, slot=stored(j); st='STORED'; xyz=slots(slot).xyz; yw=slots(slot).boxYaw; elseif j<=nS+nD, slot=dockIdx(j-nS); st='DOCK'; xyz=slots(slot).xyz; yw=slots(slot).boxYaw; end
  cartons(j)=struct('id',j,'slot',slot,'state',st,'xyz',xyz,'park',park,'yaw',yw);
end
robots=struct('id',{},'name',{},'colour',{},'home',{}); for k=1:6, robots(k)=struct('id',k,'name',names{k},'colour',cols(k,:),'home',WH.robotXY(k,:)); end
C.WH=WH;
C.P3=struct('robots',robots,'slots',slots,'static',static,'cartons',cartons,'nCartons',M,'dockShelf',0,'stands',WH.stands,'cartonColour',[.9 .05 .75],'missing',[],'labelMode','off');
C.TargetDimensions=dims;
for t=1:3
  C.Stations(t)=struct('id',t,'name',sprintf('Packing Station %d',t),'side','south','yaw',pi/2,'slotXYZ',[WH.tables(t,1)+[0 .8 -.8 1.6 -1.6 2.4 -2.4 3.2 -3.2].' 65.4*ones(9,1) 1.07*ones(9,1)],'approach',[WH.tables(t,1)+[0 .8 -.8 1.6 -1.6 2.4 -2.4 3.2 -3.2].' 64.5*ones(9,1)],'centre',WH.tables(t,1:2));
end
end
