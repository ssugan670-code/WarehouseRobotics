function G = helperShelfBays(d)
%HELPERSHELFBAYS Shared geometry for the open-panel shelf.
cx=d(1); cy=d(2); lenY=d(3); widX=d(4);
G.H=2.45;
G.deckTop=[0.35 1.25 2.15];
G.deckT=0.05;
G.panelT=0.08;
G.panelPitch=5.5;
G.slotPitch=1.4;
G.rowInset=0.05;
nPB=max(1,round(lenY/G.panelPitch));
yEdge=cy-lenY/2+(0:nPB)*(lenY/nPB);
G.panelY=yEdge; G.panelY(1)=yEdge(1)-G.panelT/2; G.panelY(end)=yEdge(end)+G.panelT/2;
nS=max(2,round(lenY/G.slotPitch));
G.slotY=cy-lenY/2+((1:nS)-0.5)*(lenY/nS);
dPanel=min(abs(G.slotY(:)-yEdge(:).'),[],2).';
G.slotClear=dPanel>0.50;
G.targetSlots=find(dPanel>0.60);
G.targetDims=[0.45 0.50 0.36];
G.faceX=cx+widX/2;
G.targetDX=G.rowInset+G.targetDims(1)/2;
end
