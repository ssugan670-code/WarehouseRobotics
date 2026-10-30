function [xyz,G] = helperTargetPose(WH,target)
%HELPERTARGETPOSE Target centre derived only from the shared shelf geometry.
assert(target.level>=1 && target.level<=3,'helperTargetPose:Level','Target level must be 1, 2, or 3.');
d=WH.shelves(target.shelf,:); G=helperShelfBays(d);
k=helperTargetSlot(G,target.slot);
xyz=[G.faceX-G.targetDX, G.slotY(k), G.deckTop(target.level)+G.targetDims(3)/2];
end
