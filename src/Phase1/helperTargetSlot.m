function k=helperTargetSlot(G,requestedSlot)
%HELPERTARGETSLOT Resolve the project-defined middle allowed pick slot.
% The caller can supply a valid explicit slot.  The automatic setting keeps
% the project's original middle-of-rack target policy; visibility is now
% diagnosed from the camera block's reported pose rather than guessed by
% silently relocating the task target.
if nargin<2 || isempty(requestedSlot)
    candidates=G.targetSlots(:).';
    [~,index]=min(abs(G.slotY(candidates)-mean(G.slotY)));
    k=candidates(index);
else
    k=requestedSlot;
    assert(ismember(k,G.targetSlots),'helperTargetSlot:Slot', ...
        'Slot %d is too close to a panel. Allowed slots: %s',k,mat2str(G.targetSlots));
end
end
