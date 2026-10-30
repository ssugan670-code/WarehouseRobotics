function slots = p3BuildSlots(WH,dims,R,frontClear,liftBase,isFree)
%P3BUILDSLOTS  All accessible slots of the Phase 3 layout (shared by WarehousePhase3Config and the headless test).
%   Shelf slots <shelf><side>-L<level>-P<pos>  (side A = west face, B = east face), staging stands D01..D14.
%   isFree(xy) -> true when the staging point [x y] (centred-X frame) is free in the navigation map.
slots = struct('num',{},'id',{},'shelf',{},'side',{},'grp',{},'gname',{},'level',{},'pos',{},'role',{},'xyz',{},'staging',{}, ...
    'dock',{},'lift',{},'faceX',{},'n',{},'faceD',{},'yaw',{},'boxYaw',{});
nSh = size(WH.shelves,1); n = 0;
stage = 2.00; dockDist = R + frontClear;
for s = 1:nSh
    G = helperShelfBays(WH.shelves(s,:)); wid = WH.shelves(s,4); cxs = WH.shelves(s,1);
    for sd = 1:2
        if sd==1, nn = [1 0]; faceP = cxs + wid/2; letter = 'B'; yaw = pi; else, nn = [-1 0]; faceP = cxs - wid/2; letter = 'A'; yaw = 0; end   % B = east face, A = west face (from the south end of a shelf: A on the left, B on the right)
        for L = 1:3
            rank = 0;
            for k = G.targetSlots(:).'
                rank = rank+1;
                stg = [faceP+nn(1)*stage G.slotY(k)];
                if ~isFree(stg), continue, end
                n = n+1;
                xyz = [faceP-nn(1)*G.targetDX G.slotY(k) G.deckTop(L)+dims(3)/2];
                slots(n) = struct('num',n,'id',sprintf('%d%s-L%d-P%02d',s,letter,L,rank),'shelf',s,'side',letter,'grp',2*s+sd-2,'gname',sprintf('%d%s',s,letter), ...
                    'level',L,'pos',rank,'role','store','xyz',xyz,'staging',stg,'dock',[faceP+nn(1)*dockDist G.slotY(k)], ...
                    'lift',max(0,xyz(3)-liftBase),'faceX',faceP,'n',nn,'faceD',nn*[faceP;0],'yaw',yaw,'boxYaw',localWrap(yaw-pi));
            end
        end
    end
end
nStand = size(WH.stands,1);
for i = 1:nStand
    sx = WH.stands(i,1); sy = WH.stands(i,2); nn = [0 1];
    cy = sy + 0.075; faceP = cy + G.targetDX;            % carton sits 0.075 m toward the aisle on its stand; same stand-off geometry as a shelf level 1
    xyz = [sx cy WH.standHeight+dims(3)/2];
    n = n+1;
    slots(n) = struct('num',n,'id',sprintf('D%02d',i),'shelf',0,'side','D','grp',100,'gname','D','level',1,'pos',i,'role','dock','xyz',xyz, ...
        'staging',[sx faceP+stage],'dock',[sx faceP+dockDist],'lift',max(0,xyz(3)-liftBase),'faceX',faceP,'n',nn,'faceD',faceP,'yaw',-pi/2,'boxYaw',localWrap(-pi/2-pi));
end
end

function a = localWrap(a)
a = mod(a+pi,2*pi)-pi;
end
