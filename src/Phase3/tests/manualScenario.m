function script = manualScenario(seed,nPer)
%MANUALSCENARIO  Like a real manual-mode run: at t = 36 s every robot gets 2..nPer jobs at once (retrieve to a station / put away from a stand), jobs picked at random.
if nargin < 2, nPer = 3; end
C = helperFakeConfig(30,10,54,seed); S = C.P3.slots; rs = rand('twister'); rand('twister',900 + seed); %#ok<NASGU>
cs = C.P3.cartons; stored = find(strcmp({cs.state},'STORED')); dock = find(strcmp({cs.state},'DOCK'));
stored = stored(randperm(numel(stored))); dock = dock(randperm(numel(dock)));
isStore = find(strcmp({S.role},'store')); occ = [cs(stored).slot]; freeS = setdiff(isStore,occ); freeS = freeS(randperm(numel(freeS)));
script = {}; si = 1; di = 1; fi = 1;
for k = 1:6
    n = 2 + floor(rand*(nPer-1));
    for q = 1:n
        if rand < 0.5 && si <= numel(stored)
            script{end+1} = {36 + 0.1*numel(script),sprintf('retrieve %d %d %s %d',k,stored(si),S(cs(stored(si)).slot).id,1 + floor(rand*3))}; si = si + 1; %#ok<AGROW>
        elseif di <= numel(dock)
            script{end+1} = {36 + 0.1*numel(script),sprintf('putaway %d %d %s',k,dock(di),S(freeS(fi)).id)}; di = di + 1; fi = fi + 1; %#ok<AGROW>
        end
    end
end
end
