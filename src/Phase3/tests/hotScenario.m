function script = hotScenario(kind)
%HOTSCENARIO  Adversarial manual-mode job sets for WorldSim: many robots, one shelf / one station (like a real operator would give).
%   'same6'   12 pinned retrieves, 2 per robot, ALL from shelf 4 (both faces) to station 3  (the operator's test)
%   'adj4'    4 robots take boxes from 4 ADJACENT slots of one face (4A P03..P06) -> station 2; 2 robots other shelves -> station 1
%   'mixed'   6 dock boxes put away into one shelf face (adjacent slots) + 6 retrieves from the same shelf to station 3
script = {}; q = 0;
function add(t,s), script{end+1} = {t,s}; end
switch kind
    case 'same6'
        slots = {'4A-L1-P01','4A-L2-P04','4A-L3-P07','4B-L1-P03','4B-L2-P06','4B-L3-P08','4A-L1-P06','4A-L2-P02','4A-L3-P05','4B-L1-P08','4B-L2-P02','4B-L3-P04'};
        for i = 1:numel(slots), add(0.5 + 0.1*i,['add ' slots{i}]); end
        for i = 1:numel(slots), add(36 + 0.1*i,sprintf('retrieve %d 0 %s 3',1 + mod(i-1,6),slots{i})); end
    case 'adj4'
        slots = {'4A-L1-P03','4A-L1-P04','4A-L1-P05','4A-L1-P06','7A-L1-P04','3B-L2-P06'};
        for i = 1:4, add(0.5 + 0.1*i,['add ' slots{i}]); end
        for i = 1:4, add(36 + 0.1*i,sprintf('retrieve %d 0 %s 2',i,slots{i})); end
        for i = 5:6, add(36 + 0.1*i,sprintf('retrieve %d 0 %s 1',i,slots{i})); end
    case 'his6'       % the operator's real run (Oct 21): six retrieves from shelf 4 side A (one face), all to station 3, his task order
        slots = {'4A-L1-P01','4A-L1-P06','4A-L3-P07','4A-L3-P05','4A-L2-P03','4A-L1-P02'};
        for i = 1:numel(slots), add(0.5 + 0.1*i,['add ' slots{i}]); end
        for i = 1:numel(slots), add(20 + 0.1*i,sprintf('retrieve %d 0 %s 3',i,slots{i})); end
    case 'cross6'     % 3 robots carry from the far WEST shelf to the far EAST station while 3 carry from the far EAST shelf to the far WEST station: head-on traffic 3 against 3
        slots = {'1A-L1-P05','1A-L1-P09','1A-L2-P13','8B-L1-P05','8B-L1-P09','8B-L2-P13'}; stn = [1 1 1 3 3 3];
        for i = 1:6, add(0.5 + 0.1*i,['add ' slots{i}]); end
        for i = 1:6, add(20 + 0.1*i,sprintf('retrieve %d 0 %s %d',i,slots{i},stn(i))); end
    case 'cross5'     % 1 robot against 5
        slots = {'1A-L1-P05','8B-L1-P05','8B-L1-P09','8B-L2-P13','8B-L1-P13','8B-L3-P03'}; stn = [1 3 3 3 3 3];
        for i = 1:6, add(0.5 + 0.1*i,['add ' slots{i}]); end
        for i = 1:6, add(20 + 0.1*i,sprintf('retrieve %d 0 %s %d',i,slots{i},stn(i))); end
    case 'real6'      % the operator's run of Oct 23 exactly: named robots, his slots, all to station 3 (blue red green black pink yellow = 1..6)
        slots = {'4A-L1-P06','4A-L1-P01','4A-L3-P07','4A-L3-P05','4A-L2-P03','4A-L1-P02'};
        for i = 1:numel(slots), add(0.5 + 0.1*i,['add ' slots{i}]); end
        for i = 1:numel(slots), add(18 + 0.1*i,sprintf('retrieve %d %d %s 3',i,i,slots{i})); end
    case 'user6'      % the operator's own test: six pinned retrieves from shelf 4 (both faces), all to station 3
        slots = {'4A-L1-P01','4A-L1-P06','4A-L3-P07','4B-L1-P08','4B-L3-P04','4B-L3-P06'};
        for i = 1:numel(slots), add(0.5 + 0.1*i,['add ' slots{i}]); end
        for i = 1:numel(slots), add(20 + 0.1*i,sprintf('retrieve %d 0 %s 3',i,slots{i})); end
    case 'face6'      % six robots, six boxes on ONE face of a 13 m shelf (4B), three stations: only 3 may work at the same time, the others wait at a safe distance and follow
        slots = {'4B-L1-P01','4B-L1-P03','4B-L2-P05','4B-L1-P07','4B-L3-P02','4B-L2-P08'};
        for i = 1:numel(slots), add(0.5 + 0.1*i,['add ' slots{i}]); end
        for i = 1:numel(slots), add(20 + 0.1*i,sprintf('retrieve %d 0 %s %d',i,slots{i},1 + mod(i-1,3))); end
    case 'face6adj'   % six boxes in adjacent slots of one face (worst case for spacing)
        slots = {'4B-L1-P02','4B-L1-P03','4B-L1-P04','4B-L1-P05','4B-L1-P06','4B-L1-P07'};
        for i = 1:numel(slots), add(0.5 + 0.1*i,['add ' slots{i}]); end
        for i = 1:numel(slots), add(20 + 0.1*i,sprintf('retrieve %d 0 %s %d',i,slots{i},1 + mod(i-1,3))); end
    case 'long6'      % 30 m shelf (1A): six robots at once are allowed
        slots = {'1A-L1-P01','1A-L1-P04','1A-L1-P07','1A-L2-P10','1A-L1-P13','1A-L1-P16'};
        for i = 1:numel(slots), add(0.5 + 0.1*i,['add ' slots{i}]); end
        for i = 1:numel(slots), add(20 + 0.1*i,sprintf('retrieve %d 0 %s %d',i,slots{i},1 + mod(i-1,3))); end
    case 'mid6'       % 19 m shelf (2B): four at once
        slots = {'2B-L1-P01','2B-L1-P03','2B-L2-P05','2B-L1-P07','2B-L3-P09','2B-L1-P11'};
        for i = 1:numel(slots), add(0.5 + 0.1*i,['add ' slots{i}]); end
        for i = 1:numel(slots), add(20 + 0.1*i,sprintf('retrieve %d 0 %s %d',i,slots{i},1 + mod(i-1,3))); end
    case 'east6'      % shelf 8 east face (the tight side before the layout was widened)
        slots = {'8B-L1-P02','8B-L1-P05','8B-L1-P08','8B-L2-P11','8B-L1-P14','8B-L1-P17'};
        for i = 1:numel(slots), add(0.5 + 0.1*i,['add ' slots{i}]); end
        for i = 1:numel(slots), add(20 + 0.1*i,sprintf('retrieve %d 0 %s %d',i,slots{i},1 + mod(i-1,3))); end
    case 'mixed'
        add(0.5,'dock 6');
        put = {'4A-L1-P02','4A-L1-P03','4A-L1-P04','4A-L1-P05','4A-L1-P06','4A-L1-P07'};
        ret = {'4B-L1-P02','4B-L1-P04','4B-L2-P03','4B-L2-P05','4A-L2-P04','4A-L3-P06'};
        for i = 1:6, add(1 + 0.1*i,['add ' ret{i}]); end
        for i = 1:6, add(36 + 0.1*i,sprintf('retrieve %d 0 %s 3',i,ret{i})); end
        for i = 1:6, add(36.5 + 0.1*i,sprintf('putaway %d %d %s',i,40+i,put{i})); end
end
end
