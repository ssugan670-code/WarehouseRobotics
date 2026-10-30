function [script,auto] = scenarioScript(sc)
    rand('twister',1000 + sc); n = floor(rand*6);
    script = {}; kind = mod(sc,12);
    t0 = 1 + 20*rand*(kind == 3);
    switch kind
        case 0, script = {{t0,'qput'},{t0+1,'order 1 1'},{t0+1,'order 2 1'},{t0+1,'order 3 1'},{t0+1,'order 4 1'}};                 % everything at once, one station
        case 1, script = {{t0,'qput'},{t0+0.5,'order 1 1'},{t0+0.5,'order 2 2'},{t0+0.5,'order 3 3'},{t0+0.5,'order 4 2'}};          % spread over stations
        case 2, script = {{t0,'qput'}};                                                                                                 % putaway only
        case 3, script = {{t0,'order 1 2'},{t0+3,'order 2 2'},{t0+6,'order 3 2'},{t0+9,'order 4 2'},{t0+12,'qput'}};                  % staggered
        case 4, for q = 1:8, script{end+1} = {1 + 10*rand,'qput'}; end, script{end+1} = {5,'order 1 3'}; script{end+1} = {6,'order 3 3'};  %#ok<AGROW>
        case 5, script = {{t0,'qput'},{t0 + 2,'order 2 1'},{t0 + 2,'order 4 3'}};
        otherwise                                                                                                                       % random mix of orders / arrivals at random times (auto on for odd kinds)
            for q = 1:(6 + floor(rand*8))
                tq = 1 + 90*rand;
                if rand < 0.35, script{end+1} = {tq,'qput'}; else, script{end+1} = {tq,sprintf('order %d %d',1 + floor(rand*4),1 + floor(rand*3))}; end %#ok<AGROW>
            end
    end
auto = kind == 4 || kind == 5 || (kind >= 6 && mod(kind,2) == 1);
end
