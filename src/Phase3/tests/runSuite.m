function runSuite(first,last,tEnd,outFile,planner)
if nargin < 5, planner = 'mstar'; end
% random scenario suite: every scenario = different mix of commands / timing / mode (manual commands, queued, auto)
if nargin < 3, tEnd = 150; end
if nargin < 4, outFile = ''; end
fid = 1; if ~isempty(outFile), fid = fopen(outFile,'a'); end
for sc = first:last
    [script,auto] = scenarioScript(sc); kind = mod(sc,12);
    try
        R = WorldSim('tEnd',tEnd,'seed',sc,'pull',true,'auto',auto,'script',script,'tag',sprintf('suite%d',first),'verbose',0,'planner',planner);
        fprintf(fid,'sc=%d kind=%d col=%d colCarry=%d minSep=%.2f minSepC=%.2f done=%d fail=%d stops>2=%d stops>5=%d maxStop=%.1f replans=%d cycles=%d thr=%.2f\n', ...
            sc,kind,R.collisions,R.collisionsCarry,R.minSep,R.minSepCarry,R.tasksDone,R.tasksFailed,R.stopsOver2s,R.stopsOver5s,R.maxStop,R.replans,R.cycles,R.throughput);
        fprintf(fid,'    M*: runs=%d coupled=%d pushed=%d fail=%d maxGroup=%d maxT=%.1fs %s\n',R.mstat.runs,R.mstat.coupled,R.mstat.pushed,R.mstat.fail,R.mstat.maxGroup,R.mstat.maxTime,R.mstat.reason);
        for i = 1:size(R.stops,1), fprintf(fid,'    stop robot %d t_end=%.0f dur=%.1f at (%.1f,%.1f) state=%d carry=%d\n',R.stops(i,:)); end
        for i = 1:min(3,numel(R.events)), fprintf(fid,'    %s\n',R.events{i}); end
    catch ME
        fprintf(fid,'sc=%d ERROR %s (%s line %d)\n',sc,ME.message,ME.stack(1).name,ME.stack(1).line);
    end
end
if fid > 2, fclose(fid); end
end
