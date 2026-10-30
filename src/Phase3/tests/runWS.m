function r = runWS(varargin)
R = WorldSim(varargin{:});
fprintf('col=%d colCarry=%d minSep=%.2f minSepC=%.2f done=%d fail=%d stops>2=%d stops>5=%d maxStop=%.1f plans=%d planTime=%.0fs wall=%.0fs replans=%d thr=%.2f/min\n', ...
 R.collisions,R.collisionsCarry,R.minSep,R.minSepCarry,R.tasksDone,R.tasksFailed,R.stopsOver2s,R.stopsOver5s,R.maxStop,R.plans,R.planTime,R.wall,R.replans,R.throughput);
for i=1:min(8,numel(R.events)), disp(R.events{i}); end
for i=1:size(R.stops,1), fprintf('  stop robot %d t_end=%.0f dur=%.1f at (%.1f,%.1f) state=%d carry=%d\n',R.stops(i,:)); end
r = rmfield(R,'fleet');
end
