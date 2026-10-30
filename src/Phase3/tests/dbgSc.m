function dbgSc(sc,tEnd,tag)
[script,auto] = scenarioScript(sc);
R = WorldSim('tEnd',tEnd,'seed',sc,'pull',true,'auto',auto,'script',script,'tag',tag);
T = R.trace; save('-text',['/tmp/w/trace_' tag '.txt'],'T');
planLog = R.planLog; save('-text',['/tmp/w/plan_' tag '.txt'],'planLog');
end
