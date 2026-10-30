function efficiencyStudy(nList,tEnd,outFile)
%EFFICIENCYSTUDY  How many robots does this warehouse need?  Same workload (auto mode: put-away of every arriving carton + a retrieve order about every 20 s),
%   different fleet sizes, headless fast model (WorldSim: the real Phase3Fleet, mock robots with the arm times of the real logs).
%   efficiencyStudy([1 2 3 4 6],400,'efficiency.csv')      columns: robots, tasks done, throughput [tasks/h], mean queue wait [s], mean service time [s],
%                                                              mean utilisation [%], time held back by other robots [% of working time], distance per task [m], collisions
if nargin < 1, nList = [1 2 3 4 6]; end
if nargin < 2, tEnd = 400; end
if nargin < 3, outFile = 'efficiency.csv'; end
fid = fopen(outFile,'a'); fprintf(fid,'robots,tasks_done,throughput_per_h,mean_queue_wait_s,mean_service_s,mean_utilisation_pct,held_back_pct,distance_per_task_m,collisions\n'); fclose(fid);
for n = nList
    R = WorldSim('tEnd',tEnd,'seed',11,'pull',true,'auto',true,'script',{},'tag',sprintf('eff%d',n),'verbose',0,'nRobots',n);
    F = R.fleet; T = F.tasks; d = T(strcmp({T.status},'DONE'));
    ut = zeros(1,n); wt = zeros(1,n); ds = 0;
    for k = 1:n, s = F.robots(k).stats; ut(k) = 100*s.busy/max(1,s.busy+s.idle); wt(k) = 100*s.wait/max(1,s.busy); ds = ds + s.dist; end
    qw = NaN; sv = NaN; if ~isempty(d), qw = mean([d.assigned]-[d.created]); sv = mean([d.done]-[d.assigned]); end
    fid = fopen(outFile,'a');
    fprintf(fid,'%d,%d,%.1f,%.1f,%.1f,%.0f,%.1f,%.1f,%d\n',n,numel(d),3600*numel(d)/tEnd,qw,sv,mean(ut),mean(wt),ds/max(1,numel(d)),R.collisions);
    fclose(fid);
end
end
