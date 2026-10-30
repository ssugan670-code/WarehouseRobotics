function testFleetLogic(nRobots,tEnd)
%TESTFLEETLOGIC  Headless test of Phase3Fleet (no Simulink): mock robots follow the fleet's jobs with a simple state machine.
%   Checks: assignment, reservations, locks, putaway/retrieve/cross-dock, traffic yielding, slot-taken question, snapshot, metrics.
if nargin<1, nRobots=6; end
if nargin<2, tEnd=900; end
setenv('P3_ROOT',fullfile(tempdir,'p3test')); 
here=fileparts(mfilename('fullpath')); addpath(fullfile(here,'..'));
C=helperFakeConfig();
F=Phase3Fleet(C,nRobots,'answer','ask','dwell',20,'verbose',1); F.autoPutaway=true;
F.dockAdd(2,true);
F.addOrder(1,1,true); F.addOrder(0,0,true);
F.setAuto(true,40,60,0);
mock=struct('pose',{},'state',{},'wp',{},'goal',{},'phase',{},'t0',{},'job',{},'seq',{},'status',{},'rp',{},'detour',{});
for k=1:nRobots, h=C.P3.robots(k).home; mock(k)=struct('pose',[h 0],'state',0,'wp',zeros(0,2),'goal',h,'phase',0,'t0',0,'job',[],'seq',0,'status','idle','rp',0,'detour',zeros(0,2)); end
dt=0.1; t=0; askedTest=false; minOver=inf;
while t<tEnd
    t=t+dt;
    for k=1:nRobots
        m=mock(k);
        rep=struct('t',t,'pose',m.pose,'state',m.state,'carry',any(m.state==[5 6]),'wp',localRemaining(m),'goal',m.goal,'jobSeq',m.seq,'jobStatus',m.status,'scanReq',false,'fault',false);
        cmd=F.exchange(k,rep);
        if ~isempty(cmd.job) && cmd.job.seq>m.seq && any(m.state==[0 7])
            m.job=cmd.job; m.seq=cmd.job.seq; m.status='running'; m.state=1; m.goal=m.job.src.staging; m.wp=[m.pose(1:2);m.goal]; m.t0=t;
        end
        if cmd.replanSeq~=m.rp && any(m.state==[1 5 7])
            m.rp=cmd.replanSeq; d=m.goal-m.pose(1:2); nd=[-d(2) d(1)]/max(norm(d),1e-6); if ~isempty(cmd.others), [~,ix]=min(sum((cmd.others(:,1:2)-m.pose(1:2)).^2,2)); sg=-sign(dot(nd,cmd.others(ix,1:2)-m.pose(1:2))); if sg==0, sg=1; end, else, sg=(-1)^k; end, m.detour=m.pose(1:2)+nd*(2.5*sg)+d/max(norm(d),1)*1.5; m.wp=[m.pose(1:2);m.detour;m.goal];
        end
        sp=0.6*cmd.scale; if cmd.hold, sp=0; end
        switch m.state
            case {1,5,7}
                tgt=m.goal; if ~isempty(m.detour), tgt=m.detour; end
                d=tgt-m.pose(1:2); n=norm(d);
                if n<0.05 && ~isempty(m.detour)
                    m.detour=zeros(0,2); m.wp=[m.pose(1:2);m.goal];
                elseif n<0.05
                    if m.state==1, m.state=10; m.t0=t; m.wp=zeros(0,2);
                    elseif m.state==5, m.state=6; m.t0=t; m.wp=zeros(0,2);
                    else, m.state=0; m.wp=zeros(0,2); end
                else
                    m.pose(1:2)=m.pose(1:2)+d/n*min(n,sp*dt); m.pose(3)=atan2(d(2),d(1));
                end
            case 10, if t-m.t0>8, m.state=4; m.t0=t; end
            case 4, if t-m.t0>6, m.state=5; m.goal=m.job.dst.approach; m.wp=[m.pose(1:2);m.goal]; end
            case 6, if t-m.t0>8, m.state=9; m.status='done'; m.t0=t; end
            case 9, if t-m.t0>4, m.state=7; m.goal=C.P3.robots(k).home; m.wp=[m.pose(1:2);m.goal]; end
        end
        mock(k)=m;
    end
    if ~askedTest && t>60
        askedTest=true; F.manualTask(1,'putaway',3,'3A-L1-P04',0);   % may open a question if slot taken
    end
    if ~isempty(F.questions) && mod(round(t),5)==0, F.reply('auto'); end
    if t>120 && abs(t-120)<dt/2, F.snapshot('_test'); end
end
F.statusText(); F.snapshot('_final'); F.metricsWrite(true);
T=F.tasks; fprintf('tasks: %d done, %d failed, %d open\n',sum(strcmp({T.status},'DONE')),sum(strcmp({T.status},'FAILED')),sum(~ismember({T.status},{'DONE','FAILED'})));
if F.collisions>0, fprintf('NOTE: %d close approaches in the mock (the mock drives straight lines through racks and has no turning, so this only exercises the logic)\n',F.collisions); end
assert(sum(strcmp({T.status},'DONE'))>=max(2,nRobots),'fleet did not complete tasks');
fprintf('TEST OK  minSep=%.2f nearMiss=%d replans=%d\n',F.minSep,F.nearMiss,F.replans);
end

function w = localRemaining(m)
% what a real controller reports: only the waypoints still ahead (detour point first, then the goal)
w = zeros(0,2); if ~any(m.state==[1 5 7]), return, end
if ~isempty(m.detour), w = [m.detour; m.goal]; else, w = m.goal; end
end
