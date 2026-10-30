function testConverge(nRobots,seed)
%TESTCONVERGE  6 robots drive through the same spot from all sides (opposite goals).  Checks: no overlap (centre distance >= body), nobody stuck, few re-plans.
if nargin<1, nRobots=6; end
randomCase = nargin>=2; setenv('P3_ROOT',fullfile(tempdir,sprintf('p3conv%d',nargin))); here=fileparts(mfilename('fullpath')); addpath(fullfile(here,'..'));
C=helperFakeConfig(); F=Phase3Fleet(C,nRobots,'answer','auto','verbose',0); F.map2D=false;
c0=[0 40]; R=7; ang=(0:nRobots-1)*2*pi/nRobots; S=c0+R*[cos(ang)' sin(ang)']; G=c0-R*[cos(ang)' sin(ang)'];
if randomCase, rng_(seed); S=localSpread(nRobots,c0); G=localSpread(nRobots,c0); end
pos=S; dt=0.1; t=0; done=false(1,nRobots); minD=inf; rp=zeros(1,nRobots); det=cell(1,nRobots);
while t<300 && ~all(done)
    t=t+dt;
    for k=1:nRobots
        if done(k), tgt=G(k,:); else, tgt=G(k,:); end
        if ~isempty(det{k}), tgtm=det{k}; else, tgtm=tgt; end
        wp=tgtm; if ~isempty(det{k}), wp=[det{k}; tgt]; end
        if done(k), wp=zeros(0,2); end
        st=1; if done(k), st=0; end
        rep=struct('t',t,'pose',[pos(k,:) 0],'state',st,'carry',false,'wp',wp,'goal',tgt,'jobSeq',0,'jobStatus','idle','scanReq',false,'fault',false);
        cmd=F.exchange(k,rep);
        if cmd.replanSeq~=rp(k) && ~done(k)
            rp(k)=cmd.replanSeq; d=tgt-pos(k,:); nd=[-d(2) d(1)]/max(norm(d),1e-6); best=-inf; obst=[cmd.others(:,1:2); cmd.routes(:,1:2)];
            for off=[-5 -3.5 -2.5 2.5 3.5 5]
                dv=pos(k,:)+nd*off+d/norm(d)*0.5; sm=[]; for u=0:0.1:1, sm(end+1,:)=pos(k,:)+(dv-pos(k,:))*u; end, for u=0.1:0.1:1, sm(end+1,:)=dv+(tgt-dv)*u; end %#ok<AGROW>
                sc_=inf; for q=1:size(sm,1), if ~isempty(obst), sc_=min(sc_,min(sqrt(sum((obst-sm(q,:)).^2,2)))); end, end
                sc_=sc_-0.15*abs(off); if sc_>best, best=sc_; det{k}=dv; end
            end
        end
        if done(k), continue, end
        sp=0.7*cmd.scale; if cmd.hold, sp=0; end
        a=tgtm-pos(k,:); n=norm(a);
        if n<0.1
            if ~isempty(det{k}), det{k}=[]; else, done(k)=true; end
        else, pos(k,:)=pos(k,:)+a/n*min(n,sp*dt); end
    end
    if mod(round(t*10),10)==0 && t>6 && t<22, fprintf('t=%.0f ',t); for k=1:nRobots, fprintf('| %s (%.1f,%.1f) sc%.2f ',F.robots(k).name(1:2),pos(k,1),pos(k,2),F.robots(k).scale); end, fprintf('\n'); end
    if false, fprintf('--- t=%.0f\n',t); for k=1:nRobots, fprintf('%s (%.2f,%.2f) sc%.2f blk%d hold%.1f done%d det%d goal(%.1f,%.1f)\n',F.robots(k).name,pos(k,1),pos(k,2),F.robots(k).scale,F.robots(k).blockedBy,F.robots(k).holdT,done(k),~isempty(det{k}),G(k,1),G(k,2)); end, disp(F.prec); disp(F.waitFor); end
    D=squareform(pdist_(pos)); D(D==0)=inf; [m,ix]=min(D(:)); if m<minD, minD=m; [ia,ib]=ind2sub(size(D),ix); tm=t; end
    if m<0.5 && ~exist('rep1','var'), rep1=1; fprintf('CLOSE t=%.1f %s-%s d=%.2f sc %.2f %.2f states: ',t,F.robots(ia).name,F.robots(ib).name,m,F.robots(ia).scale,F.robots(ib).scale); fprintf('pos (%.2f,%.2f) (%.2f,%.2f)\n',pos(ia,:),pos(ib,:)); end
end
fprintf('converge: %d/%d arrived in %.0f s, min centre distance %.2f m (body %.2f), replans %d, collisions %d\n',sum(done),nRobots,t,minD,F.TP.body,F.replans,F.collisions);
assert(all(done),'robots got stuck'); assert(minD>=F.TP.body,'robots overlapped');
fprintf('CONVERGE OK\n');
end
function d=pdist_(X)
n=size(X,1); d=[]; for i=1:n-1, for j=i+1:n, d(end+1)=norm(X(i,:)-X(j,:)); end, end %#ok<AGROW>
end
function M=squareform(d)
n=round((1+sqrt(1+8*numel(d)))/2); M=zeros(n); q=0; for i=1:n-1, for j=i+1:n, q=q+1; M(i,j)=d(q); M(j,i)=d(q); end, end
end

function P=localSpread(n,c0)
% n random points in the open area around c0 (10 m x 8 m), at least 1.6 m apart
P=zeros(0,2); while size(P,1)<n, p=c0+[(rand-0.5)*20 (rand-0.5)*16]; if isempty(P) || all(sqrt(sum((P-p).^2,2))>1.6), P(end+1,:)=p; end, end %#ok<AGROW>
end
function rng_(seed), rand('seed',seed); rand('state',seed); end
