function runRound30(mode,varargin)
%RUNROUND30  Warehouse pick-and-place mission with three ways to give the orders.
%   runRound30                 menu in the command window (1 auto / 2 one by one / 3 own order)
%   runRound30('auto')         i)   all cartons, nearest shelf first
%   runRound30('single')       ii)  one carton at a time; after each placement it asks the next carton or end
%   runRound30([3 1 2])        iii) your own full order
%   Delivery station per carton:  runRound30([3 2 1],'station',[2 1 1])   (same length as the order, or one number for all)
%   runRound30(...,'cmd')      start only; then type  pickBox(3)  /  pickBox(3,2)  /  endMission  yourself
%   runRound30(...,'vision','view'|'none'|'record')   robot camera window; if not given, the command window asks (view / no view / record)
%   runRound30(...,'yawsign',-1)   only if the camera picture turns the wrong way when the robot turns
%   runRound30(...,'missing',2)   test: carton 2 not in its slot (robot must report it absent)
% Cartons are named by shelf number (the number plate on both end panels of every shelf).
if nargin<1, mode=[]; end
nonblock=false; missing=[]; skip=0; stationArg=[]; vision=''; yawSign=1;
k=1;
while k<=numel(varargin)
    v=varargin{k};
    if strcmpi(v,'cmd'), nonblock=true; k=k+1; continue, end   % 'cmd' is a flag without a value
    if k==numel(varargin), break, end
    if strcmpi(v,'missing'), missing=varargin{k+1}; elseif strcmpi(v,'skipladder'), skip=varargin{k+1}; elseif strcmpi(v,'station'), stationArg=varargin{k+1}; elseif strcmpi(v,'vision'), vision=lower(char(varargin{k+1})); elseif strcmpi(v,'yawsign'), yawSign=varargin{k+1}; end
    k=k+2;
end
if ischar(mode) || isstring(mode)
    mode=lower(char(mode));
    if strcmp(mode,'cmd'), nonblock=true; mode='single'; end
end
C=WarehouseRobotPhase2Config();
nb=numel(C.Boxes); ns=numel(C.Stations);
fprintf('\nCartons:\n'); for k=1:nb, fprintf('  %d : shelf %d, level %d (%.2f m high)\n',k,C.Boxes(k).shelf,C.Boxes(k).level,C.Boxes(k).xyz(3)); end
fprintf('Delivery stations:\n'); for t=1:ns, fprintf('  %d : %s (%s side)\n',t,C.Stations(t).name,C.Stations(t).side); end
if isempty(mode)
    fprintf('\nHow should the robot get its orders?\n  1 : AUTO         - nearest carton first, all cartons\n  2 : ONE BY ONE   - you give one carton at a time (live, also while the robot drives home)\n  3 : OWN ORDER    - you give the full order now, e.g. 3 1 2\n');
    while true
        s=strtrim(input('Mode (1/2/3): ','s')); n=str2double(s);
        if any(n==[1 2 3]), break, end
        fprintf('  please type 1, 2 or 3\n');
    end
    if n==1, mode='auto'; elseif n==2, mode='single'; else, mode=localAskOrder(nb); end
end
queue=[]; byBox=ones(1,nb);
if ischar(mode)
    if strcmp(mode,'auto'), queue='auto'; byBox=localByBox(localAskStations(ns,C,nb,'all cartons',stationArg),1:nb,nb); end
elseif isnumeric(mode)
    queue=reshape(double(mode),1,[]);
    assert(~isempty(queue) && all(ismember(queue,1:nb)) && numel(unique(queue))==numel(queue),'Order must use carton numbers 1..%d without repeats.',nb);
    byBox=localByBox(localAskStations(ns,C,numel(queue),['order ' mat2str(queue)],stationArg),queue,nb);
end
singleMode=isempty(queue);
assignin('base','Phase2MissionQueue',queue); assignin('base','Phase2MissionStations',byBox);
assignin('base','Phase2TestMissingBox',missing); assignin('base','Phase2TestSkipLadder',skip);
assignin('base','Phase2MissionCmd',struct('seq',0,'box',0,'finish',false,'station',0));
try, evalin('base','clear Phase2MissionStatus'); catch, end
if isempty(vision)
    fprintf('\nRobot camera window?\n  1 : VIEW      - live window: what the robot camera sees + what it is computing\n  2 : NO VIEW   - no camera window\n  3 : RECORD    - live window and save it as an .mp4 in this folder\n');
    while true
        n=str2double(strtrim(input('Camera (1/2/3, Enter = 1): ','s')));
        if isnan(n), n=1; end
        if any(n==[1 2 3]), break, end
        fprintf('  please type 1, 2 or 3\n');
    end
    opts={'view','none','record'}; vision=opts{n};
end
if any(strcmp(vision,{'on','view'})), vision='view'; elseif any(strcmp(vision,{'off','none'})), vision='none'; end
buildWarehouseRobotSim;
mdl='WarehouseRobotSim';
% one forward camera on the lift carriage replaces the old (unused) head camera; if anything goes wrong, rebuild exactly as before
okCam=false; try, okCam=addRobotVisionView(mdl,'Mode',vision,'YawSign',yawSign); catch, end
if ~okCam   % second try: keep the old (unused) head camera in place, only add the new one
    buildWarehouseRobotSim;
    try, okCam=addRobotVisionView(mdl,'Mode',vision,'YawSign',yawSign,'Retire',false); catch, end
    if ~okCam, buildWarehouseRobotSim; end
end
if okCam && strcmp(vision,'record') || okCam && strcmp(vision,'view')
    set_param(mdl,'StopFcn','try, robotVisionViewer(''close''); catch, end');   % finalises the .mp4 / .csv when the simulation stops
end
set_param(mdl,'StopTime','inf','ReturnWorkspaceOutputs','off');
set_param(mdl,'SimulationCommand','start');
fprintf('\n[RUN] simulation started - waiting for the 3-D window...\n');
t0=tic;
while ~localHasStatus() && localRunning(mdl) && toc(t0)<300, pause(0.5); end
if nonblock
    fprintf('[RUN] running. Type  pickBox(3)  or  pickBox(3,2)  to send a carton,  endMission  to finish (robot goes home, window closes).\n');
    return
end
if ~singleMode
    fprintf('[RUN] robot works through %s and parks at home; the window closes by itself. (pickBox / endMission still work.)\n',localTxt(queue));
    localWait(mdl); localSummary(C); return
end
asked=-1;
while localRunning(mdl)
    st=localStatus();
    if isempty(st), pause(0.3); continue, end
    if st.state==11, localFault(mdl); break, end
    nDone=numel(st.done)+numel(st.absent);
    if nDone>=nb && ~st.finish
        fprintf('[RUN] all cartons handled -> end.\n'); endMission(); asked=nDone; continue
    end
    if ~st.finish && isempty(st.pending) && nDone~=asked
        asked=nDone;
        left=setdiff(1:nb,[st.done st.absent]);
        [a,stn]=localAsk(left,C);
        if strcmp(a,'end'), endMission(); else, pickBox(str2double(a),stn); end
    else
        pause(0.3);
    end
end
localSummary(C);
end

function order=localAskOrder(nb)
while true
    s=strtrim(input(sprintf('Full order, e.g. 3 1 2  (carton numbers 1..%d): ',nb),'s'));
    n=str2double(regexp(s,'\d+','match'));
    if ~isempty(n) && all(n>=1 & n<=nb) && all(n==round(n)) && numel(unique(n))==numel(n), order=n; return, end
    fprintf('  use distinct carton numbers between 1 and %d, e.g. 3 1 2\n',nb);
end
end
function st=localAskStations(ns,C,count,label,given)
if ns==1, st=ones(1,count); return, end
if ~isempty(given), st=reshape(given,1,[]); if isscalar(st), st=repmat(st,1,count); end
    assert(numel(st)==count && all(ismember(st,1:ns)),'station list must have %d entries between 1 and %d',count,ns); return, end
names=strjoin(arrayfun(@(t) sprintf('%d=%s',t,C.Stations(t).name),1:ns,'UniformOutput',false),', ');
while true
    s=strtrim(input(sprintf('Deliver %s to which station? %s\n  one number for all, or %d numbers in order (Enter = 1): ',label,names,count),'s'));
    if isempty(s), st=ones(1,count); return, end
    n=str2double(regexp(s,'\d+','match'));
    if isscalar(n) && any(n==1:ns), st=repmat(n,1,count); return, end
    if numel(n)==count && all(ismember(n,1:ns)), st=n; return, end
    fprintf('  please give 1 number or %d numbers, each 1..%d\n',count,ns);
end
end
function byBox=localByBox(st,order,nb)
byBox=ones(1,nb);
if ischar(order), return, end
if numel(st)==nb && numel(order)==nb && isequal(order,1:nb), byBox=st; return, end
for i=1:numel(order), byBox(order(i))=st(i); end
end
function [a,stn]=localAsk(left,C)
ns=numel(C.Stations); stn=1;
while true
    s=strtrim(lower(input(sprintf('Next carton? %s or end : ',mat2str(left)),'s')));
    if any(strcmp(s,{'end','e','stop','no'})), a='end'; return, end
    n=str2double(regexp(s,'\d+','match'));
    if isscalar(n) && ismember(n,left), a=num2str(n); break, end
    if numel(n)>1, fprintf('  one carton at a time here (use mode 3 for a full order)\n'); else, fprintf('  please type one of %s, or end\n',mat2str(left)); end
end
if ns>1
    names=strjoin(arrayfun(@(t) sprintf('%d=%s',t,C.Stations(t).name),1:ns,'UniformOutput',false),', ');
    while true
        s=strtrim(input(sprintf('Deliver carton %s to which station? %s (Enter = 1): ',a,names),'s'));
        if isempty(s), stn=1; return, end
        n=str2double(s);
        if isscalar(n) && ismember(n,1:ns), stn=n; return, end
        fprintf('  please type a number 1..%d\n',ns);
    end
end
end
function s=localTxt(q), if ischar(q), s='all cartons (nearest first)'; else, s=['order ' mat2str(q)]; end, end
function tf=localRunning(mdl), tf=~strcmp(get_param(mdl,'SimulationStatus'),'stopped'); end
function tf=localHasStatus(), tf=false; try, st=evalin('base','Phase2MissionStatus'); tf=st.time>0; catch, end, end
function st=localStatus(), st=[]; try, st=evalin('base','Phase2MissionStatus'); catch, end, end
function localFault(mdl)
fprintf('[RUN] robot is in FAULT_HOLD (see the [FAULT] / [IKCHK] lines above). Window stays open for 8 s, then the run stops.\n');
pause(8); set_param(mdl,'SimulationCommand','stop');
end
function localWait(mdl)
while localRunning(mdl)
    st=localStatus();
    if ~isempty(st) && st.state==11, localFault(mdl); break, end
    pause(0.5);
end
end
function localSummary(C)
fprintf('\n[RUN] simulation stopped.\n');
names={'Phase2TargetBoxTranslationLog','Phase2TargetBox2TranslationLog','Phase2TargetBox3TranslationLog'};
for b=1:numel(names)
    try
        d=evalin('base',names{b}); v=d.signals.values; if ndims(v)==3, v=squeeze(v).'; end; if size(v,2)~=3, v=v.'; end
        p=v(end,:); box=[p(1) -p(2) p(3)];
        best=inf; bn=''; for t=1:numel(C.Stations), for j=1:3, e=norm(box(1:2)-C.Stations(t).slotXYZ(j,1:2)); if e<best, best=e; bn=sprintf('%s slot %d',C.Stations(t).name,j); end, end, end
        fprintf('[FINAL] box %d world=[%.3f %.3f %.3f] | nearest %s (%.3f m) | bottomAboveTable=%.4f\n',b,box,bn,best,box(3)-0.5*C.TargetDimensions(3)-C.TableSurfaceZ);
    catch ME
        fprintf('[FINAL] box %d log unavailable: %s\n',b,ME.message);
    end
end
end
