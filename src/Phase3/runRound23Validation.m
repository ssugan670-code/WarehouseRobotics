function out=runRound23Validation(stopTime)
%RUNROUND23VALIDATION Rebuild, run, and print the evidence for the 3-carton mission (Round 30).
% Run this from src\Phase2 in the local MATLAB installation.  Do any desired
% clear/rehash cleanup *before* calling this function: clearing functions
% here would discard this function's input arguments mid-run.
% The base-workspace variable Phase2MissionQueue ([2 3 1] or 'auto') selects the pick order.
if nargin<1, stopTime=3000; end
C=WarehouseRobotPhase2Config();
buildWarehouseRobotSim;
robot='WarehouseRobotSim/Robot 1 Kinova';
ports=get_param(robot,'PortHandles'); params=get_param(robot,'DialogParameters');
fprintf('[GRASP-PORTS] in=%d out=%d params=%s\n',numel(ports.Inport),numel(ports.Outport),strjoin(fieldnames(params).',','));
set_param('WarehouseRobotSim','StopTime',num2str(stopTime)); out=sim('WarehouseRobotSim');
names={'Phase2TargetBoxTranslationLog','Phase2TargetBox2TranslationLog','Phase2TargetBox3TranslationLog'};
sfc=C.TableSurfaceZ;
for b=1:numel(names)
    try
        p=localLastRow(out.get(names{b}));          % Unreal frame (y negated)
        box=[p(1) -p(2) p(3)]; slot=C.Boxes(b).slotXYZ;
        fprintf('[FINAL] box %d (shelf %d level %d) world=[%.3f %.3f %.3f] | table slot=[%.3f %.3f] | bottomAboveTable=%.4f | slotError=%.3f m\n', ...
            b,C.Boxes(b).shelf,C.Boxes(b).level,box,slot(1:2),box(3)-0.5*C.TargetDimensions(3)-sfc,norm(box(1:2)-slot(1:2)));
    catch ME
        fprintf('[FINAL] box %d transform log unavailable: %s\n',b,ME.message);
    end
end
try
    d=out.get('Phase2TaskStateLog'); st=double(reshape(d.signals.values,[],1)); t=d.time(:);
    ch=[true; diff(st)~=0]; fprintf('[STATE-TIMELINE] %s\n',mat2str([t(ch).'; st(ch).'],4));
catch
end
end

function p=localLastRow(d)
v=d.signals.values;
if ndims(v)==3, v=squeeze(v).'; end            % 1x3xN -> Nx3
if size(v,2)~=3 && size(v,1)==3, v=v.'; end
p=v(end,:);
end
