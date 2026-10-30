function chart=buildRobotTaskFSM(modelName,C,position)
%BUILDROBOTTASKFSM Create a discrete Stateflow task supervisor.
if nargin<3, position=[270 90 610 390]; end
path=[modelName '/RobotTaskFSM'];
if ~isempty(find_system(modelName,'SearchDepth',1,'Name','RobotTaskFSM')), delete_block(path); end
add_block('sflib/Chart',path,'Position',position);
root=sfroot; chart=root.find('-isa','Stateflow.Chart','Path',path);
chart.ChartUpdate='DISCRETE'; chart.SampleTime=num2str(C.SampleTime); chart.ActionLanguage='MATLAB';
for name={'AtGoal','ArmDone','PerceptionValid','BoxX','BoxY','BoxZ','IsTargetGrasped','ArmFault','MoreBoxes','ScanDone','Finish','NeedHome'}
    type='double'; if any(strcmp(name,{'AtGoal','ArmDone','PerceptionValid','IsTargetGrasped','ArmFault','MoreBoxes','ScanDone','Finish','NeedHome'})), type='boolean'; end
    localData(chart,name{1},'Input',type);
end
for name={'TaskState','GoalX','GoalY','IKTargetX','IKTargetY','IKTargetZ','GraspCmd','PlaceCmd'}
    type='double'; if strcmp(name{1},'TaskState'), type='uint8'; end
    if any(strcmp(name,{'GraspCmd','PlaceCmd'})), type='boolean'; end
    localData(chart,name{1},'Output',type);
end
labels={ ...
sprintf('IDLE\nen: TaskState=uint8(0); GoalX=%.4f; GoalY=%.4f; GraspCmd=false; PlaceCmd=false;',C.HomeXY), ...
sprintf('NAV_TO_STAGING\nen: TaskState=uint8(1); GoalX=%.4f; GoalY=%.4f; GraspCmd=false; PlaceCmd=false;',C.ShelfStagingXY), ...
sprintf('DOCK_SHELF\nen: TaskState=uint8(10); GoalX=%.4f; GoalY=%.4f; IKTargetX=BoxX; IKTargetY=BoxY; IKTargetZ=BoxZ; GraspCmd=false; PlaceCmd=false;',C.ShelfDockXY), ...
sprintf('ARM_EXTEND\nen: TaskState=uint8(2); IKTargetX=BoxX; IKTargetY=BoxY; IKTargetZ=BoxZ; GraspCmd=false;'), ...
sprintf('GRASP\nen: TaskState=uint8(3); GraspCmd=true; PlaceCmd=false;'), ...
sprintf('ARM_RETRACT\nen: TaskState=uint8(4); GraspCmd=true; PlaceCmd=false;'), ...
sprintf('NAV_TO_TABLE\nen: TaskState=uint8(5); GoalX=%.4f; GoalY=%.4f; GraspCmd=true;',C.TableApproachXY), ...
sprintf('ARM_PLACE\nen: TaskState=uint8(6); IKTargetX=%.4f; IKTargetY=%.4f; IKTargetZ=%.4f; GraspCmd=true;',C.TablePlaceXYZ), ...
sprintf('RELEASE\nen: TaskState=uint8(8); GraspCmd=false; PlaceCmd=true;'), ...
sprintf('ARM_STOW\nen: TaskState=uint8(9); GraspCmd=false; PlaceCmd=false;'), ...
sprintf('RETURN_HOME\nen: TaskState=uint8(7); GoalX=%.4f; GoalY=%.4f; GraspCmd=false;',C.HomeXY), ...
sprintf('FAULT_HOLD\nen: TaskState=uint8(11); GraspCmd=false; PlaceCmd=false;'), ...
sprintf('MISSION_DONE\nen: TaskState=uint8(12); GoalX=%.4f; GoalY=%.4f; GraspCmd=false; PlaceCmd=false;',C.HomeXY)};
states=cell(1,numel(labels));
for k=1:numel(labels)
    states{k}=Stateflow.State(chart); states{k}.LabelString=labels{k};
    states{k}.Position=[20+150*mod(k-1,5),30+145*floor((k-1)/5),125,82];
end
localDefault(chart,states{1});
% Round 30 multi-carton mission.  The controller picks the next carton while in IDLE (MoreBoxes) and, after a 1 s
% scan dwell at the staging pose (ScanDone), reports whether the carton is really in its slot (PerceptionValid).
% A missing carton sends the task back to IDLE, where it is marked absent and the next one is chosen: no retry loop.
% Round 31: boxes arrive live (pickBox / endMission).  With nothing to do the robot waits in IDLE; if it is away from home it
% drives home first.  MISSION_DONE only after endMission AND parked at home.
localTransition(chart,states{1},states{2},'after(1,sec)[MoreBoxes]');
localTransition(chart,states{1},states{11},'after(1,sec)[~MoreBoxes && NeedHome]');
localTransition(chart,states{1},states{13},'after(2,sec)[~MoreBoxes && Finish && ~NeedHome]');
localTransition(chart,states{2},states{3},'[AtGoal && ScanDone && PerceptionValid]');
localTransition(chart,states{2},states{1},'[AtGoal && ScanDone && ~PerceptionValid]');
% DOCK_SHELF does not hand the arm to the rack until both the precision base
% pose and the one-time, velocity-limited READY unfold have completed.
localTransition(chart,states{3},states{4},'[AtGoal && ArmDone]');
localTransition(chart,states{3},states{12},'[ArmFault]');      % Round 15: a dock that never settles (or a watchdog stop) faults instead of waiting for ever
% A failed IK/grasp must not route through IDLE: IDLE starts a new task
% after two seconds, which previously created an endless reverse/U-turn/
% re-approach loop.  FAULT_HOLD commands neither base nor arm motion and
% remains latched unless the fleet manager recovers it (see the end of this file).
localTransition(chart,states{4},states{12},'[ArmFault]');
localTransition(chart,states{4},states{5},'[ArmDone]');
localTransition(chart,states{5},states{12},'[ArmFault]');
localTransition(chart,states{6},states{12},'[ArmFault]');
% State 3 owns approach -> seal -> confirm -> lift.  Its controller starts
% the 1.5 s confirmation timer at the native vacuum rising edge, not on
% state entry; ArmDone therefore means the verified lift has completed.
localTransition(chart,states{5},states{6},'[ArmDone && IsTargetGrasped]');
localTransition(chart,states{6},states{7},'[ArmDone]');
localTransition(chart,states{7},states{8},'[AtGoal]');
localTransition(chart,states{8},states{12},'[ArmFault]');
localTransition(chart,states{8},states{9},'[ArmDone]');
localTransition(chart,states{9},states{10},'[~IsTargetGrasped]');
localTransition(chart,states{9},states{10},'after(1,sec)');
localTransition(chart,states{10},states{11},'[ArmDone]');
% After the carton is placed the robot drives home and returns to IDLE; IDLE decides between the next carton and MISSION_DONE.
% A new box entered while driving home changes the route at once: IDLE selects it, then NAV_TO_STAGING starts.
localTransition(chart,states{11},states{1},'[AtGoal || MoreBoxes]');
% Fault recovery (Round 14, latch fixed in Round 15): the fleet manager clears the latch when it has a way out.  Carton in the gripper -> ARM_RETRACT (back off from the rack), then
% NAV_TO_TABLE with the new destination;  no carton in the gripper -> ARM_STOW, RETURN_HOME, IDLE.  Without a recovery the latch stays and the robot holds (the fleet always offers one, see Phase3Fleet.recoverFault).
localTransition(chart,states{12},states{6},'[~ArmFault && IsTargetGrasped]');
localTransition(chart,states{12},states{10},'[~ArmFault && ~IsTargetGrasped]');
end
function d=localData(chart,name,scope,type), d=Stateflow.Data(chart); d.Name=name; d.Scope=scope; d.DataType=type; end
function localDefault(chart,d), t=Stateflow.Transition(chart); t.Destination=d; t.DestinationOClock=9; t.SourceEndPoint=[10 10]; end
function localTransition(chart,s,d,label), t=Stateflow.Transition(chart); t.Source=s; t.Destination=d; t.LabelString=label; end
