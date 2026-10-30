function endMission()
%ENDMISSION  No more cartons: the robot finishes what is queued, drives home, parks and the 3-D window closes.
seq=0;
try, c=evalin('base','Phase2MissionCmd'); seq=c.seq; catch, end
assignin('base','Phase2MissionCmd',struct('seq',seq+1,'box',0,'finish',true,'station',0));
fprintf('[ENDMISSION] end sent to the robot\n');
end
