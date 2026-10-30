function pickBox(k,station)
%PICKBOX  Send the next carton to the running warehouse simulation.
%   pickBox(3)        carton 3, delivered to Packing Station 1
%   pickBox(3,2)      carton 3, delivered to Packing Station 2
%   pickBox('3rd box')  text is fine too
% Works while the simulation is running, also while the robot is still driving home: the route changes at once.
if nargin<2, station=0; end
if ischar(k) || isstring(k)
    d=regexp(char(k),'\d+','match','once'); k=str2double(d);
end
assert(isscalar(k) && isfinite(k) && k>=1 && k==round(k),'pickBox: give a carton number 1..3');
seq=0;
try, c=evalin('base','Phase2MissionCmd'); seq=c.seq; catch, end
assignin('base','Phase2MissionCmd',struct('seq',seq+1,'box',k,'finish',false,'station',station));
if station>0, fprintf('[PICKBOX] carton %d -> station %d sent to the robot\n',k,station); else, fprintf('[PICKBOX] carton %d sent to the robot\n',k); end
end
