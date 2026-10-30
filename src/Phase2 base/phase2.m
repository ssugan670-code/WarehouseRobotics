function phase2(varargin)
%PHASE2  Start the Phase 2 warehouse mission. Just type   phase2
%   It asks, one after the other: order mode (1 auto / 2 one by one / 3 own order), delivery station(s),
%   and the robot camera (1 view / 2 no view / 3 record). Everything else is as before.
%   Options (all optional, they skip the matching question):  phase2('auto'), phase2([3 1 2],'station',[2 1 1]),
%   phase2('single','vision','record'), phase2(...,'cmd')  (see runRound30 for the full list)
runRound30(varargin{:});
end
