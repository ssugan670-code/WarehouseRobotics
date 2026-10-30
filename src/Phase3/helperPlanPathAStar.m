function waypoints = helperPlanPathAStar(WH,startXY,goalXY)
%HELPERPLANPATHASTAR Phase 1 A* helper retained as an independent dependency.
% Coordinates supplied/returned use Phase 1 centred-X warehouse coordinates.
% Simulink vector signals can be column-shaped whereas MATLAB test calls use
% row vectors. Normalise at the API boundary before concatenating route rows.
startXY=reshape(startXY,1,2);
goalXY=reshape(goalXY,1,2);
map = WH.navMap;
startMapXY = [startXY(1)+WH.X/2, startXY(2)];
goalMapXY  = [goalXY(1)+WH.X/2, goalXY(2)];
% plannerAStarGrid rejects occupied cells. Dock points are deliberately near
% the inflated boundary of a rack/table, so resolve them to the nearest free
% grid cell rather than terminating the complete co-simulation.
startMapXY=localNearestFree(map,startMapXY,'start');
goalMapXY=localNearestFree(map,goalMapXY,'goal');
startIJ = world2grid(map,startMapXY); goalIJ = world2grid(map,goalMapXY);
% Axis-aligned cell transitions deliberately avoid a diagonal move through
% the inside corner formed by two inflated obstacles. The mobile base then
% has a physically traversable corridor, not merely free waypoint centres.
planner = plannerAStarGrid(map,'GCost','Euclidean','HCost','Euclidean','DiagonalSearch','off');
pathIJ = plan(planner,startIJ,goalIJ);
if isempty(pathIJ)
    warning('helperPlanPathAStar:NoPath','No path found between start and goal.');
    waypoints = [startXY;goalXY];
    return
end
waypoints = grid2world(map,pathIJ);
waypoints(:,1) = waypoints(:,1)-WH.X/2;

% plannerAStarGrid returns grid cells, whose ordering has varied between
% Navigation Toolbox releases. Normalise the route before sending it to the
% feedback controller. Remove repeated cells too: a duplicate can become a
% zero-length carrot and stop the base well before the real goal.
if norm(waypoints(1,:)-startXY)>norm(waypoints(end,:)-startXY)
    waypoints=flipud(waypoints);
end
keep=[true; vecnorm(diff(waypoints,1,1),2,2)>1e-9];
waypoints=waypoints(keep,:);
if norm(waypoints(1,:)-startXY)>0.08
    waypoints=[startXY;waypoints];
else
    waypoints(1,:)=startXY;
end
if norm(waypoints(end,:)-goalXY)>0.08
    waypoints=[waypoints;goalXY];
else
    waypoints(end,:)=goalXY;
end
waypoints=localKeepCorners(waypoints);
end

function points=localKeepCorners(points)
% An occupancy-grid route has one 10 cm point per cell.  Feeding those
% points to a look-ahead follower rounds the right-angle corners and cuts
% into inflated obstacles. Keep only endpoints and genuine direction
% changes, so feedback travels straight through free corridors and makes a
% complete alignment turn at each planned corner.
if size(points,1)<3, return; end
delta=diff(points,1,1);
keep=[true; abs(delta(1:end-1,1).*delta(2:end,2)- ...
                delta(1:end-1,2).*delta(2:end,1))>1e-8; true];
points=points(keep,:);
end

function point=localNearestFree(map,point,label)
limits=[map.XWorldLimits;map.YWorldLimits];
point=min(max(point,[limits(1,1)+0.05 limits(2,1)+0.05]), ...
              [limits(1,2)-0.05 limits(2,2)-0.05]);
if getOccupancy(map,point)<0.5, return; end
resolution=map.Resolution;
for cells=1:ceil(2.0*resolution)
    [dx,dy]=meshgrid(-cells:cells,-cells:cells);
    edge=max(abs(dx),abs(dy))==cells;
    candidates=point+[dx(edge) dy(edge)]/resolution;
    inside=candidates(:,1)>=limits(1,1) & candidates(:,1)<=limits(1,2) & ...
           candidates(:,2)>=limits(2,1) & candidates(:,2)<=limits(2,2);
    candidates=candidates(inside,:);
    free=getOccupancy(map,candidates)<0.5;
    if any(free)
        candidates=candidates(free,:);
        [~,index]=min(sum((candidates-point).^2,2)); point=candidates(index,:);
        return
    end
end
error('helperPlanPathAStar:NoFreeCell','No free %s cell exists within 2 m.',label);
end
