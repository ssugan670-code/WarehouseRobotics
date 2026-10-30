function [G,res,off] = p3StaticGrid(C,res)
%P3STATICGRID  The warehouse as the robots' lidar map: logical occupancy grid of everything that is fixed (walls, shelves, packing tables,
%   office block, staging stands).  Pure MATLAB (no toolbox): the same grid is used to build the Navigation Toolbox occupancyMap for
%   localization (p3Map), to ray-cast the fallback lidar, and to decide which lidar points are "explained by the map".
%   G(row=y, col=x) true = occupied.   World -> cell:  col = floor((x+off(1))*res)+1, row = floor((y+off(2))*res)+1   (off = [X/2 0]).
%   Footprints are the real ones from the layout (nothing is added or removed to make localization easier).
if nargin<2 || isempty(res), res = 20; end
WH = C.WH; off = [WH.X/2 0];
nx = ceil(WH.X*res); ny = ceil(WH.Y*res); G = false(ny,nx);
w = max(2,ceil(0.15*res));                                   % outer wall, 15 cm
G(1:w,:) = true; G(end-w+1:end,:) = true; G(:,1:w) = true; G(:,end-w+1:end) = true;
for k = 1:size(WH.shelves,1), G = localRect(G,res,off,WH.shelves(k,1),WH.shelves(k,2),WH.shelves(k,4),WH.shelves(k,3)); end
for k = 1:size(WH.tables,1),  G = localRect(G,res,off,WH.tables(k,1),WH.tables(k,2),WH.tables(k,3),WH.tables(k,4)); end
if isfield(WH,'offCx') && WH.offCx > -100, G = localRect(G,res,off,WH.offCx,WH.offCy,WH.offW,WH.offD); end
if isfield(WH,'stands')
    for k = 1:size(WH.stands,1), G = localRect(G,res,off,WH.stands(k,1),WH.stands(k,2),WH.standSize,WH.standSize); end
end
end
function G = localRect(G,res,off,cx,cy,w,d)
c0 = max(1,floor((cx - w/2 + off(1))*res)+1); c1 = min(size(G,2),ceil((cx + w/2 + off(1))*res));
r0 = max(1,floor((cy - d/2 + off(2))*res)+1); r1 = min(size(G,1),ceil((cy + d/2 + off(2))*res));
if c1 >= c0 && r1 >= r0, G(r0:r1,c0:c1) = true; end
end
