function fleetMap2D(F,cmd)
%FLEETMAP2D  Live 2-D map of the whole warehouse with all robots (opens together with the 3-D window).
%   * floor plan: shelves (numbers + A/B faces), 3 packing stations, staging stands D01..D14, robot bays
%   * every robot is a disc of its real chassis size with a heading line and its name above; the carried box is drawn in front of it
%   * thin solid line = where the robot has driven on its current leg, dotted line = the route it is going to drive, x = its goal
%   * FIXED update scheme (no timer, no smoothing, no throttling - the map always shows the true state):
%       every fleet report (every 0.1 s of simulation time): robot positions, headings, carried boxes, routes, trails = the poses the 3-D robots have
%       every 10th report (1 s of simulation time): robot cards, station bars, events, header
%   * header: RUN TIME min:sec (real clock since the simulation started), simulation time, mode, tasks done, shipped
%   fleetMap2D(F,'draw') opens / refreshes everything;  fleetMap2D(F,'close') closes the window.
persistent fig H Fprev trail lastNav cnt tWall0 lastTxt nErr
try
    if nargin >= 2 && strcmpi(cmd,'close')
        if ~isempty(fig) && isgraphics(fig), delete(fig); end
        fig = []; return
    end
    force = nargin >= 2 && strcmpi(cmd,'draw');
    if isempty(fig) || ~isgraphics(fig) || isempty(Fprev) || Fprev ~= F
        old = findall(0,'Type','figure','Tag','Phase3Map'); delete(old);
        [fig,H] = localBuild(F); Fprev = F; trail = cell(1,F.N); lastNav = false(1,F.N); cnt = 0; tWall0 = []; lastTxt = ''; force = true;
    end
    cnt = cnt + 1;
    if ~force && F.mapEvery > 1 && mod(cnt,F.mapEvery) ~= 0, return, end     % perf light: positions every 3rd report (0.3 s); details still every 1 s
    if isempty(tWall0) && F.t > 0, tWall0 = tic; end
    N = F.N; rr = H.rr; nv = H.nv;
    % ---------------- every report: exact poses
    P = zeros(N,3); carry = false(1,N); known = false(1,N);
    for k = 1:N
        r = F.robots(k); if ~r.rep, P(k,:) = [F.C.P3.robots(k).home 0]; continue, end
        known(k) = true; P(k,:) = r.pose(1:3); carry(k) = r.carry;
    end
    Xv = P(:,1) + rr*cos(H.th0); Yv = P(:,2) + rr*sin(H.th0);
    set(H.rob,'Vertices',[reshape(Xv.',[],1) reshape(Yv.',[],1)]);
    hx = [P(:,1) + 0.3*cos(P(:,3)), P(:,1) + 1.0*cos(P(:,3)), nan(N,1)].'; hy = [P(:,2) + 0.3*sin(P(:,3)), P(:,2) + 1.0*sin(P(:,3)), nan(N,1)].';
    set(H.hdg,'XData',hx(:),'YData',hy(:));
    bx = P(:,1) + 0.55*cos(P(:,3)); by = P(:,2) + 0.55*sin(P(:,3)); d = 0.25;
    BV = [bx-d by-d; bx+d by-d; bx+d by+d; bx-d by+d]; BV = reshape(permute(reshape(BV,N,4,2),[2 1 3]),4*N,2);
    BV(repelem(~carry,4),:) = NaN; set(H.box,'Vertices',BV);
    for k = 1:N, set(H.lab(k),'Position',[P(k,1) P(k,2)+0.75 0]); end
    for k = 1:N
        if ~known(k), continue, end
        r = F.robots(k); nav = any(r.state == [1 5 7]);
        if nav && ~lastNav(k) || isempty(trail{k}), trail{k} = P(k,1:2); end          % a new driving leg starts a new line
        lastNav(k) = nav;
        if nav && norm(trail{k}(end,:) - P(k,1:2)) > 0.2, trail{k}(end+1,:) = P(k,1:2); end %#ok<AGROW>
        if size(trail{k},1) > 700, trail{k} = trail{k}(end-600:end,:); end
        if nav, set(H.trail(k),'XData',[trail{k}(:,1); P(k,1)],'YData',[trail{k}(:,2); P(k,2)]); else, set(H.trail(k),'XData',trail{k}(:,1),'YData',trail{k}(:,2)); end
        if nav && ~isempty(r.wp)
            R = [P(k,1:2); r.wp]; set(H.route(k),'XData',R(:,1),'YData',R(:,2));
            set(H.goal(k),'XData',r.goal(1),'YData',r.goal(2),'Visible','on');
        else
            set(H.route(k),'XData',NaN,'YData',NaN); set(H.goal(k),'Visible','off');
        end
    end
    % ---------------- every 10th report (1 s): panel
    if force || mod(cnt,10) < F.mapEvery
        localPanel(F,H,lastTxt,tWall0);
        ev = F.evlog; lines = {};
        for i = size(ev,1):-1:1
            m = ev{i,3}; if any(strncmp(m,{'DONE','FAULT','SHIP','ORDER','ASSIGN','AUTO','all r'},4))
                lines{end+1} = sprintf('%s  %s',localClock(ev{i,1}),m); %#ok<AGROW>
                if numel(lines) >= 8, break, end
            end
        end
        txt = strjoin(lines,newline); if ~strcmp(txt,lastTxt), set(H.ev,'String',txt); lastTxt = txt; end
    end
    if ~isempty(tWall0), set(H.clock,'String',localClock(toc(tWall0))); end
    drawnow limitrate
catch ME
    if isempty(nErr), nErr = 0; end
    nErr = nErr + 1; if nErr <= 3, fprintf('[MAP] 2-D map problem (the simulation is not affected): %s\n',ME.message); end
end
end

function localPanel(F,H,~,~)
C = F.C; names = {'IDLE','NAV TO SHELF','ARM EXTEND','GRASP','ARM RETRACT','NAV TO DEST','ARM PLACE','RETURN HOME','RELEASE','ARM STOW','DOCK','FAULT','DONE'};
for k = 1:F.N
    r = F.robots(k); if ~r.rep, continue, end
    nav = any(r.state == [1 5 7]);
    st = names{min(13,r.state+1)}; if r.fault, st = 'FAULT'; end, if r.stopNow, st = [st '  (STOP)']; elseif r.paused, st = [st '  (paused)']; end %#ok<AGROW>
    if r.scale < 0.3 && r.task > 0 && nav, st = [st '  - giving way']; end %#ok<AGROW>
    l2 = 'no task';
    if r.task > 0 && ~isempty(r.job), j = r.job; l2 = sprintf('%s  C%d   %s  ->  %s',j.type,j.carton,j.src.name,j.dst.name); end
    set(H.cName(k),'String',upper(r.name)); set(H.cState(k),'String',st); set(H.cTask(k),'String',l2);
    if r.fault, set(H.cBar(k),'FaceColor',[0.85 0.15 0.15]); else, set(H.cBar(k),'FaceColor',C.P3.robots(k).colour); end
end
[xs,ys,xd,yd,xt,yt] = deal([]);
for j = 1:F.M
    c = F.cartons(j);
    switch c.state
        case 'STORED', xs(end+1) = c.xyz(1); ys(end+1) = c.xyz(2); %#ok<AGROW>
        case 'DOCK',   xd(end+1) = c.xyz(1); yd(end+1) = c.xyz(2); %#ok<AGROW>
        case 'STATION'
            s = c.stationSlot; if s(1) > 0, q = C.Stations(s(1)).slotXYZ(s(2),1:2); xt(end+1) = q(1); yt(end+1) = q(2); end %#ok<AGROW>
    end
end
set(H.cs,'XData',xs,'YData',ys); set(H.cd,'XData',xd,'YData',yd); set(H.ct,'XData',xt,'YData',yt);
totOcc = 0; totCap = 0;
for s = 1:numel(F.stations)
    st = F.stations(s); nocc = sum(~strcmp(st.state,'FREE')); ncap = numel(st.state); totOcc = totOcc + nocc; totCap = totCap + ncap;
    set(H.stn(s),'String',sprintf('%s   %d / %d',C.Stations(s).name,sum(strcmp(st.state,'OCC')),ncap));
    set(H.sBar(s),'Position',[H.barX H.sY(s) max(1e-4,H.barW*nocc/ncap) H.barH]); set(H.sTxt(s),'String',sprintf('%d / %d',nocc,ncap));
end
dk = strcmp({F.slots.role},'dock'); nd = sum(~strcmp({F.slots(dk).state},'FREE')); ndTot = sum(dk);
set(H.dBar,'Position',[H.barX H.dY max(1e-4,H.barW*nd/ndTot) H.barH]); set(H.dTxt,'String',sprintf('%d / %d',nd,ndTot));
set(H.tBar,'Position',[H.barX H.tY max(1e-4,H.barW*totOcc/totCap) H.barH]); set(H.tTxt,'String',sprintf('%d / %d',totOcc,totCap));
nDone = sum(strcmp({F.tasks.status},'DONE'));
set(H.chip,'String',sprintf('SIM  %s      MODE  %s      TASKS DONE  %d      SHIPPED  %d',localClock(F.t),localCap(F.mode),nDone,F.shipped));
end

function s = localCap(w)
w = char(w); if isempty(w), s = w; else, s = [upper(w(1)) lower(w(2:end))]; end
end

function s = localClock(sec)
sec = max(0,floor(sec)); s = sprintf('%d:%02d',floor(sec/60),mod(sec,60));
end

% ------------------------------------------------------------------------------------------------------------ static drawing
function [fig,H] = localBuild(F)
C = F.C; WH = C.WH; N = F.N;
bg = [0.945 0.949 0.957]; ink = [0.12 0.15 0.20]; grey = [0.45 0.48 0.53]; fnt = 'Segoe UI';
fig = figure('Name','Warehouse - live 2-D map (all robots)','NumberTitle','off','Tag','Phase3Map','Color',bg,'Position',[40 40 1320 840],'MenuBar','none','ToolBar','none');
% ---- header band
axh = axes('Parent',fig,'Position',[0 0.952 1 0.048],'Visible','off','XLim',[0 1],'YLim',[0 1]); hold(axh,'on');
patch(axh,[0 1 1 0],[0 0 1 1],[0.09 0.12 0.18],'EdgeColor','none');
text(axh,0.018,0.52,'WAREHOUSE FLEET  -  LIVE MAP','Color','w','FontName',fnt,'FontSize',12.5,'FontWeight','bold','VerticalAlignment','middle');
text(axh,0.435,0.50,'RUN TIME  (min:sec)','Color',[0.62 0.68 0.78],'FontName',fnt,'FontSize',8,'HorizontalAlignment','right','VerticalAlignment','middle');
H.clock = text(axh,0.445,0.48,'0:00','Color',[0.40 0.88 0.60],'FontName','Consolas','FontSize',16,'FontWeight','bold','HorizontalAlignment','left','VerticalAlignment','middle');
H.chip = text(axh,0.985,0.52,'','Color',[0.85 0.89 0.95],'FontName',fnt,'FontSize',10,'HorizontalAlignment','right','VerticalAlignment','middle','Interpreter','none');
% ---- floor plan
ax = axes('Parent',fig,'Position',[0.015 0.07 0.45 0.875]); hold(ax,'on'); axis(ax,'equal'); axis(ax,[-WH.X/2-1 WH.X/2+1 -1 WH.Y+1]); box(ax,'on');
set(ax,'XTick',-30:10:30,'YTick',0:10:80,'Color',[0.985 0.985 0.99],'FontName',fnt,'FontSize',8,'XColor',grey,'YColor',grey); grid(ax,'on'); try, ax.GridAlpha = 0.12; catch, end
rectangle(ax,'Position',[-WH.X/2 0 WH.X WH.Y],'EdgeColor',[0.25 0.28 0.33],'LineWidth',2,'FaceColor',[0.925 0.93 0.94]);
for m = 1:size(WH.mats,1), M = WH.mats(m,:); rectangle(ax,'Position',[M(1)-M(3)/2 M(2)-M(4)/2 M(3) M(4)],'FaceColor',[0.60 0.62 0.67],'EdgeColor','none','Curvature',[0.05 0.05]); end
for k = 1:size(WH.shelves,1)
    d = WH.shelves(k,:); rectangle(ax,'Position',[d(1)-d(4)/2 d(2)-d(3)/2 d(4) d(3)],'FaceColor',[0.17 0.30 0.58],'EdgeColor',[0.08 0.14 0.30],'LineWidth',1,'Curvature',[0.04 0.04]);
    text(ax,d(1),d(2),sprintf('%d',k),'Color','w','FontName',fnt,'FontWeight','bold','FontSize',12,'HorizontalAlignment','center');
    text(ax,d(1)-d(4)/2-0.3,d(2)+d(3)/2+0.6,'A','Color',[0.25 0.28 0.55],'FontName',fnt,'FontSize',8,'HorizontalAlignment','right');
    text(ax,d(1)+d(4)/2+0.3,d(2)+d(3)/2+0.6,'B','Color',[0.25 0.28 0.55],'FontName',fnt,'FontSize',8,'HorizontalAlignment','left');
end
H.stn = gobjects(1,size(WH.tables,1));
for t = 1:size(WH.tables,1)
    T = WH.tables(t,:); rectangle(ax,'Position',[T(1)-T(3)/2 T(2)-T(4)/2 T(3) T(4)],'FaceColor',[0.90 0.80 0.58],'EdgeColor',[0.45 0.35 0.15],'LineWidth',1,'Curvature',[0.08 0.08]);
    H.stn(t) = text(ax,T(1),T(2),sprintf('%s',C.Stations(t).name),'FontName',fnt,'FontSize',9,'FontWeight','bold','HorizontalAlignment','center','Color',[0.25 0.18 0.05]);
end
for i = 1:size(WH.stands,1)
    s = WH.stands(i,:); rectangle(ax,'Position',[s(1)-0.35 s(2)-0.35 0.7 0.7],'FaceColor',[0.95 0.52 0.10],'EdgeColor','none','Curvature',[0.2 0.2]);
    text(ax,s(1),s(2)-0.9,sprintf('D%02d',i),'FontName',fnt,'FontSize',6.5,'HorizontalAlignment','center','Color',[0.45 0.25 0.02]);
end
for k = 1:N
    h = C.P3.robots(k).home; rectangle(ax,'Position',[h(1)-0.6 h(2)-0.6 1.2 1.2],'Curvature',[1 1],'EdgeColor',C.P3.robots(k).colour,'LineStyle','--','LineWidth',1.2);
end
H.cs = plot(ax,NaN,NaN,'s','MarkerSize',3,'MarkerFaceColor',[0.60 0.40 0.18],'MarkerEdgeColor','none');
H.cd = plot(ax,NaN,NaN,'s','MarkerSize',6,'MarkerFaceColor',[0.60 0.40 0.18],'MarkerEdgeColor','k');
H.ct = plot(ax,NaN,NaN,'s','MarkerSize',5,'MarkerFaceColor',[0.60 0.40 0.18],'MarkerEdgeColor','k');
for k = 1:N
    col = C.P3.robots(k).colour; lite = 0.75*col + 0.25;
    H.trail(k) = plot(ax,NaN,NaN,'-','Color',lite,'LineWidth',1.3);
    H.route(k) = plot(ax,NaN,NaN,':','Color',col,'LineWidth',2.2);
    H.goal(k)  = plot(ax,NaN,NaN,'x','Color',col,'LineWidth',2,'MarkerSize',9,'Visible','off');
end
rr = F.TP.body/2; nv = 24; th0 = linspace(0,2*pi,nv+1); th0(end) = [];
H.rr = rr; H.th0 = th0; H.nv = nv;
Hm = zeros(N,2); cols = zeros(N,3); for k = 1:N, Hm(k,:) = C.P3.robots(k).home; cols(k,:) = C.P3.robots(k).colour; end
Xv = Hm(:,1) + rr*cos(th0); Yv = Hm(:,2) + rr*sin(th0);               % N x nv: every robot is a disc of its real chassis radius
H.rob = patch('Parent',ax,'Vertices',[reshape(Xv.',[],1) reshape(Yv.',[],1)],'Faces',(0:N-1).'*nv + (1:nv),'FaceVertexCData',cols,'FaceColor','flat','EdgeColor',[1 1 1],'LineWidth',1.4);
H.hdg = plot(ax,NaN,NaN,'-','Color',[0.1 0.1 0.1],'LineWidth',2.2);
H.box = patch('Parent',ax,'Vertices',nan(4*N,2),'Faces',(0:N-1).'*4 + (1:4),'FaceColor',[0.72 0.50 0.25],'EdgeColor',[0.3 0.2 0.1]);
H.lab = gobjects(1,N);
for k = 1:N
    col = C.P3.robots(k).colour; h = C.P3.robots(k).home;
    H.lab(k) = text(ax,h(1),h(2)+0.75,C.P3.robots(k).name,'FontName',fnt,'FontSize',7,'FontWeight','bold','HorizontalAlignment','center','VerticalAlignment','bottom','Color',min(col*0.6,0.5));
end
if ~isempty(WH.mats), text(ax,mean(WH.mats(:,1)),min(WH.mats(:,2)-WH.mats(:,4)/2)-1.0,'STAGING AREA','FontName',fnt,'FontSize',6.5,'Color','k','HorizontalAlignment','center','FontWeight','normal','BackgroundColor','none','EdgeColor','none','Clipping','off'); end      % drawn last (on top of the stands and bay circles), black
axl = axes('Parent',fig,'Position',[0.015 0.012 0.45 0.04],'Visible','off','XLim',[0 1],'YLim',[0 1]);
text(axl,0.5,0.5,'solid line = driven    |    dotted line = planned route    |    x = goal    |    brown square = carried box','FontName',fnt,'FontSize',9,'FontWeight','bold','Color',[0.10 0.12 0.18],'HorizontalAlignment','center','VerticalAlignment','middle');
% ---- side panel
ax2 = axes('Parent',fig,'Position',[0.485 0.07 0.40 0.875],'Visible','off','XLim',[0 1],'YLim',[0 1]); hold(ax2,'on');
text(ax2,0,0.995,'ROBOTS','FontName',fnt,'FontSize',9,'FontWeight','bold','Color',grey,'VerticalAlignment','top');
H.cName = gobjects(1,N); H.cState = H.cName; H.cTask = H.cName; H.cBar = H.cName;
ch = 0.088; gap = 0.0095; top0 = 0.968;
for k = 1:N
    col = C.P3.robots(k).colour; top = top0 - (k-1)*(ch+gap); bot = top - ch;
    rectangle(ax2,'Position',[0 bot 1 ch],'FaceColor',[1 1 1],'EdgeColor',[0.82 0.84 0.88],'Curvature',[0.04 0.2]);
    H.cBar(k) = rectangle(ax2,'Position',[0 bot 0.028 ch],'FaceColor',col,'EdgeColor','none');
    H.cName(k) = text(ax2,0.05,top-0.022,upper(C.P3.robots(k).name),'FontName',fnt,'FontSize',10,'FontWeight','bold','Color',ink,'VerticalAlignment','middle');
    H.cState(k) = text(ax2,0.985,top-0.022,'IDLE','FontName',fnt,'FontSize',8.5,'FontWeight','bold','Color',[0.30 0.45 0.75],'HorizontalAlignment','right','VerticalAlignment','middle','Interpreter','none');
    H.cTask(k) = text(ax2,0.05,top-0.058,'no task','FontName','Consolas','FontSize',8.5,'Color',grey,'VerticalAlignment','middle','Interpreter','none');
end
yS = top0 - N*(ch+gap) - 0.012;
text(ax2,0,yS,'PACKING STATIONS  /  STAGING','FontName',fnt,'FontSize',9,'FontWeight','bold','Color',grey,'VerticalAlignment','top');
H.barX = 0.40; H.barW = 0.46; H.barH = 0.016; H.sY = zeros(1,numel(F.stations)); H.sBar = gobjects(1,numel(F.stations)); H.sTxt = H.sBar;
y = yS - 0.040;
for s = 1:numel(F.stations)
    H.sY(s) = y; text(ax2,0,y+0.008,sprintf('%s',C.Stations(s).name),'FontName',fnt,'FontSize',9,'Color',ink,'VerticalAlignment','middle');
    rectangle(ax2,'Position',[H.barX y H.barW H.barH],'FaceColor',[0.86 0.88 0.91],'EdgeColor','none','Curvature',[0.5 0.5]);
    H.sBar(s) = rectangle(ax2,'Position',[H.barX y 1e-4 H.barH],'FaceColor',[0.90 0.62 0.12],'EdgeColor','none','Curvature',[0.5 0.5]);
    H.sTxt(s) = text(ax2,0.99,y+0.008,'0 / 0','FontName','Consolas','FontSize',9,'Color',ink,'HorizontalAlignment','right','VerticalAlignment','middle');
    y = y - 0.034;
end
H.dY = y; text(ax2,0,y+0.008,'Staging stands','FontName',fnt,'FontSize',9,'Color',ink,'VerticalAlignment','middle');
rectangle(ax2,'Position',[H.barX y H.barW H.barH],'FaceColor',[0.86 0.88 0.91],'EdgeColor','none','Curvature',[0.5 0.5]);
H.dBar = rectangle(ax2,'Position',[H.barX y 1e-4 H.barH],'FaceColor',[0.95 0.52 0.10],'EdgeColor','none','Curvature',[0.5 0.5]);
H.dTxt = text(ax2,0.99,y+0.008,'0 / 0','FontName','Consolas','FontSize',9,'Color',ink,'HorizontalAlignment','right','VerticalAlignment','middle');
y = y - 0.034; H.tY = y; text(ax2,0,y+0.008,'All stations (goal)','FontName',fnt,'FontSize',9,'FontWeight','bold','Color',ink,'VerticalAlignment','middle');
rectangle(ax2,'Position',[H.barX y H.barW H.barH],'FaceColor',[0.86 0.88 0.91],'EdgeColor','none','Curvature',[0.5 0.5]);
H.tBar = rectangle(ax2,'Position',[H.barX y 1e-4 H.barH],'FaceColor',[0.20 0.65 0.40],'EdgeColor','none','Curvature',[0.5 0.5]);
H.tTxt = text(ax2,0.99,y+0.008,'0 / 0','FontName','Consolas','FontSize',9,'FontWeight','bold','Color',ink,'HorizontalAlignment','right','VerticalAlignment','middle');
y = y - 0.035; text(ax2,0,y,'LAST EVENTS','FontName',fnt,'FontSize',9,'FontWeight','bold','Color',grey,'VerticalAlignment','top');
H.ev = text(ax2,0,y-0.03,'','FontName','Consolas','FontSize',8,'Color',ink,'VerticalAlignment','top','Interpreter','none');
end
