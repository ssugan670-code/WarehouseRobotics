function robotVisionViewer(img, pose, q, record)
%ROBOTVISIONVIEWER  Live "robot eye" window: the camera picture with the robot's live numbers written ON the picture.
% Called every camera frame by the Vision Viewer block. It only READS (image, base pose, commanded joints,
% Phase2MissionStatus); it never writes to the model or the controller, and every error is swallowed, so
% this window can never disturb the run.
%   record=true : also saves the window as an .mp4 plus a .csv of the numbers, in  <project>\RobotVideos
%                 (separate folder, next to the debug folder, not inside src\Phase2).
%   robotVisionViewer('close') : finalises the mp4/csv (the model's StopFcn calls this when the simulation stops).
persistent fig im ax hud vw fid frameNo robot C tLast videoFile csvFile sz0
try
    if ischar(img) || isstring(img)
        if strcmpi(char(img),'close')
            if ~isempty(vw), try, close(vw); fprintf('[VISION] video saved: %s\n',videoFile); catch, end, end
            if ~isempty(fid) && fid>0, try, fclose(fid); fprintf('[VISION] robot log saved: %s\n',csvFile); catch, end, end
            vw=[]; fid=[]; tLast=[]; sz0=[];
        end
        return
    end
    if nargin<4, record=false; end
    if ~isempty(tLast) && toc(tLast)<0.09, return, end      % the block runs at 50 Hz; redraw at ~10 Hz
    tLast=tic;
    if isempty(frameNo), frameNo=0; end
    img=uint8(img);
    if frameNo>0 && (isempty(fig) || ~isgraphics(fig)), return, end        % user closed the window: stay closed
    if isempty(fig)
        fig=figure('Name','Robot Vision - live camera on the robot','NumberTitle','off','Tag','Phase2RobotVision', ...
            'MenuBar','none','ToolBar','none','Color','k','Position',[80 80 720 540]);
        ax=axes('Parent',fig,'Position',[0 0 1 1]);
        im=image(ax,img); axis(ax,'off'); set(ax,'XLim',[0.5 640.5],'YLim',[0.5 480.5],'YDir','reverse');
        hud=localMakeHud(ax);
        try, evalc('C=WarehouseRobotPhase2Config();'); catch, C=[]; end      % evalc: keep the command window quiet
        try, robot=helperBuildMobileManipulator(); catch, robot=[]; end
        if record
            try, root=fileparts(C.Debug.SnapshotDir); catch, root=fullfile(pwd,'..'); end
            base=fullfile(root,'robot rec');                                  % C:\WarehouseRobotics\robot rec
            dirV=fullfile(base,'videos'); dirL=fullfile(base,'robot logs');
            if ~isfolder(dirV), mkdir(dirV); end
            if ~isfolder(dirL), mkdir(dirL); end
            stamp=datestr(now,'yyyymmdd_HHMMSS');
            videoFile=fullfile(dirV,['RobotVision_' stamp '.mp4']);
            csvFile=fullfile(dirL,['RobotVision_' stamp '.csv']);
            vw=VideoWriter(videoFile,'MPEG-4'); vw.FrameRate=10; vw.Quality=70; open(vw); sz0=[];
            fid=fopen(csvFile,'w');
            if fid>0, fprintf(fid,'t_s,state,carton,x_m,y_m,yaw_deg,lift_m,q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg,q7_deg,hand_fwd_m,hand_y_m,hand_z_m,arm_cross_deg,goal_dist_m,goal_turn_deg\n'); end
            fprintf('[VISION] recording: videos -> %s | robot logs -> %s\n',dirV,dirL);
        end
    end
    frameNo=frameNo+1;
    set(im,'CData',img);
    [txt,m]=localInfo(pose(:).',q(:),C,robot);
    localSetHud(hud,txt);
    drawnow limitrate
    if ~isempty(vw)
        try
            c=getframe(fig); c=c.cdata;                                       % picture + numbers, as on screen
            if isempty(sz0), sz0=size(c); end
            if ~isequal(size(c),sz0)                                          % window was resized/maximised: keep the video size constant
                c=c(round(linspace(1,size(c,1),sz0(1))),round(linspace(1,size(c,2),sz0(2))),:);
            end
            writeVideo(vw,c);
        catch
        end
        if ~isempty(fid) && fid>0, try, fprintf(fid,'%s\n',m); catch, end, end
    end
catch
end
end

% ------------------------------------------------------------------ HUD (letters and numbers only, no boxes)
function hud=localMakeHud(ax)
% slot: x,y (pixels of the 640x480 picture), horizontal alignment, vertical alignment
spec={ 'tl', 22,  14,'left','top'; ...      % top-left corner: carton number
       't1',100,  14,'left','top'; ...      % top row, left:   what the robot is doing
       't2',272,  14,'left','top'; ...      % top row, middle: which carton -> which station
       't3',440,  14,'left','top'; ...      % top row, right:  where to / how far / how much to turn
       'r', 626, 150,'right','top'; ...     % right side:      lift now / needed
       'b1', 30, 458,'left','bottom'; ...   % bottom left:     hand reach
       'b2',420, 458,'left','bottom'};      % bottom right:    arm cross + camera
hud=struct();
for k=1:size(spec,1)
    sh=text(ax,spec{k,2}+1.2,spec{k,3}+1.2,'','Color','k','FontName','Consolas','FontSize',12,'FontWeight','bold', ...
        'HorizontalAlignment',spec{k,4},'VerticalAlignment',spec{k,5},'Interpreter','none');   % thin shadow keeps it readable
    tx=text(ax,spec{k,2},spec{k,3},'','Color',[0.6 1 0.6],'FontName','Consolas','FontSize',12,'FontWeight','bold', ...
        'HorizontalAlignment',spec{k,4},'VerticalAlignment',spec{k,5},'Interpreter','none');
    hud.(spec{k,1})=[sh tx];
end
end

function localSetHud(hud,t)
f=fieldnames(hud);
for k=1:numel(f)
    s=''; if isfield(t,f{k}), s=t.(f{k}); end
    set(hud.(f{k}),'String',s);
end
end

% ------------------------------------------------------------------ numbers
function [t,csvLine]=localInfo(pose,q,C,robot)
names={'IDLE','NAV TO SHELF','ARM EXTEND','GRASP','ARM RETRACT','NAV TO STATION','ARM PLACE','RETURN HOME','RELEASE','ARM STOW','DOCK AT SHELF','FAULT HOLD','MISSION DONE'};
t=struct('tl','','t1','ROBOT VISION','t2','','t3','','r','','b1','','b2',''); csvLine='';
try
    st=evalin('base','Phase2MissionStatus'); id=st.state; sel=st.sel;
catch
    return
end
nan_=NaN; dist=nan_; turn=nan_; hf=nan_; hy=nan_; hz=nan_; cross=nan_;
try
    t.t1=names{id+1};
    if sel>0, t.tl=sprintf('#%d',sel); end
    if sel>0 && ~isempty(C)
        b=C.Boxes(sel); s=sprintf('shelf %d  level %d',b.shelf,b.level); stn=0; try, stn=st.station; catch, end
        if stn>=1 && stn<=numel(C.Stations), s=sprintf('%s  >  STATION %d',s,stn); end
        t.t2=s;
    end
    if any(id==[1 10 5 7]) && isfield(st,'goal') && numel(st.goal)==2
        d=st.goal(:).'-pose(1:2); dist=hypot(d(1),d(2));
        turn=atan2(d(2),d(1))-pose(3); turn=atan2(sin(turn),cos(turn))*180/pi;     % + = goal is to the left
        if abs(turn)<2, tn='straight'; elseif turn>0, tn=sprintf('left %.0f deg',turn); else, tn=sprintf('right %.0f deg',-turn); end
        t.t3=sprintf('GOAL %.1f m  %s',dist,tn);
    end
    if numel(q)>=8
        s=sprintf('LIFT\n%.2f m',q(1));
        if sel>0 && ~isempty(C) && any(id==[1 10 2 3 4]), s=sprintf('%s\nNEED\n%.2f m',s,C.Boxes(sel).lift); end
        t.r=s;
    end
    if numel(q)>=8 && ~isempty(robot)
        T=getTransform(robot,q(1:8),'ToolTip','mobile_base_root');
        hf=T(1,4)-0.10; hy=T(2,4); hz=T(3,4); cross=atan2(hy,hf)*180/pi;           % + = hand is to the left of the robot centre line
        s=sprintf('HAND %.2f m ahead   %.2f m high',hf,hz);
        if sel>0 && ~isempty(C) && any(id==[2 3])
            R=[cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))];
            hw=pose(1:2).'+R*T(1:2,4); g=C.Boxes(sel).xyz(:);
            s=sprintf('%s   %.2f m to grasp',s,max(0,norm([hw;T(3,4)]-g)-norm(C.GraspOffsetTrans)));
        end
        t.b1=s;
        t.b2=sprintf('ARM %+.0f deg off centre\nq1 %+.0f deg',cross,q(2)*180/pi);
    end
catch
end
try
    csvLine=sprintf('%.2f,%d,%d,%.3f,%.3f,%.1f,%.3f,%s,%.3f,%.3f,%.3f,%.1f,%.2f,%.1f',st.time,id,sel,pose(1),pose(2),pose(3)*180/pi,q(1), ...
        strjoin(arrayfun(@(v) sprintf('%.1f',v*180/pi),q(2:8).','UniformOutput',false),','),hf,hy,hz,cross,dist,turn);
catch
end
end
