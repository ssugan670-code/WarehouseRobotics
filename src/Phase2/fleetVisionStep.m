function fleetVisionStep(img,pose,lift,sel)
%FLEETVISIONSTEP  Called by the 'Fleet Vision Viewer' block at every camera frame (10 Hz).
%   1. hands the picture to the fleet manager, which uses it to verify the carton at the shelf slot (computer vision in the loop)
%   2. if a robot is being watched (p3 watch red) shows the live picture with the robot's numbers; 'p3 watch record' also saves an mp4.
%   fleetVisionStep('close') finalises the video.  Every error is swallowed: this block can never disturb the run.
persistent fig im ax hud vw videoFile tLast
try
    if ischar(img) || isstring(img)
        if strcmpi(char(img),'close')
            if ~isempty(vw), try, close(vw); fprintf('[VISION] video saved: %s\n',videoFile); catch, end, end
            vw = []; tLast = [];
        end
        return
    end
    F = evalin('base','Phase3Fleet'); sel = double(sel);
    img = uint8(img); pose = double(pose(:).'); lift = double(lift);
    F.visionFrame(sel,img,pose,lift);
    if F.watch == 0 && ~isempty(fig) && isgraphics(fig)       % p3 cam off: close the window and finish the video
        delete(fig); fig = []; if ~isempty(vw), try, close(vw); fprintf('[VISION] video saved: %s\n',videoFile); catch, end, vw = []; end, end
    if F.watch ~= sel, return, end
    if ~isempty(tLast) && toc(tLast) < 0.09, return, end
    tLast = tic;
    if isempty(fig) || ~isgraphics(fig)
        fig = figure('Name','Fleet camera - live','NumberTitle','off','Tag','Phase3Camera','MenuBar','none','ToolBar','none','Color','k','Position',[80 80 720 540],'CloseRequestFcn',@(h,~) localClosed(h));
        ax = axes('Parent',fig,'Position',[0 0 1 1]); im = image(ax,img); axis(ax,'off'); set(ax,'XLim',[0.5 640.5],'YLim',[0.5 480.5],'YDir','reverse');
        hud = [text(ax,21,15,'','Color','k','FontName','Consolas','FontSize',12,'FontWeight','bold','VerticalAlignment','top','Interpreter','none') ...
               text(ax,20,14,'','Color',[0.6 1 0.6],'FontName','Consolas','FontSize',12,'FontWeight','bold','VerticalAlignment','top','Interpreter','none')];
        if F.recordVideo
            base = phase3Paths().videos; if ~isfolder(base), mkdir(base); end
            videoFile = fullfile(base,['Phase3Camera_' datestr(now,'yyyymmdd_HHMMSS') '.mp4']);
            vw = VideoWriter(videoFile,'MPEG-4'); vw.FrameRate = 10; vw.Quality = 70; open(vw);
            fprintf('[VISION] recording %s\n',videoFile);
        end
    end
    try, img = p3CamOverlay(img,F.camOv{sel}); catch, end      % yellow = search window, green = detected outline, magenta cross = expected centre (same as cam_*.png)
    set(im,'CData',img);
    r = F.robots(sel); names = {'IDLE','NAV TO SHELF','ARM EXTEND','GRASP','ARM RETRACT','NAV TO DEST','ARM PLACE','RETURN HOME','RELEASE','ARM STOW','DOCK','FAULT','DONE'};
    txt = sprintf('%s   %s   lift %.2f m\n',upper(r.name),names{min(13,r.state+1)},lift);
    if r.task > 0 && ~isempty(r.job), txt = sprintf('%stask %d: C%d  %s -> %s\n',txt,r.task,r.job.carton,r.job.src.name,r.job.dst.name); end
    txt = sprintf('%sat (%.1f, %.1f)   camera check: %s',txt,r.pose(1),r.pose(2),r.vis);
    set(hud,'String',txt);
    drawnow limitrate
    if ~isempty(vw), try, c = getframe(fig); writeVideo(vw,c.cdata); catch, end, end
catch
end
end

function localClosed(h)
% the X button of the camera window = p3 cam off
try, F = evalin('base','Phase3Fleet'); F.watch = 0; catch, end
delete(h);
end
