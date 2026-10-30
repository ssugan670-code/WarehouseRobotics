function p3(varargin)
%P3  Command interface of the running multi-robot warehouse.  Type  p3 help  for the list.
% Command syntax (no brackets, no quotes):   p3 dock add 2    p3 red retrieve 3A-L2-P04 station 1    p3 all pause
if nargin==0, varargin = {'help'}; end
a = cellfun(@(x) strtrim(char(string(x))),varargin,'UniformOutput',false);
if strcmpi(a{1},'help'), localHelp(); return, end
try, F = evalin('base','Phase3Fleet'); catch, fprintf('No fleet running. Start with  phase2\n'); return, end
cmd = lower(a{1}); rest = a(2:end);
if strcmp(F.mode,'standby') && ~any(strcmp(cmd,{'help','mode','wms','status','cartons','map','cam','camera','watch','rec','shelves','snapshot','verbose','metrics','end','stop','queue','add','remove','clear','reset'}))
    fprintf('[P3] first choose a mode:   p3 mode auto   |   p3 mode manual   |   p3 mode batch      (p3 help)\n'); return
end
switch cmd
    case 'help', localHelp();
    case 'wms', arg = ''; if ~isempty(rest), arg = rest{1}; end, fprintf('%s',F.wmsText(arg));
    case 'urgent'       % p3 urgent <slot|C<id>> [station <n>]
        if isempty(rest), fprintf('usage: p3 urgent <slot or C<id>> [station <n>]     e.g.  p3 urgent 8B-L3-P06 station 1\n'); return, end
        st = 0; for i = 2:numel(rest)-1, if strcmpi(rest{i},'station'), st = str2double(rest{i+1}); end, end
        F.urgentJob(rest{1},p3tern(isfinite(st),st,0));
    case 'move'         % p3 move <slot|C<id>> station [n] | <slot>   [robot <name>]
        if numel(rest) < 2, fprintf('usage: p3 move <slot or C<id>> station [n]     |     p3 move D03 <shelf slot>     [robot <name>]\n'); return, end
        k = 0; st = 0; ri = find(strcmpi(rest,'robot'),1); if ~isempty(ri) && ri < numel(rest), k = F.robotIndex(rest{ri+1}); rest = rest([1:ri-1 ri+2:end]); end
        if strcmpi(rest{2},'station') && numel(rest) >= 3, st = str2double(rest{3}); if ~isfinite(st), st = 0; end, end
        F.moveJob(rest{1},rest{2},st,k);
    case 'dock'
        if isempty(rest) || ~strcmpi(rest{1},'add'), fprintf('usage: p3 dock add <n>   or   p3 dock add C<id> [C<id> ...]\n'); return, end
        if numel(rest)>=2 && ~isempty(regexp(rest{2},'^[cC]\d+$','once'))
            F.dockAdd(0,false,cellfun(@(s) str2double(s(2:end)),rest(2:end)));
        else
            n = 1; if numel(rest)>=2, n = str2double(rest{2}); end
            if ~isfinite(n) || n<1, fprintf('usage: p3 dock add <n>\n'); return, end
            F.dockAdd(round(n),false);
        end
    case 'putaway'      % p3 putaway all | p3 putaway C9 [slot]   (any free robot)
        if isempty(rest), fprintf('usage: p3 putaway all   or   p3 putaway C<id> [slot]\n'); return, end
        if strcmpi(rest{1},'all'), F.queuePutaway(0,''); else
            sl = ''; if numel(rest)>=2, sl = rest{2}; end
            F.queuePutaway(str2double(regexprep(rest{1},'^[cC]','')),sl);
        end
    case 'map'
        if ~isempty(rest) && strcmpi(rest{1},'off'), F.map2D = false; fleetMap2D(F,'close'); fprintf('[P3] 2-D map closed\n'); else, F.map2D = true; fleetMap2D(F,'draw'); fprintf('[P3] 2-D map open\n'); end
    case 'cartons', fprintf('%s',F.cartonList(~isempty(rest) && strcmpi(rest{1},'all')));
    case 'ship', k = 0; if ~isempty(rest), k = str2double(rest{1}); end, F.shipNow(k);
    case 'remove'       % p3 remove C<id> | <slot> | D03     (the box goes outside again)
        if isempty(rest), fprintf('usage: p3 remove C<id>  or  p3 remove <slot>   e.g. p3 remove 4A-L1-P02    p3 remove D03\n'); return, end
        if ~isempty(regexp(rest{1},'^[cC]\d+$','once')), F.removeCarton(str2double(rest{1}(2:end))); else, j = F.boxOf(rest{1}); if j > 0, F.removeCarton(j); end, end
    case 'add'          % p3 add <slot|D..> [C<id>]   |   p3 add station <n> [C<id>]
        if isempty(rest), fprintf('usage: p3 add 4A-L1-P02   |   p3 add D05   |   p3 add station 2      (optional: C<id> = which spare box)\n'); return, end
        j = 0; if numel(rest) >= 2 && ~isempty(regexp(rest{end},'^[cC]\d+$','once')), j = str2double(rest{end}(2:end)); end
        if strcmpi(rest{1},'station'), sn = 1; if numel(rest) >= 2 && isempty(regexp(rest{2},'^[cC]\d+$','once')), sn = str2double(rest{2}); end, F.addBoxStation(sn,j);
        else, F.addBoxAt(rest{1},j); end
    case 'clear'        % p3 clear shelves | stands | stations [n] | all
        if isempty(rest) || ~any(strcmpi(rest{1},{'shelves','stands','stations','station','all'})), fprintf('usage: p3 clear shelves | stands | stations [n] | all      (boxes go outside; boxes with a job stay)\n'); return, end
        w = lower(rest{1}); if strcmp(w,'station'), w = 'stations'; end
        n = 0; if numel(rest) >= 2, n = str2double(rest{2}); if ~isfinite(n), n = 0; end, end
        F.clearBoxes(w,n);
    case 'reset'        % p3 reset = all boxes back to the START layout (robots must be free)
        F.resetBoxes();
    case 'mode'
        if isempty(rest) || ~any(strcmpi(rest{1},{'manual','batch','auto','1','2','3'}))
            fprintf('usage: p3 mode manual|batch|auto   (1|2|3).  Now: %s\n',F.mode); return, end
        m = lower(rest{1}); m = p3tern(strcmp(m,'1'),'manual',p3tern(strcmp(m,'2'),'batch',p3tern(strcmp(m,'3'),'auto',m))); F.setMode(m);
    case 'go'
        k = 0; if ~isempty(rest), k = F.robotIndex(rest{1}); end
        F.go(k);
    case 'queue', fprintf('%s',F.queueText());
    case {'cam','camera'}, p3('watch',rest{:});
    case 'rec'
        if isempty(rest) || strcmpi(rest{1},'on'), p3('watch','record'); else, p3('watch','stoprecord'); end
    case 'order'
        [item,st,~] = localParseItem(rest); if isnan(item), return, end
        F.addOrder(item,st,false);
    case {'auto'}
        if isempty(rest) || strcmpi(rest{1},'on')
            per = []; arr = []; st = [];
            for i = 2:numel(rest)-1
                v = str2double(rest{i+1});
                if strcmpi(rest{i},'order'), per = v; elseif strcmpi(rest{i},'arrive'), arr = v; elseif strcmpi(rest{i},'station'), st = v; end
            end
            F.setAuto(true,per,arr,st);
        elseif strcmpi(rest{1},'off'), F.setAuto(false);
        elseif ~isempty(regexp(rest{1},'^(\d+[ABab]-[Ll]\d+-[Pp]\d+|[cC]\d+|[dD]\d+)$','once')), p3('urgent',rest{:});
        else, fprintf('usage: p3 auto on [order <s>] [arrive <s>] [station <n>]   |   p3 auto off\n');
        end
    case {'shelves','snapshot','map'}
        tag = ''; if ~isempty(rest), tag = rest{1}; end
        f = F.snapshot(tag); fprintf('%s\n',F.slotMap()); fprintf('[P3] saved: %s\n',f);
    case 'status', F.statusText();
    case 'watch'
        if isempty(rest), fprintf('usage: p3 cam <robot>|off   (p3 cam pink off also closes)\n'); return, end
        w = lower(rest{1}); if numel(rest) >= 2 && strcmpi(rest{end},'off'), w = 'off'; end
        if strcmp(w,'off'), F.setWatch(0);
        elseif strcmp(w,'record'), F.recordVideo = true; fprintf('[P3] camera video recording on (file written when the simulation stops)\n');
        elseif strcmp(w,'stoprecord'), F.recordVideo = false;
        else, k = F.robotIndex(w); if k==0, fprintf('unknown robot %s\n',w); else, F.setWatch(k); end
        end
    case 'reply'
        if isempty(rest), fprintf('usage: p3 reply <slot id>|auto\n'); else, F.reply(rest{1}); end
    case 'slot'
        if numel(rest)<2, fprintf('usage: p3 slot <id> free|blocked\n'); else, F.setSlot(rest{1},lower(rest{2})); end
    case 'verbose'
        if ~isempty(rest), F.verbose = str2double(rest{1}); fprintf('[P3] verbosity %d\n',F.verbose); end
    case 'answer'
        if ~isempty(rest) && any(strcmpi(rest{1},{'ask','auto'})), F.answerMode = lower(rest{1}); fprintf('[P3] if a target slot is taken the robot will: %s\n',F.answerMode); end
    case 'metrics', F.metricsWrite(false); fprintf('[P3] metrics written to %s\n',F.outDir);
    case 'end', F.finishAll();
    case 'stop', F.stopNow();
    case 'robots'      % p3 robots red blue <cmd ...>
        names = {}; i = 1;
        if ~isempty(rest) && strcmpi(rest{1},'all'), names = {F.robots.name}; i = 2; end
        while i<=numel(rest) && F.robotIndex(rest{i})>0, names{end+1} = rest{i}; i = i+1; end %#ok<AGROW>
        if isempty(names) || i>numel(rest), fprintf('usage: p3 robots red blue <command>\n'); return, end
        for q = 1:numel(names), localRobot(F,F.robotIndex(names{q}),rest(i:end)); end
    case 'all'
        if isempty(rest), fprintf('usage: p3 all pause|resume|stop|status|home\n'); return, end
        for k = 1:F.N, localRobot(F,k,rest); end
    otherwise
        k = F.robotIndex(cmd);
        if k>0, localRobot(F,k,rest); else, fprintf('unknown command or robot "%s". Try  p3 help\n',cmd); end
end
end

function localRobot(F,k,r)
if isempty(r), F.robotCmd(k,'status'); return, end
c = lower(r{1}); r = r(2:end);
switch c
    case {'pause','resume','stop','status'}, F.robotCmd(k,c);
    case 'home', F.robotCmd(k,'resume'); fprintf('[P3] %s: goes home when it has no task (it does so automatically)\n',F.robots(k).name);
    case 'retrieve'
        if isempty(r), fprintf('usage: p3 %s retrieve <slot id or #> [station <n>]\n',F.robots(k).name); return, end
        slot = r{1}; st = 0;
        for i = 2:numel(r)-1, if strcmpi(r{i},'station'), st = str2double(r{i+1}); end, end
        F.manualTask(k,'retrieve',0,slot,st);
    case 'putaway'
        if isempty(r), fprintf('usage: p3 %s putaway C<id> [slot id]\n',F.robots(k).name); return, end
        item = str2double(regexprep(r{1},'^[cC]','')); slot = ''; if numel(r)>=2, slot = r{2}; end
        F.manualTask(k,'putaway',item,slot,0);
    case 'watch', F.setWatch(k);
    otherwise, fprintf('unknown robot command "%s" (pause resume stop status retrieve putaway watch)\n',c);
end
end

function [item,st,ok] = localParseItem(r)
item = nan; st = 0; ok = false;
if isempty(r), fprintf('usage: p3 order any|C<id> [station <n>|any]\n'); return, end
if strcmpi(r{1},'any'), item = 0; else
    t = regexp(r{1},'^[cC]?(\d+)$','tokens','once'); if isempty(t), fprintf('item must be "any" or C<id>\n'); return, end
    item = str2double(t{1});
end
for i = 2:numel(r)-1, if strcmpi(r{i},'station') && ~strcmpi(r{i+1},'any'), st = str2double(r{i+1}); end, end
ok = true;
end

function localHelp()
fprintf(['\n==== P3 HELP ====\n' ...
 'phase2                  = start simulation (3-D window + 2-D map), then pick a mode\n' ...
 'p3 mode auto            = robots work alone: 18 critical tasks (3 per robot), then go home\n' ...
 'p3 mode manual          = you give jobs, they start at once, robots work in parallel\n' ...
 'p3 mode batch           = load jobs per robot, nothing moves until  p3 go\n' ...
 'p3 move <slot|C#> station [n]   = box -> packing station      e.g. p3 move 3A-L2-P04 station 1\n' ...
 'p3 move D03 <slot>      = box on stand -> shelf slot          e.g. p3 move D03 1A-L1-P02\n' ...
 'p3 <robot> retrieve <slot> station [n]  = one robot, one job  (also: putaway C# <slot>)\n' ...
 'p3 urgent <slot|C#> [station n] = emergency job, nearest free robot takes it\n' ...
 'p3 go [robot]           = start the batch jobs\n' ...
 'p3 wms                  = boxes on shelves / stands / stations + free slots\n' ...
 'p3 dock add <n>         = n new boxes arrive on stands D01..D14\n' ...
 'p3 add <slot|D05>       = a box appears in that free slot  (p3 add station 2 = on station 2)\n' ...
 'p3 remove <slot|C#>     = take that box out       p3 clear shelves|stands|stations [n]|all = remove many\n' ...
 'p3 reset                = all boxes back to the START layout (robots must be free)\n' ...
 'p3 status | queue | cartons | metrics = state, job list, box list, numbers\n' ...
 'p3 map on|off | cam <robot>|off | rec on|off = 2-D map, camera view, recording\n' ...
 'p3 <robot>|all pause|resume|stop = robots: blue\n' ...
 'p3 end                  = finish, go home, write files        p3 stop = stop at once\n' ...
 'phase2 layout 6 | phase2 fill max = 6 shelves | all slots filled (strong PC)\n' ...
 'slots: <shelf><A|B>-L<level>-P<nn>  e.g. 3B-L2-P05\n' ...
 '=================\n\n']);
end
