function [src,tried] = p3FindBlock(name)
%P3FINDBLOCK  Path of a Simulink library block by its name (e.g. 'Simulation 3D Lidar') without assuming which library holds it:
%   known candidates first, then every library file in the folder of sim3dcameralib (the Unreal interface libraries live side by side).
%   src = '' when not found; tried = the library names that were searched (for the log).
src = ''; tried = {};
cands = {'sim3dlib','sim3dlidarlib','sim3dsensorlib','drivingsim3d','sim3dradarlib','sim3dcameralib','robotsim3dlib','uavsim3dlib'};
try
    folder = fileparts(which('sim3dcameralib'));
    if ~isempty(folder)
        d = [dir(fullfile(folder,'*.slx')); dir(fullfile(folder,'*.mdl'))];
        for q = 1:numel(d)
            [~,n] = fileparts(d(q).name); if ~isempty(regexpi(n,'3d|lidar|sensor|driving','once')), cands{end+1} = n; end %#ok<AGROW>
        end
    end
catch
end
cands = unique(cands,'stable');
for q = 1:numel(cands)
    try
        load_system(cands{q}); tried{end+1} = cands{q}; %#ok<AGROW>
        f = find_system(cands{q},'SearchDepth',5,'FollowLinks','on','LookUnderMasks','all','MatchFilter',@Simulink.match.allVariants,'Name',name);
        if ~isempty(f), src = f{1}; return, end
    catch
    end
end
end
