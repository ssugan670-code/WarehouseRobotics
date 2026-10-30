classdef Phase3FleetBlock < matlab.System
%PHASE3FLEETBLOCK  Gives the Simulink model the fleet manager's carton commands (park / arrival teleports) and the stop flag.
%   Outputs (Unreal frame): cmd 1x3M [x -y z per carton], rotc 1x3M [0 0 -yaw per carton], act 1xM (fleet drives this carton), stop.
properties (Nontunable)
    NumCartons = 12
    SampleTime = 0.02
end
properties (Access=private)
    F
end
methods (Access=protected)
    function setupImpl(obj)
        obj.F = evalin('base','Phase3Fleet');
    end
    function [cmd,rotc,act,stop] = stepImpl(obj)
        M4 = obj.F.cartonCmd();
        cmd = reshape([M4(:,1) -M4(:,2) M4(:,3)].',1,[]);
        rotc = reshape([zeros(size(M4,1),1) zeros(size(M4,1),1) -M4(:,5)].',1,[]);
        act = (M4(:,4) > 0.5).';
        stop = logical(obj.F.stopFlag);
    end
    function n = getNumInputsImpl(~), n = 0; end
    function n = getNumOutputsImpl(~), n = 4; end
    function [a,r,b,c] = getOutputSizeImpl(obj), a = [1 3*obj.NumCartons]; r = a; b = [1 obj.NumCartons]; c = [1 1]; end
    function [a,r,b,c] = getOutputDataTypeImpl(~), a = 'double'; r = 'double'; b = 'boolean'; c = 'boolean'; end
    function [a,r,b,c] = isOutputComplexImpl(~), a = false; r = false; b = false; c = false; end
    function [a,r,b,c] = isOutputFixedSizeImpl(~), a = true; r = true; b = true; c = true; end
    function sts = getSampleTimeImpl(obj), sts = createSampleTime(obj,'Type','Discrete','SampleTime',obj.SampleTime); end
end
end
