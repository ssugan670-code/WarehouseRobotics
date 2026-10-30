function varargout = pink(varargin)
%PINK  Command the pink robot:  pink putaway C5 1A-L1-P03 | pink retrieve 3A-L2-P04 station 1 | pink pause|resume|stop|status|home|watch
% (If called with a number / output argument it behaves like MATLAB's colormap pink.)
if nargout>0 || (nargin>0 && isnumeric(varargin{1}))
    w = which('-all','pink'); here = mfilename('fullpath');
    for i = 1:numel(w)
        if ~strncmp(w{i},here,numel(here)), [varargout{1:max(nargout,1)}] = feval(w{i},varargin{:}); return, end
    end
end
p3('pink',varargin{:});
end
