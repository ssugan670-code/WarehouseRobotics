function L = pathLength(w)
%PATHLENGTH  Length of a polyline (N x 2).
if size(w,1) < 2, L = 0; else, L = sum(sqrt(sum(diff(w,1,1).^2,2))); end
end
