function [cen,cnt] = p3Cluster(P,eps,minPts)
%P3CLUSTER  Single-linkage clustering of 2-D points (distance < eps), clusters with fewer than minPts points are dropped.  cen = centroids, cnt = sizes.
cen = zeros(0,2); cnt = zeros(0,1); n = size(P,1); if n == 0, return, end
lab = zeros(n,1); nl = 0;
for i = 1:n
    if lab(i), continue, end
    nl = nl + 1; lab(i) = nl; st = i;
    while ~isempty(st)
        j = st(end); st(end) = [];
        d = hypot(P(:,1)-P(j,1),P(:,2)-P(j,2)); nb = find(d < eps & lab == 0);
        lab(nb) = nl; st = [st; nb(:)]; %#ok<AGROW>
    end
end
for c = 1:nl
    m = lab == c; if nnz(m) >= minPts, cen(end+1,:) = mean(P(m,:),1); cnt(end+1,1) = nnz(m); end %#ok<AGROW>
end
end
