function occ = p3GridHit(G,res,off,xy)
%P3GRIDHIT  true where the world points xy (n-by-2) fall into an occupied cell of the static grid G (outside the grid = occupied).
c = floor((xy(:,1) + off(1))*res) + 1; r = floor((xy(:,2) + off(2))*res) + 1;
out = c < 1 | c > size(G,2) | r < 1 | r > size(G,1);
occ = true(size(xy,1),1); ok = ~out; occ(ok) = G(sub2ind(size(G),r(ok),c(ok)));
end
