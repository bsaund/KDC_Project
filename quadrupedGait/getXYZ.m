function xyz = getXYZ(g)
xyz = reshape(g(1:3,4,:),[3,size(g,3)]);

end