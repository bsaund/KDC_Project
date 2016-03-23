function out = adjoint3D(g)
% g is 4x4 transformation matrix
R = g(1:3,1:3);
p = g(1:3,4);

out = [R    skew(p)*R;...
       zeros(3,3) R];

end