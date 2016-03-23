function R = rotmat(n, theta)
% find the 3D rotation matrix for any initial point
% rp = R*r.
% theta must be in radians. 
% n is a column vector- axis of rotation. 

% normalize n:
n = n/sqrt(sum(n.^2));

% make skew matrix S(n)
S = [0 -n(3) n(2) ; n(3) 0 -n(1) ; -n(2) n(1) 0 ];

% rotation matrix
R = n*n' + cos(theta)*(eye(3) - n*n') + sin(theta)*S;

end