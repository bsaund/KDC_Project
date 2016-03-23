function out = rodrigues(w,theta)
%Rodrigues' formula
% w = omega = [3x1] angular velocity axis
% theta = angle about axis
% https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula

out = eye(3) + skew(w)*sin(theta) + skew(w)^2*(1-cos(theta));


end