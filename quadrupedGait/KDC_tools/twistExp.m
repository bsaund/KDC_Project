function out = twistExp(xi, theta)
% exp(xihat*theta) from xi(vector) and theta
w = xi(4:6);
v = xi(1:3);
ewth = rodrigues(w,theta);
out = [ewth,  (eye(3)-ewth)*cross(w,v) + w*w.'*v*theta; [0 0 0 1]];
end