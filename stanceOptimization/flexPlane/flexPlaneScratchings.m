% flex plane: allow the transformation between the body and the foot plane
% to change, along with the foot positions on the plane.

addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));
close all;

rB_P = [.1 .2 .3].'; % vector of B wrt P, 
 % (body frame wrt plane origin)
RB_P = R_y(pi/8)*R_x(pi/4); % rotation of the frame B wrt P frame
% RB_P = R_x(0); % rotation of the frame B wrt P frame
% (body frame coordinate system as viewed in plane frame)
TB_P = [RB_P rB_P; 0 0 0 1]; % transformation to the body frame from the plane frame
TP_B = inv(TB_P);
% the foot positions in the plane frame. (homogeneous)
xyzp = [-.1 .1 .1 -.1;...
        .2  .2 -.2 -.2;...
        0 0 0 0;...
        1 1 1 1];
% foot positions in the body frame 
xyzb = TP_B*xyzp;
% xyzb = TB_P\xyzp;


figure;
scatter3(xyzb(1,:), xyzb(2,:), xyzb(3,:)); hold on;
quiver3(ones(3,1)*TP_B(1,4), ones(3,1)*TP_B(2,4), ones(3,1)*TP_B(3,4), ...
    TP_B(1:3,1)*.1, TP_B(1:3,2)*.1, TP_B(1:3,3)*.1);
quiver3(zeros(3,1), zeros(3,1), zeros(3,1), ...
    [1;0;0]*.1, [0;1;0]*.1, [0;0;1]*.1);
axis([-1 1 -1 1 -1 1]*.6);

% parameters to optimize:
% thetaX at each step (pitch)
% thetaY at each step (roll)
% (leave yaw = 0 for now)
% rB_P at each step (all three components)
% foot pose x y in plane at t=0

