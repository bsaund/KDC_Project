% quadruped gait testing
addpath('C:\Users\medgroup01\Documents\Julian\snake monster\quadruped gait\KDC_tools');
addpath('C:\Users\medgroup01\Documents\Julian\snake monster\quadruped gait\hebi');
addpath('C:\Users\medgroup01\Documents\Julian\snake monster\quadruped gait\matlab_SEA-3-21\plottingTools');


% the position of the chassis center is idendentity for now
% g_body = eye(4);

th = zeros(3,6); % joint angles: each column is a leg (proximal to distal)
params = SMPhysicalParameters();
SMData = makeSMData(params);
[gt, gl2, gl3, gl4] = FK(SMData, th);

% points that we want to get the feet to:
% xd = zeros(3,6); % [x; y; z] for each leg

xd_base = getXYZ(gt);
xd = xd_base;

% move all feet in towards chassis
xd(1,[1,3,5]) = xd(1,[1,3,5])-.1;
xd(1,[2,4,6]) = xd(1,[2,4,6])+.1;

% front legs
xd(1,1) = params.W/2 + params.l(1);
xd(1,2) = -params.W/2- params.l(1);
xd(3,[1,2]) = .2;
xd(2,[1,2]) = params.L/2+.1;
% middle legs
% xd(2,[3,4]) = .1;
% back legs
% xd(2,[5,6]) = -.1;
% th = IK(SMData, xd);

close all;
 plt = SnakeMonsterPlotter(); hold on;
 scatterL2 = scatter3(0,0,0, 'k','sizedata',400); 
 scatterL3 = scatter3(0,0,0, 'k','sizedata',400);
 scatterL4 = scatter3(0,0,0, 'k','sizedata',400);
% %  Sr = .025;
% %  [unit_sphx,unit_sphy,unit_sphz] = sphere(10);
% %  surf(unit_sphx*Sr, unit_sphy*Sr, unit_sphz*Sr, zeros(size(unit_sphz)));
 
 z0 = ones(1,6)*-.1;
 y0 = [0 0 .1 .1 -.1 -.1]*1.0;
 a = .075; % step length = 2*a
 b = .05; % step height = b
%  a =0;
%  b=0;

for t = linspace(0,2*pi,100)
% leg 6
[xd(2,6),xd(3,6)] = ellipticalGait(a,b,y0(6),z0(6), t);
% leg 4
[xd(2,4),xd(3,4)] = ellipticalGait(a,b,y0(4),z0(4), t+pi/2);
% leg 5
[xd(2,5),xd(3,5)] = ellipticalGait(a,b,y0(5),z0(5), t+2*pi/2);
% leg 3
[xd(2,3),xd(3,3)] = ellipticalGait(a,b,y0(3),z0(3), t+3*pi/2);
% the x component stays the same so it goes straight
scatter3(xd(1,:), xd(2,:), xd(3,:), 'g','filled');

% do the IK for the new position
th = IK(params, SMData, xd);
% th(:,[2 4 6]) = 0; % for now just look at one side
% th(:,[1 3 5]) = 0; % for now just look at one side
 plt.plot(reshape(th,[1,18])); hold on;

  [gtNew, gl2, gl3, gl4] = FK(SMData, th);
  xyzFeet= getXYZ(gtNew);
 scatter3(xyzFeet(1,:), xyzFeet(2,:), xyzFeet(3,:), 'r');
  xyzL2= getXYZ(gl2);
 set(scatterL2, 'xdata', xyzL2(1,:), 'ydata', xyzL2(2,:), 'zdata', xyzL2(3,:));
  xyzL3= getXYZ(gl3);
 set(scatterL3, 'xdata', xyzL3(1,:), 'ydata', xyzL3(2,:), 'zdata', xyzL3(3,:));
  xyzL4= getXYZ(gl4);
 set(scatterL4, 'xdata', xyzL4(1,:), 'ydata', xyzL4(2,:), 'zdata', xyzL4(3,:));
 % calculate the center of mass
 COM = calcCOM(params, xyzL2, xyzL3, xyzL4);
 
 pause(0.05); 

end



%% plotting
%  plt.plot(zeros(1,18));



