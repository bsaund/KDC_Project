% quadruped gait testing with HebiKinematics
addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));

th = zeros(3,6); % joint angles: each column is a leg (proximal to distal)
params = SMPhysicalParameters();
SMData = makeSMData(params);
inch2m = 0.0254;
odd = [1,3,5]; even= [2 4 6];

% initialize hebi object
% chassis_bend= 1.5; % degrees
chassis_bend= 0; % degrees
% transformation from the origin to the leg frame
baseTransform = zeros(4,4,6);
baseTransform(:,:,1) = [R_y(deg2rad(90-chassis_bend))*R_z(deg2rad(90))  [params.W/2; params.L/2; 0]; [0 0 0 1]];
baseTransform(:,:,2) = [R_y(deg2rad(-90+chassis_bend))*R_z(deg2rad(90)) [-params.W/2; params.L/2; 0]; [0 0 0 1]];
baseTransform(:,:,3) = [R_y(deg2rad(90-chassis_bend))*R_z(deg2rad(90))  [params.W/2; 0; 0]; [0 0 0 1]];
baseTransform(:,:,4) = [R_y(deg2rad(-90+chassis_bend))*R_z(deg2rad(90)) [-params.W/2; 0; 0]; [0 0 0 1]];
baseTransform(:,:,5) = [R_y(deg2rad(90-chassis_bend))*R_z(deg2rad(90))  [params.W/2; -params.L/2; 0]; [0 0 0 1]];
baseTransform(:,:,6) = [R_y(deg2rad(-90+chassis_bend))*R_z(deg2rad(90)) [-params.W/2; -params.L/2; 0]; [0 0 0 1]];
baseXYZ = getXYZ(baseTransform);

legKin = cell(1,6);
frames = cell(1,6);
lastLinkTwist = [pi 0 pi 0 pi 0]-pi/2;
legCoM = zeros(3,7); % last entry is the chassis
masses = zeros(7,1); % last entry is the chassis
masses(end) = params.robotMass;
effectors= zeros(3,6); % xyz of effectors
for i = 1:6
legKin{i} = HebiKinematics();
legKin{i}.setBaseFrame(baseTransform(:,:,i));
legKin{i}.addBody('FieldableElbowJoint');
legKin{i}.addBody('FieldableElbowJoint');
legKin{i}.addBody('FieldableStraightLink', ...
    'ext', params.staticLength1, 'twist', pi/2);
legKin{i}.addBody('FieldableElbowJoint');
legKin{i}.addBody('FieldableElbowLink', ...
    'ext1', params.staticLength2, 'twist1', lastLinkTwist(i), ...
    'ext2', params.staticLength3, 'twist2', 0);
legKin{i}.addBody('FieldableStraightLink', ...
    'ext', params.footLength, 'twist', 0);

 frames{i} = legKin{i}.getForwardKinematics('output', th(:,i));
  CoMs = legKin{i}.getForwardKinematics('CoM', th(:,i));
  legCoM(:,i) = getXYZ(CoMs)*legKin{i}.getBodyMasses;
  masses(i) = sum(legKin{i}.getBodyMasses);
effectors(:,i) = frames{i}(1:3,4,end);
end
% effectors

th_IK = zeros(3,6);
xd = effectors;
% move all feet in towards chassis
xd(1,odd) = xd(1,odd)-.1;
xd(1,even) = xd(1,even)+.1;
xd(3,:) = xd(3,:)+.05;
% % xyz(1,odd) = 10; reach far test
% % xyz(1,even) = -10;
% front legs
xd(1,1) = params.W/2 + params.l(1);
xd(1,2) = -params.W/2- params.l(1);
xd(3,[1,2]) = .3;
xd(2,[1,2]) = params.L/2+.1;

close all;
effectors2= zeros(3,6); % xyz of effectors
plt = SnakeMonsterPlotter(); hold on; 
scatterCoM= scatter3(legCoM(1,:), legCoM(2,:), legCoM(3,:), 'k', 'filled');

a = .075; % step length = 2*a
 b = .05; % step height = b
 z0 = ones(1,6)*-.15;
 y0 = [0 0 .1 .1 -.1-a -.1-a];

 
for t = linspace(0,2*pi,100)
 
 % leg 6
[xd(2,6),xd(3,6)] = ellipticalGait(a,b,y0(6),z0(6), t);
% leg 4
[xd(2,4),xd(3,4)] = ellipticalGait(a,b,y0(4),z0(4), t+pi/2);
% leg 5
[xd(2,5),xd(3,5)] = ellipticalGait(a,b,y0(5),z0(5), t+2*pi/2);
% leg 3
[xd(2,3),xd(3,3)] = ellipticalGait(a,b,y0(3),z0(3), t+3*pi/2);
 
 for i = 1:6
  th_IK(:,i) = legKin{i}.getInverseKinematics('xyz', xd(:,i));
  frames{i} = legKin{i}.getForwardKinematics('output', th_IK(:,i));
  effectors2(:,i) = frames{i}(1:3,4,end);
  CoMs = legKin{i}.getForwardKinematics('CoM', th_IK(:,i));
  legCoM(:,i) = getXYZ(CoMs)*legKin{i}.getBodyMasses/sum(legKin{i}.getBodyMasses);
 end

 legCoM(:,end) = legCoM*masses/sum(masses);
 
 plt.plot(reshape(th_IK,[1,18]));
 scatter3(effectors2(1,:), effectors2(2,:), effectors2(3,:), 'r');
 scatter3(xd(1,:), xd(2,:), xd(3,:), 'g', 'filled');
 set(scatterCoM, 'xdata', legCoM(1,:), 'ydata',legCoM(2,:), 'zdata',legCoM(3,:));
end
 