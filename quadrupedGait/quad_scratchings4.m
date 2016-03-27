% quadruped gait testing with HebiKinematics
% With simulation and sendCommands option.

simulation = 1; %0 to turn off simulation
sendCommands = 0; % 1 to turn on commands to real robot

addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));
% addpath(genpath('C:\Users\Julian\Box Sync\CMU sem 1+2\snakeMonster\KDC_project'));

th0 = zeros(3,6); % joint angles: each column is a leg (proximal to distal)
params = SMPhysicalParameters();
SMData = makeSMData(params);
inch2m = 0.0254;
odd = [1,3,5]; even= [2 4 6];
x =1; y =2; z = 3;

% initialize hebi object
chassis_bend= 1.5; % degrees
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
    'ext', params.staticLength1-.0122, 'twist', pi/2);
legKin{i}.addBody('FieldableElbowJoint');
legKin{i}.addBody('FieldableElbowLink', ...
    'ext1', params.staticLength2 - 0.0360, 'twist1', lastLinkTwist(i), ...
    'ext2', params.staticLength3 - 0.0336, 'twist2', 0);
legKin{i}.addBody('FieldableStraightLink', ...
    'ext', params.footLength-0.0122, 'twist', 0);

 frames{i} = legKin{i}.getForwardKinematics('output', th0(:,i));
  CoMs = legKin{i}.getForwardKinematics('CoM', th0(:,i));
  legCoM(:,i) = getXYZ(CoMs)*legKin{i}.getBodyMasses;
  masses(i) = sum(legKin{i}.getBodyMasses);
effectors(:,i) = frames{i}(1:3,4,end);
end
% effectors

th_IK = zeros(3,6);
xyz0 = effectors;
xyz = effectors;

% move all feet in towards chassis
xyz(1,odd) = xyz(1,odd)-.1;
xyz(1,even) = xyz(1,even)+.1;
xyz(3,:) = xyz(3,:)+.05;


if simulation
    close all;
plt = SnakeMonsterPlotter(); hold on; 
scatterCoM= scatter3(legCoM(1,:), legCoM(2,:), legCoM(3,:), 'k');
projectedCOM = scatter3(0,0,0,'k', 'filled');
supportLines = plot3(0,0,0,'k');
scatterCentroid = scatter3(0,0,0,'b');
end

if sendCommands
    setupSnakeMonsterGroups;
end
% command structure for if sending commands
      cmd = CommandStruct();
        cmd.position = [];
        cmd.velocity = [];
        cmd.torque = [];
        
        
a = params.L/3; % step length = 2*a
 b = .05; % step height = b
 z0 = ones(1,6)*-.15;

 y0 = zeros(1,6);
 y0(odd)= [0 2*a a];
 y0(even) = [a 0 -a];
 
   swingLegs = zeros(1,6); % 1 indicates leg is in the air 
 walkingLegs = [ 2 3 4 5 6]; % specify which legs will be used for walking
 nWalkingLegs = length(walkingLegs);
 fractionStep = 1/nWalkingLegs;
 stepOrderBase = [6 1 5 2 3 4];
 stepOrder = [];
 extraLegs = [];
 % remove the non-walking legs from the step order
 for i = 1:6
    if sum(walkingLegs == stepOrderBase(i))<1
         extraLegs = [extraLegs stepOrderBase(i)];
    else
        stepOrder = [stepOrder stepOrderBase(i)];
    end
 end
 swingLegs(extraLegs) = 2;
 
 % set the non-walking legs to be up in the air
 for i = 1:(6-nWalkingLegs) 
     xyz(3,extraLegs(i)) =.5;
     xyz(2,extraLegs(i)) =.2*sign(xyz0(2,extraLegs(i)));
     xyz(1,extraLegs(i)) =.1*sign(xyz0(1,extraLegs(i)));
 end

  % get initial IK
  for i = 1:6
  th_IK(:,i) = legKin{i}.getInverseKinematics('xyz', xyz(:,i));
  frames{i} = legKin{i}.getForwardKinematics('output', th_IK(:,i));
  effectors(:,i) = frames{i}(1:3,4,end);
  CoMs = legKin{i}.getForwardKinematics('CoM', th_IK(:,i));
  legCoM(:,i) = getXYZ(CoMs)*legKin{i}.getBodyMasses/sum(legKin{i}.getBodyMasses);
  end    

  nCycles = 2;
 
 for t = linspace(0,2*pi*nCycles,100*nCycles)
%  % find the jacobian
%  legKin{6}.getJacobian('EndEffector', th(:,i))
%  % find the gravity compensation torques
%    legKin{6}.getGravCompTorques(th(:,i),gravity)
    
    % find the point on the eliptical path
    for i = 1:nWalkingLegs
        leg = stepOrder(i);
        t_leg = t+2*pi/nWalkingLegs*(i-1);
        [xyz(2,leg),xyz(3,leg)] = ellipticalGait(a,b,y0(leg),z0(leg), fractionStep, t_leg);
        if (mod(t_leg, 2*pi)<2*pi*fractionStep)&&(mod(t_leg, 2*pi)>0)
            swingLegs(leg) = 1;
        else
            swingLegs(leg) = 0;
        end
    end
    
 
 for i = 1:6
  th_IK(:,i) = legKin{i}.getInverseKinematics('xyz', xyz(:,i));
   frames{i} = legKin{i}.getForwardKinematics('output', th_IK(:,i));
  effectors2(:,i) = frames{i}(1:3,4,end);
  CoMs = legKin{i}.getForwardKinematics('CoM', th_IK(:,i));
  legCoM(:,i) = getXYZ(CoMs)*legKin{i}.getBodyMasses/sum(legKin{i}.getBodyMasses);
 end

 legCoM(:,end) = legCoM*masses/sum(masses); % full body COM
 
 % plot the support polygon
 xContact = xyz(:,~swingLegs);
 K = convhull(xContact(1:2,:).');
 xOrdered = xContact(1:2,K.');
 % check if COM in the support poluygon
 inSupport = inpolygon(legCoM(1,end),legCoM(2,end),xOrdered(1,:),xOrdered(2,:));
 % find the centroid of the support polygon
 xCentroid = mean(xOrdered(:,1:end-1),2);
 
  if simulation
 plt.plot(reshape(th_IK,[1,18]));
 scatter3(effectors2(1,:), effectors2(2,:), effectors2(3,:), 'r');
 scatter3(xyz(1,:), xyz(2,:), xyz(3,:), [], swingLegs, 'filled');
 set(scatterCoM, 'xdata', legCoM(1,:), 'ydata',legCoM(2,:), 'zdata',legCoM(3,:));
 set(supportLines, 'xdata', xOrdered(1,:), 'ydata',xOrdered(2,:), ...
     'zdata',ones(1,size(xOrdered,2))*mean(z0));
  set(scatterCoM, 'xdata', legCoM(1,:), 'ydata',legCoM(2,:), 'zdata',legCoM(3,:));
 set(projectedCOM, 'xdata', legCoM(1,end), 'ydata',legCoM(2,end), 'zdata',mean(z0),...
     'markerFaceColor', [1 0 0]*~inSupport);
 set(scatterCentroid, 'xdata', xCentroid(1), 'ydata',xCentroid(2), 'zdata',mean(z0));
  end
  
  if sendCommands
      cmd.position = th_IK;
      snakeMonster.set(cmd);
  end
 
end
 