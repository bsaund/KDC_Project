% main program for stance optimization
% for some number of feet on the ground, find the stance which will put the
% COM farthest from the edges of the polygon subject to joint limits 

close all; clc;
addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));

global kin thExtra nLegs stanceLegs extraLegs masses params zd

% stuff to set by hand:
stanceLegs = [  3 4 5 6]; % array of legs that are in the air, stretched out far
zd = .1; % the minimum body height

% sorting and making leg arrays
stanceLegs =sort(stanceLegs); % in case they werent in order
nLegs = 6;
nStanceLegs = length(stanceLegs);
extraLegs = [];
 % remove the non-walking legs from the step order
 for i = 1:6
    if sum(i == stanceLegs)<1
         extraLegs = [extraLegs i];
    end
 end


th0 = zeros(1, 3*nStanceLegs); % joint angles: for each leg, proximal to distal

params = SMPhysicalParameters();
SMData = makeSMData(params);
kin = SnakeMonsterKinematics; % does all the kinamatics
masses = kin.getLegMasses();

%% find the "reach out" joint angles for the extra legs
xyz0 = kin.getLegPositions(zeros(1,3*nLegs)); % zero position of all the legs
xyz = xyz0;
 % set the non-walking legs to be up in the air
 for i = 1:(6-nStanceLegs) 
     xyz(3,extraLegs(i)) =xyz0(3,extraLegs(i)) + .2;
     xyz(2,extraLegs(i)) =xyz0(2,extraLegs(i))*5;
     xyz(1,extraLegs(i)) =xyz0(1,extraLegs(i))*(.097-xyz0(2,extraLegs(i)))/.097;
%      xyz(2,extraLegs(i)) =5*xyz0(2,extraLegs(i))*sign(xyz0(2,extraLegs(i)));
%      xyz(1,extraLegs(i)) =.1*sign(xyz0(1,extraLegs(i)));
 end
 thIK = kin.getIK(xyz);
 thMat = reshape(thIK,[3,nLegs]);
 thExtra = thMat(:,extraLegs);
 
%% set up the optimization 
 % constraints
 UB = pi/2*ones(1, 3*nStanceLegs); % th max upper bound
 LB= -pi/2*ones(1, 3*nStanceLegs); % th min lower bound
A = []; % linear ineq constraints
B = [];
Aeq = []; % linear eq constraints
Beq = [];
options = optimset('TolCon', 1e-3, 'TolFun', 1e-3);% for speed


 %% the big optimization
 thOpt = fmincon(@cost3,th0,A,B,Aeq,Beq,LB,UB,@nonlincon1,options);
 
 %% plot the results
thMat = reshape(thOpt, [3,nStanceLegs]);
thFull = zeros(3,nLegs);
thFull(:,extraLegs) = thExtra;
thFull(:, stanceLegs) = thMat;

 
 
 xyz = kin.getLegPositions(thFull);
 xyzContact = xyz(:,stanceLegs);
 z0 = mean(xyzContact(3,:));
   % plot the support polygon
 K = convhull(xyzContact(1:2,:).');
 xOrdered = xyzContact(1:3,K.');
%  % check if COM in the support poluygon
 CoMs = kin.getCenterOfMasses(thFull);
bodyCoM = ...
 [ reshape(CoMs, [3,30]) zeros(3,1)]*...
 [reshape(masses, [30,1]); params.robotMass] / sum([reshape(masses, [30,1]); params.robotMass]);
 inSupport = inpolygon(bodyCoM(1,end),bodyCoM(2,end),xOrdered(1,:),xOrdered(2,:));
 % find the centroid of the support polygon
 xCentroid = mean(xOrdered(:,1:end-1),2);
 
 % plotting
  plt = SnakeMonsterPlotter(); 
  plt.plot(thFull); hold on;
  set(gcf, 'position', [10 100 800 700]);
%  scatterCoM= scatter3(legCoM(1,:), legCoM(2,:), legCoM(3,:), 'k');
projectedCOM = scatter3(0,0,0,'k', 'filled');
supportLines = plot3(0,0,0,'k');
scatterCentroid = scatter3(0,0,0,'b');
  set(supportLines, 'xdata', xOrdered(1,:), 'ydata',xOrdered(2,:), ...
     'zdata', xOrdered(3,:));
  set(projectedCOM, 'xdata', bodyCoM(1), 'ydata',bodyCoM(2), 'zdata', z0,...
     'markerFaceColor', [1 0 0]*~inSupport);
  set(scatterCentroid, 'xdata', xCentroid(1), 'ydata',xCentroid(2), 'zdata',z0);

 