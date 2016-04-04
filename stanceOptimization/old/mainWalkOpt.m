% mainWalkOpt
% main program for walking parameters optimization
% assume a given gait and walking legs,
% find the gait parameters (starting foot positions from body center, step length) 
% which maximize stability for a few time points in the gait cycle
% where stability is COM farthest from the edges of the polygon subject to joint limits 


close all; clc;
addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));

global kin xyzExtra nLegs stanceLegs extraLegs masses params zd
% stuff to set by hand:
stanceLegs = [ 2  3 4 5 6]; % array of legs that are in the air, stretched out far
% zd = .15; % the minimum body height
z0 = -.15; % the max body height

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
 end
 xyzExtra = xyz(:,extraLegs);

 
 % set the base of the stance legs to be at zd
%  xyz(3,stanceLegs) = -zd;
 
%  thIK = kin.getIK(xyz);
%  thMat = reshape(thIK,[3,nLegs]);
%  thExtra = thMat(:,extraLegs);


% xyzStep0 is the 3xnStepLegs positions of the stepping feet at t=0 in
% the gait cycle
% Here I optimize for the best xyStep0 (which will be 1x2*nStepLegs)
% assuming a fixed zStep0
xyStep0 = reshape(xyz0(1:2,stanceLegs), [1, 2*nStanceLegs]); % initial value: all legs 
% form of: [x1 y1 x2 y2 ...]
state0 = [xyStep0 z0];

%% set up the optimization 
 % constraints
 legBaseXY = [params.W/2, -params.W/2, params.W/2, -params.W/2, params.W/2, -params.W/2;...
               params.L/2, params.L/2,      0 ,         0,       -params.L/2, -params.L/2];
stanceLegBaseXY = legBaseXY(:, stanceLegs);
 UB = reshape(stanceLegBaseXY, [1, 2*nStanceLegs]) + .3*ones(1, 2*nStanceLegs); %  max upper bound
 LB=  reshape(stanceLegBaseXY, [1, 2*nStanceLegs]) - .3*ones(1, 2*nStanceLegs); %  min lower bound
 UB = [UB 0];
 LB = [LB z0];
A = []; % linear ineq constraints
B = [];
Aeq = []; % linear eq constraints
Beq = [];
options = optimset('TolCon', 1e-3, 'TolFun', 1e-3);% for speed

%  % the big optimization
%  stateOpt = fmincon(@costWalk1,state0,A,B,Aeq,Beq,LB,UB,@nonlinconWalk1,options);
 stateOpt = simulannealbnd(@costWalk1,state0,LB,UB);
%  xyStepOpt = xyStep0;
 
 %% plot the results
 xyStepOpt = stateOpt(1:end-1);
 xyz(1:2,stanceLegs) = reshape(xyStepOpt, [2,nStanceLegs]); 
 xyz(3,stanceLegs) = stateOpt(end);
  thIK = kin.getIK(xyz);
  xyzFK = kin.getLegPositions(thIK);
   
% get the COM:
 CoMs = kin.getCenterOfMasses(thIK);
bodyCoM = ...
 [ reshape(CoMs, [3,30]) zeros(3,1)]*...
 [reshape(masses, [30,1]); params.robotMass] / sum([reshape(masses, [30,1]); params.robotMass]);
bodyCoM = [bodyCoM(1:2,:); stateOpt(end)]; % project it down

% %  the support polygon
 xyzContact = xyz(:,stanceLegs);
 K = convhull(xyzContact(1:2,:).');
 xOrdered = xyzContact(:,K.');
 xCentroid = mean(xOrdered(:,1:end-1),2);
  
  plt = SnakeMonsterPlotter(); 
  plt.plot(thIK); hold on;
    set(gcf, 'position', [10 100 800 700]);
projectedCOM = scatter3(0,0,0,'k', 'filled');
supportLines = plot3(0,0,0,'k');
scatterCentroid = scatter3(0,0,0,'b');
  set(supportLines, 'xdata', xOrdered(1,:), 'ydata',xOrdered(2,:), ...
     'zdata', xOrdered(3,:));
  set(projectedCOM, 'xdata', bodyCoM(1), 'ydata',bodyCoM(2), 'zdata', bodyCoM(3),...
     'markerFaceColor', [1 0 0]*~inSupport);
  set(scatterCentroid, 'xdata', xCentroid(1), 'ydata',xCentroid(2), 'zdata',xCentroid(3));
