% mainWalkOpt
% main program for walking parameters optimization
% assume a given walking legs,
% which maximize stability for the stance, tilted, not level,
% where stability is COM farthest from the edges of the polygon subject to joint limits 


close all; clc;
addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));
% addpath(genpath('C:\Users\Julian\Box Sync\CMU sem 1+2\snakeMonster\KDC_project'));

global kin xyzExtra nLegs stanceLegs extraLegs masses params stanceLegBaseXY
% stuff to set by hand:
stanceLegs = [  3 4 5 6]; % array of legs that are in the air, stretched out far


%% sorting and making leg arrays
stanceLegs =sort(stanceLegs); % in case they werent in order
nLegs = 6;
nStanceLegs = length(stanceLegs);


%% initial pose and parameters
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

 %% optimization state and constraints setup
% xyzStep0 is the 3xnStepLegs positions of the stepping feet at t=0 in
% the gait cycle
% Here I optimize for the best xyStep0 (which will be 1x2*nStepLegs)
% assuming a fixed zStep0
xyStep0 = reshape(xyz0(1:2,stanceLegs), [1, 2*nStanceLegs]); % initial value: all legs 
% form of: [x1 y1 x2 y2 ...]

% fitting a plane in stead of a z0... planex*x + planey*y + z + planec = 0
% this is a plane 
planex = 0;
planey = 0;
planec = -.1;
state0 = [xyStep0 planex planey planec];

%% set up the optimization 
 % constraints
 legBaseXY = [params.W/2, -params.W/2, params.W/2, -params.W/2, params.W/2, -params.W/2;...
               params.L/2, params.L/2,      0 ,         0,       -params.L/2, -params.L/2];
stanceLegBaseXY = legBaseXY(:, stanceLegs);
oddInds =find(mod(stanceLegs,2)==1); % the indexes of the odd legs in stanceLegs
evenInds= find(mod(stanceLegs,2)==0);
UBMat = Inf(2,nStanceLegs); LBMat = -UBMat;
UBMat(1,evenInds) = stanceLegBaseXY(1,evenInds) + params.l(1); 
LBMat(1,oddInds) = stanceLegBaseXY(1,oddInds) - params.l(1);
zmin = -.3;
 UB = [reshape(UBMat, [1 2*nStanceLegs]) .25 .25 -zmin];
 LB = [reshape(LBMat, [1 2*nStanceLegs]) -.25 -.25 0];
% A = []; % linear ineq constraints
% B = [];
% linear ineq constraints: the front legs are in front of the back legs, etc.
A = zeros(length(oddInds)-1 + length(evenInds)-1, length(UB));
for i = 1:length(oddInds)-1
A(i,oddInds(i)*2) = -1;
A(i,oddInds(i+1)*2) = 1;
end
for i = 1:length(evenInds)-1
A(i+length(oddInds)-1,evenInds(i)*2) = -1;
A(i+length(oddInds)-1,evenInds(i+1)*2) = 1;
end
B = ones(length(oddInds)-1 + length(evenInds)-1 ,1) * -.01;
Aeq = []; % linear eq constraints
Beq = [];
options = optimset('TolCon', 1e-3, 'TolFun', 1e-3);% for speed
costFun = @costWalk2;
nonlinconFun = @nonlinconWalk2;

%% the big optimization
 stateOpt = fmincon(costFun,state0,A,B,Aeq,Beq,LB,UB,nonlinconFun,options);
%  stateOpt = simulannealbnd(@costWalk1,state0,LB,UB);
 
 %% plot the results
 xyStepOpt = stateOpt(1:end-3);
planexyc = stateOpt(end-2:end);
 xyz(1:2,stanceLegs) = reshape(xyStepOpt, [2,nStanceLegs]); 
% set the feet z positions to be the projection on to the plane
% planex*x + planey*y + z +planec = 0
% z = -planex*x - planey*y -planec
xyz(3,stanceLegs) = ...
    - xyz(1,stanceLegs)*planexyc(1) - xyz(2,stanceLegs)*planexyc(2) - planexyc(3);
  thIK = kin.getIK(xyz);
  xyzFK = kin.getLegPositions(thIK);
   
% get the COM:
 CoMs = kin.getCenterOfMasses(thIK);
bodyCoM = ...
 [ reshape(CoMs, [3,30]) zeros(3,1)]*...
 [reshape(masses, [30,1]); params.robotMass] / sum([reshape(masses, [30,1]); params.robotMass]);
% project it onto the plane:
% difference between origins of plane and point
% dp = bsxfun(@minus, origins, point);
dp = [0; 0; -planexyc(3)] - bodyCoM;
% relative position of point on normal's line
% t = bsxfun(@rdivide, sum(bsxfun(@times,normals,dp),2), sum(normals.^2,2));
normals = [planexyc(1); planexyc(2); 1];
temp = sum(normals.*dp)/sum(normals.^2);
% add relative difference to project point back to plane
% projBodyCoM = bsxfun(@plus, point, bsxfun(@times, t, normals));
projBodyCoM = bodyCoM + temp*normals;

% %  the support polygon
 xyzContact = xyz(:,stanceLegs);
 K = convhull(xyzContact(1:2,:).');
 xOrdered = xyzContact(:,K.');
 xCentroid = mean(xOrdered(:,1:end-1),2);
  
  plt = SnakeMonsterPlotter(); 
  plt.plot(thIK); hold on;
    set(gcf, 'position', [10 100 500 500]);
projectedCOM = scatter3(0,0,0,'k', 'filled');
supportLines = plot3(0,0,0,'k');
scatterCentroid = scatter3(0,0,0,'b');
  set(supportLines, 'xdata', xOrdered(1,:), 'ydata',xOrdered(2,:), ...
     'zdata', xOrdered(3,:));
  inSupport = inpolygon(projBodyCoM(1,end),projBodyCoM(2,end),xOrdered(1,:),xOrdered(2,:));
  set(projectedCOM, 'xdata', projBodyCoM(1), 'ydata',projBodyCoM(2), 'zdata', projBodyCoM(3),...
     'markerFaceColor', [1 0 0]*~inSupport);
  set(scatterCentroid, 'xdata', xCentroid(1), 'ydata',xCentroid(2), 'zdata',xCentroid(3));
  
  % plot the step direction arrow:
  % take the forwards direction, project onto the plane.
  stepDirVector = [0;1; 0]; % directly forwards
  planeNormal = [planexyc(1); planexyc(2); 1];
  projOntoPlane = stepDirVector - dot(stepDirVector, planeNormal)/dot(planeNormal, planeNormal) * planeNormal;
  quiver3(projBodyCoM(1),projBodyCoM(2), projBodyCoM(3), projOntoPlane(1)/2, projOntoPlane(2)/2, projOntoPlane(3)/2);
