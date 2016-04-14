% mainWalkOpt
% main program for walking parameters optimization
% assume a given gait and walking legs,
% find the gait parameters (starting foot positions from body center, step length) 
% which maximize stability for a few time points in the gait cycle
% where stability is COM farthest from the edges of the polygon subject to joint limits 
% allow the transformation between the body and foot plane to change at
% each time step phase tested. Foot start positions fixed in plane.

close all; clc;
addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));
% addpath(genpath('C:\Users\Julian\Box Sync\CMU sem 1+2\snakeMonster\KDC_project'));


global kin params plt A evals
global xyzExtra nLegs stanceLegs extraLegs stepOrder  stanceLegBaseXY
global stepDirection stepLength phasesToTest swingAtPhasesToTest
% stuff to set by hand:
stanceLegs = [ 3 4 5 6]; % array of legs that are in the air, stretched out far


%% sorting and making leg arrays
stanceLegs =sort(stanceLegs); % in case they werent in order
nLegs = 6;
nStanceLegs = length(stanceLegs);

stepDirection = 0; % the heading for the steps in terms of the body frame.
% 0 is straight ahead, pi/2 is walking right, etc.
stepDirection = mod(stepDirection,2*pi);
stepLength = .15; % .1 ok. .15 good for 5 legs.

% walking states: which legs are walking, swinging, extra.
fractionStep = 1/nStanceLegs;
if nStanceLegs ==5
stepOrderBase = [1 3 4 2 6 5]; % works best for leg 1 up
else
   stepOrderBase = [1 2 6 4 5 3]; % works ok
%    stepOrderBase = [1 2 6 3 5 4]; %  no good
%    stepOrderBase = [1 2 6 5 4 3]; %  nope
%       stepOrderBase = [1 2 6 5 3 4 ]; %  nah
%    stepOrderBase = [1 2 5 3 6 4 ]; % not so great
end
stepOrder = [];
extraLegs = [];
% remove the non-walking legs from the step order
for i = 1:6
%     if sum(walkingLegs == stepOrderBase(i))<1
    if ~any(stanceLegs == stepOrderBase(i))
        extraLegs = [extraLegs stepOrderBase(i)]; %#ok<AGROW>
    else
        stepOrder = [stepOrder stepOrderBase(i)]; %#ok<AGROW>
    end
end

%% identify the phases (out of 2pi) in a gait cycle that are of interest
% when the foot has just lifted, and when it is about to set down.
% this is the problem: find tfoot1,2 s.t.:
% t0 + tfoot1 = 0, and mod(t0 + tfoot2,2*pi) = 2*pi*fractionStep, where
% t0 = 2*pi/nStanceLegs*(nStanceLegs:-1:1); the starting phases.
phase0 = mod(2*pi/nStanceLegs*(nStanceLegs:-1:1),2*pi);
swingPhaseLength = 2*pi/nStanceLegs;
% swing legs:   t0: 1 0 0 0 0 (first just took off)
%  t0 + 1*2*pi/nStanceLegs: 1 0 0 0 0 (first is about to land)
%  t0 + 1*2*pi/nStanceLegs: 0 1 0 0 0 (second just took off)
%  t0 + 2*2*pi/nStanceLegs: 0 1 0 0 0 (second about to land)
%  t0 + 2*2*pi/nStanceLegs: 0 0 1 0 0 (third just took off)
%  t0 + 3*2*pi/nStanceLegs: 0 0 1 0 0 (third about to land)
%  t0 + 3*2*pi/nStanceLegs: 0 0 0 1 0 (fourth just took off)
%  t0 + 4*2*pi/nStanceLegs: 0 0 0 1 0 (fourth about to land)
%  t0 + 4*2*pi/nStanceLegs: 0 0 0 0 1 (fifth just took off)
%  t0 + 5*2*pi/nStanceLegs: 0 0 0 0 1 (fifth about to land)
phasesToTest = zeros(2*nStanceLegs, nStanceLegs);
swingAtPhasesToTest = zeros(2*nStanceLegs, nStanceLegs);
% being lazy for now, fill with a for loop. 
for k = 1:nStanceLegs
   swingAtPhasesToTest(2*k-1:2*k,k) = 1;    
end
phasesToTest(1,:) = phase0;%+.01;
phasesToTest(end,:) = phase0;%-.01;
for k = 1:nStanceLegs-1
   phasesToTest(2*k,:)  = phase0 + k*swingPhaseLength;% +.01;
   phasesToTest(2*k+1,:)= phase0 + k*swingPhaseLength;% -.01;
end
phasesToTest = mod(phasesToTest, 2*pi);
nPhases = size(phasesToTest,1);

%% initial pose and parameters
th0 = zeros(1, 3*nLegs); % joint angles: for each leg, proximal to distal
th0([13,16]) = [pi/2, -pi/2]; % start back legs pointing back
params = SMPhysicalParameters();
SMData = makeSMData(params);
kin = SnakeMonsterKinematics; % does all the kinamatics
% masses = kin.getLegMasses();
%% find the "reach out" joint angles for the extra legs
xyz0 = kin.getLegPositions(th0); % zero position of all the legs
xyz = xyz0;
 % set the non-walking legs to be up in the air
 for i = 1:(6-nStanceLegs) 
     xyz(3,extraLegs(i)) =xyz0(3,extraLegs(i)) + .2;
     xyz(2,extraLegs(i)) =xyz0(2,extraLegs(i))*3;
     xyz(1,extraLegs(i)) =xyz0(1,extraLegs(i))*(.097-xyz0(2,extraLegs(i)))/.097;
 end
 xyzExtra = xyz(:,extraLegs);

 % xyz for back legs move back a little
 xyz0(2,[5 6]) = xyz0(2,[5 6])-.05;
 xyz0(2,[3 4]) = xyz0(2,[3 4])+.05;
 
 %% optimization state and constraints setup
% xyzStep0 is the 3xnStepLegs positions of the stepping feet at t=0 in
% the gait cycle
% Here I optimize for the best xyStep0 (which will be 1x2*nStepLegs)
% assuming a fixed zStep0
xyStep0 = reshape(xyz0(1:2,stanceLegs), [1, 2*nStanceLegs]); % initial value: all legs 
% form of: [x1 y1 x2 y2 ...]

% parameters to optimize:
% thetaX at each step (pitch in RB_P)
% thetaY at each step (roll in RB_P)
% (leave yaw = 0 for now)
% rB_P at each step (all three components)
% foot pose x y in plane at t=0
thetaX = 0;
thetaY = 0;
rB_P = [0;0; .2];
transform0 = repmat([thetaX; thetaY; rB_P], [1,nPhases/2] );

% state0 = [xyStep0 transform0(:).'];

% % get the starting position from the last stance optimizer:
state0 = stateOpt;


%% set up the optimization 
 % constraints
 legBaseXY = [params.W/2, -params.W/2, params.W/2, -params.W/2, params.W/2, -params.W/2;...
               params.L/2, params.L/2,      0 ,         0,       -params.L/2, -params.L/2];
stanceLegBaseXY = legBaseXY(:, stanceLegs);
oddInds =find(mod(stanceLegs,2)==1); % the indexes of the odd legs in stanceLegs
evenInds= find(mod(stanceLegs,2)==0);
footUBMat = Inf(2,nStanceLegs); footLBMat = -footUBMat;
footUBMat(1,evenInds) = stanceLegBaseXY(1,evenInds) - (params.l(1) + params.l(1)); % not below body
footLBMat(1,oddInds) = stanceLegBaseXY(1,oddInds) +  (params.l(1) + params.l(1));  % not below body
footUBMat(1,oddInds) = .35+.05; % not too far out 
footLBMat(1,evenInds) = -.35-.05; % not too far out 

% bounds for the transformation
transformUBMat = zeros(size( transform0));
transformLBMat = zeros(size( transform0));
% first row: thetaX
transformUBMat(1,:) = pi/4;
transformLBMat(1,:) = -pi/4;
% second row: thetaY
transformUBMat(2,:) = pi/6;
transformLBMat(2,:) = -pi/6;
% third row: rB_P x
transformUBMat(3,:) = params.W/8; % was /4
transformLBMat(3,:) = -params.W/8; % was /4
% fourth row: rB_P y
transformUBMat(4,:) = params.L/8; % was /4
transformLBMat(4,:) = -params.L/8; % was /4
% fifth row: rB_P z
transformUBMat(5,:) = .35;
transformLBMat(5,:) = .10;

UB = [reshape(footUBMat, [1 2*nStanceLegs]) transformUBMat(:).'];
LB = [reshape(footLBMat, [1 2*nStanceLegs]) transformLBMat(:).'];
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
B = ones(length(oddInds)-1 + length(evenInds)-1 ,1) * -.05; % -.1 works
Aeq = []; % linear eq constraints
Beq = [];
options = optimset('TolCon', 1e-3, 'TolFun', 1e-3, 'MaxFunEvals', 10000 );
costFun = @costWalk7;
nonlinconFun = @nonlinconWalk7;

plt = SnakeMonsterPlotter(); 

%% the big optimization
evals = 0; % number of evaluations of cost function
%  stateOpt = fmincon(costFun,state0,A,B,Aeq,Beq,LB,UB,nonlinconFun,options);

 % use CMA-ES
opts.LBounds = LB.';
opts.UBounds = UB.';
opts.TolFun = 1e-3; % lower function change threshold so that the program actually ends eventually
sigma = (UB-LB/2).'; sigma(sigma>10) = 1;
stateOpt = cmaes('costWalk7cmaes', state0, sigma, opts); 

 
plotResults;
