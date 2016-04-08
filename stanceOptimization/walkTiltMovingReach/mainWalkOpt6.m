% mainWalkOpt
% main program for walking parameters optimization
% assume a given gait and walking legs,
% find the gait parameters (starting foot positions from body center, step length) 
% which maximize stability for a few time points in the gait cycle
% where stability is COM farthest from the edges of the polygon subject to joint limits 

% add ability to reach out to keep arm up but manipulate CoM better
% add movement to extra arm: they can swing around to differnet places each
% of the phases to check

close all; clc; 
addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));
% addpath(genpath('C:\Users\Julian\Box Sync\CMU sem 1+2\snakeMonster\KDC_project'));


global kin params plt A evals
global nLegs  stanceLegs extraLegs stepOrder  stanceLegBaseXY
global stepDirection stepLength phasesToTest swingAtPhasesToTest
% stuff to set by hand:
stanceLegs = [ 2 3 4 5 6]; % array of legs that are in the air, stretched out far


%% sorting and making leg arrays
stanceLegs =sort(stanceLegs); % in case they werent in order
nLegs = 6;
allLegs = 1:nLegs;
nStanceLegs = length(stanceLegs);
nExtraLegs = nLegs - nStanceLegs;

stepDirection = 0; % the heading for the steps in terms of the body frame.
% 0 is straight ahead, pi/2 is walking right, etc.
stepDirection = mod(stepDirection,2*pi);
stepLength = .1; % might later be an optimized parameter

% walking states: which legs are walking, swinging, extra.
fractionStep = 1/nStanceLegs;
if nStanceLegs ==5
stepOrderBase = [1 3 4 2 6 5]; % works best for leg 1 up
else
   stepOrderBase = [1 2 6 4 5 3]; % works ok
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
% swing legs:           t0: 1 0 0 0 0 (first just took off)
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
% th0([12,15]) = [-pi/2, pi/2];
params = SMPhysicalParameters();
SMData = makeSMData(params);
kin = SnakeMonsterKinematics; % does all the kinamatics
% masses = kin.getLegMasses();
%% find the "reach out" joint angles for the extra legs
xyz0 = kin.getLegPositions(th0); % zero position of all the legs
xyz = xyz0;
 % set the non-walking legs to be up in the air
 for i = 1:(6-nStanceLegs) 
     xyz(3,extraLegs(i)) =0;
     xyz(2,extraLegs(i)) =xyz0(2,extraLegs(i))*3;
     xyz(1,extraLegs(i)) =xyz0(1,extraLegs(i))*.5;
 end
 xyzExtra = xyz(1:2,extraLegs);
xyzExtra0 = reshape(xyzExtra, [1,2*length(extraLegs)]);
 
 % xyz for back legs move back a little
 xyz0(2,[5 6]) = xyz0(2,[5 6])-.05;
 xyz0(2,[3 4]) = xyz0(2,[3 4])+.05;
 
 %% optimization state and constraints setup
% xyzStep0 is the 3xnStepLegs positions of the stepping feet at t=0 in
% the gait cycle
% Here I optimize for the best xyStep0 (which will be 1x2*nStepLegs)
% assuming a fixed zStep0
xyStance0 = reshape(xyz0(1:2,stanceLegs), [1, 2*nStanceLegs]); % initial value
% form of: [x1 y1 x2 y2 ...]
xyExtraMat0 = repmat(xyzExtra0, [nPhases/2, 1]);
xyExtraPhases0 = reshape(xyExtraMat0, [1, nExtraLegs*nPhases]);


% fitting a plane in stead of a z0... planex*x + planey*y + z + planec = 0
% this is a plane 
planex = 0;
planey = .2;
planec = .2;
% state0 = [xyStance0 xyExtraPhases0 planex planey planec ];


state0 = stateOpt;


%% set up the optimization 
 % constraints
 legBaseXY = [params.W/2, -params.W/2, params.W/2, -params.W/2, params.W/2, -params.W/2;...
               params.L/2, params.L/2,      0 ,         0,       -params.L/2, -params.L/2];
stanceLegBaseXY = legBaseXY(:, stanceLegs);
oddInds =find(mod(stanceLegs,2)==1); % the indexes of the odd legs in stanceLegs
evenInds= find(mod(stanceLegs,2)==0);
xyStepUBMat = Inf(2,nStanceLegs); xyStepLBMat = -xyStepUBMat;
xyStepUBMat(1,evenInds) = stanceLegBaseXY(1,evenInds) - (params.l(1) + params.l(1)); % not below body
xyStepLBMat(1,oddInds) = stanceLegBaseXY(1,oddInds) +  (params.l(1) + params.l(1));  % not below body
xyStepUBMat(1,oddInds) = .35; % not too far out 
xyStepLBMat(1,evenInds) = -.35; % not too far out 
xyStepUBMat(2,:) = stanceLegBaseXY(2,:) +.3; % not too far forwards
xyStepLBMat(2,:) = stanceLegBaseXY(2,:) -.3; % not too far back

xyExtraUBMat = Inf(size(xyExtraMat0)); xyExtraLBMat = -xyExtraUBMat;
oddIndsEx =find(mod(extraLegs,2)==1); % the indexes of the odd legs in extraLegs
evenIndsEx= find(mod(extraLegs,2)==0);
xyExtraUBMat(:,evenIndsEx*2-1) = -params.W/2 -params.l(1); % x ub even
xyExtraUBMat(:,oddIndsEx*2-1) = .4; % x ub odd
xyExtraLBMat(:,evenIndsEx*2-1) = -.4; % x lb even
xyExtraLBMat(:,oddIndsEx*2-1) = params.W/2 +params.l(1); % x lb odd
xyExtraUBMat(:,evenIndsEx*2) = .5; % y ub even
xyExtraUBMat(:,oddIndsEx*2) = .5; % y ub odd
xyExtraLBMat(:,evenIndsEx*2) = .1; % y lb even
xyExtraLBMat(:,oddIndsEx*2) = .1; % y lb odd

% give a smaller limit on the x direction tilt than the y direction.
planeUB = [ .02 .5 .2];
planeLB = [-.02 -.5 .05];
UB = [reshape(xyStepUBMat, [1 2*nStanceLegs]), reshape(xyExtraUBMat, [1, nExtraLegs*nPhases]), planeUB];
LB = [reshape(xyStepLBMat, [1 2*nStanceLegs]), reshape(xyExtraLBMat, [1, nExtraLegs*nPhases]), planeLB];
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
B = ones(length(oddInds)-1 + length(evenInds)-1 ,1) * -.1;
Aeq = []; % linear eq constraints
Beq = [];
options = optimset('TolCon', 1e-3, 'TolFun', 1e-3, 'MaxFunEvals', 10000 );
costFun = @costWalk6;
nonlinconFun = @nonlinconWalk6;

 plt = SnakeMonsterPlotter(); 

%% the big optimization
evals = 0; % number of evaluations of cost function
 stateOpt = fmincon(costFun,state0,A,B,Aeq,Beq,LB,UB,nonlinconFun,options);
%  stateOpt = simulannealbnd(@costWalk1,state0,LB,UB);
 
plotOptResultsStatic;
