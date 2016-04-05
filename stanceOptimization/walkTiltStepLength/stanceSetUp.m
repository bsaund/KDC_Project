% stanceSetUp

close all; clc;
addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));
% addpath(genpath('C:\Users\Julian\Box Sync\CMU sem 1+2\snakeMonster\KDC_project'));


global kin masses params plt A
global xyzExtra nLegs stanceLegs extraLegs stepOrder  stanceLegBaseXY
global stepDirection stepLength phasesToTest swingAtPhasesToTest

% stuff to set by hand:
stanceLegs = [  3 4 5 6]; % array of legs that are in the air, stretched out far


%% sorting and making leg arrays
stanceLegs =sort(stanceLegs); % in case they werent in order
nLegs = 6;
nStanceLegs = length(stanceLegs);

stepDirection = 0; % the heading for the steps in terms of the body frame.
% 0 is straight ahead, pi/2 is walking right, etc.
stepDirection = mod(stepDirection,2*pi);
stepLength = .1; % might later be an optimized parameter

% walking states: which legs are walking, swinging, extra.
fractionStep = 1/nStanceLegs;
if nStanceLegs ==5
stepOrderBase = [1 3 4 2 6 5]; % works best for leg 1 up
else
%    stepOrderBase = [1 2 6 4 5 3]; % works ok
   stepOrderBase = [1 2 6 3 4 5];
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

params = SMPhysicalParameters();
SMData = makeSMData(params);
kin = SnakeMonsterKinematics; % does all the kinamatics

%% find the "reach out" joint angles for the extra legs
th0 = zeros(1, 3*nLegs); % joint angles: for each leg, proximal to distal
xyz0 = kin.getLegPositions(th0); % zero position of all the legs
xyz = xyz0;
 % set the non-walking legs to be up in the air
 for i = 1:(6-nStanceLegs) 
     xyz(3,extraLegs(i)) =xyz0(3,extraLegs(i)) + .2;
     xyz(2,extraLegs(i)) =xyz0(2,extraLegs(i))*5;
     xyz(1,extraLegs(i)) =xyz0(1,extraLegs(i))*(.097-xyz0(2,extraLegs(i)))/.097;
 end
 xyzExtra = xyz(:,extraLegs);
 
  legBaseXY = [params.W/2, -params.W/2, params.W/2, -params.W/2, params.W/2, -params.W/2;...
               params.L/2, params.L/2,      0 ,         0,       -params.L/2, -params.L/2];
stanceLegBaseXY = legBaseXY(:, stanceLegs);
oddInds =find(mod(stanceLegs,2)==1); % the indexes of the odd legs in stanceLegs
evenInds= find(mod(stanceLegs,2)==0);
 % linear ineq constraints: the front legs are in front of the back legs, etc.
A = zeros(length(oddInds)-1 + length(evenInds)-1, 2*nStanceLegs +3);
for i = 1:length(oddInds)-1
A(i,oddInds(i)*2) = -1;
A(i,oddInds(i+1)*2) = 1;
end
for i = 1:length(evenInds)-1
A(i+length(oddInds)-1,evenInds(i)*2) = -1;
A(i+length(oddInds)-1,evenInds(i+1)*2) = 1;
end
B = ones(length(oddInds)-1 + length(evenInds)-1 ,1) * -.1;

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
  
