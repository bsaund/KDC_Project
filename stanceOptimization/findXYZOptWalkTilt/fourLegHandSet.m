% four leg scripted gait
% the starting positions and such for a 4 leg walk that's set by hand

stanceSetUp
 
 %% do a little extra work here by hand
 xyz0(2,[3, 4]) =  xyz0(2,[3, 4]) + .1; % bring middle two forwards
 xyz0(2,[5 6]) =  xyz0(2,[5 6]) - .05; % bring back two back
 xyz0(1,[3, 5]) =   xyz0(1,[3, 5])  - [.15 .15]; % bring odd legs in
  xyz0(1,[4, 6]) =   xyz0(1,[4, 6])  + [.15 .15]; % bring even legs in
  
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
planey = .4;
planec = .15;
state0 = [xyStep0 planex planey planec];

stateOpt = state0;

plotOptResultsStatic

runGait

