function  [ineq_violations,eq_violations]=nonlinconWalk3w(state)
% nonlinear constriants
% C(X) <= 0 
global xyzExtra stanceLegs extraLegs  kin stepHeight A
global stepDirection stepLength phasesToTest stepOrder 
nStanceLegs = length(stanceLegs);
xyStep = state(1:end-3);
planexyc = state(end-2:end);

xyz = zeros(3,6);
xyz(:,extraLegs) =xyzExtra;
xyz(1:2,stanceLegs) = reshape(xyStep, [2,nStanceLegs]); 
% set the feet z positions to be the projection on to the plane
% planex*x + planey*y + z +planec = 0
% z = -planex*x - planey*y -planec
xyz(3,stanceLegs) = ...
    - xyz(1,stanceLegs)*planexyc(1) - xyz(2,stanceLegs)*planexyc(2) - planexyc(3);


ineq_violations = zeros(1,nStanceLegs); 
% ineq_violations = zeros(1,2*nStanceLegs); 


% get the forwards direction vector
stepDirVector = R_z(stepDirection)*[0;1;0];
a_forward = stepLength/2; % params.L/3
a_back = stepLength/2; % step length = a_forward  + a_back
% b = .06; % step height = b. .06 works.
fractionStep = 1/nStanceLegs;
nPhases = size(phasesToTest,1);

planeNormal = [planexyc(1); planexyc(2); 1]/norm([planexyc(1); planexyc(2); 1]);
[n_theta] = vrrotvec([0;0;1], planeNormal);

  stepDirOnPlane = stepDirVector - dot(stepDirVector, planeNormal)/dot(planeNormal, planeNormal) * planeNormal;
stepDirOnPlane = stepDirOnPlane/norm(stepDirOnPlane); % normalize to length one


stepPeriod = 2*pi*fractionStep;  % how long the stepping lasts
stepWayPoints = [-a_back*stepDirOnPlane.';  0 0 stepHeight; a_forward*stepDirOnPlane.'];
jerkCoeffs = minimumJerk( 0, 0, 0, ... % Starting Phase/Vel/Accel
    1, 0, 0, ... % Ending Phase/Vel/Accel
    stepPeriod);  % Time to touchdown

  RFromPlane = rotmat(n_theta(1:3).', n_theta(end));


FKViolationsMat5 = zeros(nStanceLegs ,nPhases/2); % for FK matching

xyz0 = xyz;

for k = 1:nPhases/2
    
    % current xyz: move feet to position in the phase. In plane.
    for i = 1:nStanceLegs
        leg = stepOrder(i);
     xyz(:,leg) = minJerkStepGait3(stepWayPoints, jerkCoeffs, xyz0(:,leg), RFromPlane, fractionStep, phasesToTest(k*2-1,i));
    
    end

% force the FK to match the xyzB desired points, within a centimeter

    xyzContact = xyz(:,stanceLegs);
    thIK = kin.getIK(xyz);
    xyzFK = kin.getLegPositions(thIK);
   errs = xyzContact - xyzFK(:,stanceLegs);
  FKViolationsMat5(:,k) = sqrt(sum(errs.^2)) - 0;
  
%   plt.plot(thIK)
  
% penalize having a foot in front of another during the gait
xyStepT = reshape(xyz(1:2,stanceLegs), [1,2*nStanceLegs]);
footOverlap= A*[xyStepT 0 0 0].' + .05;
 ineq_violations = [ineq_violations, footOverlap.'];

end
% ineq_violations = [ineq_violations, FKViolationsMat5(:).'];

% make sure the z component is <0
ineq_violations = [ineq_violations, xyz(3,stanceLegs)+.1];



% % make sure that the feet are "in order" so they don't overlap.. .works
% ok
% oddInds =find(mod(stanceLegs,2)==1); % the indexes of the odd legs in stanceLegs
% evenInds= find(mod(stanceLegs,2)==0);
% ineq_violations = [ineq_violations diff(xyzContact(2,oddInds))+.02 diff(xyzContact(2,evenInds))+.02];


% % make sure the FK and IK match... slow and crappy.
%  thIK = kin.getIK(xyz);
%  xyzFK = kin.getLegPositions(thIK);
% xyz(:,stanceLegs) - xyzFK(:,stanceLegs)
eq_violations= [FKViolationsMat5(:).'];

end
