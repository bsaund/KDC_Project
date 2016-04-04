function  [ineq_violations,eq_violations]=nonlinconWalk3(state)
% nonlinear constriants
% C(X) <= 0 
global xyzExtra stanceLegs extraLegs  stanceLegBaseXY
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

xyzContact = xyz(:,stanceLegs);
ineq_violations = zeros(1,nStanceLegs); 

% make sure the xy positions stay within a circle of the base
R = .2;
for k = 1:nStanceLegs
    dFromBody = [xyStep(2*k-1:2*k)- stanceLegBaseXY(:,k).', xyzContact(3,k)^2];
   ineq_violations(k) =...
      dFromBody*dFromBody.' - R^2;
end

% make sure the z component is <0
ineq_violations = [ineq_violations, xyz(3,stanceLegs)];


% % make sure that the feet are "in order" so they don't overlap.. .works
% ok
% oddInds =find(mod(stanceLegs,2)==1); % the indexes of the odd legs in stanceLegs
% evenInds= find(mod(stanceLegs,2)==0);
% ineq_violations = [ineq_violations diff(xyzContact(2,oddInds))+.02 diff(xyzContact(2,evenInds))+.02];


% % make sure the FK and IK match... slow and crappy.
%  thIK = kin.getIK(xyz);
%  xyzFK = kin.getLegPositions(thIK);
% xyz(:,stanceLegs) - xyzFK(:,stanceLegs)
eq_violations= [];

end
