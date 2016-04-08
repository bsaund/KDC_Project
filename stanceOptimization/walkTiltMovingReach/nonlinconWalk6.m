function  [ineq_violations,eq_violations]=nonlinconWalk6(state)
% nonlinear constriants
% C(X) <= 0 
global stanceLegs   stanceLegBaseXY 
nStanceLegs = length(stanceLegs);
% nExtraLegs = length(extraLegs);
% nPhases = size(phasesToTest,1);

xyStep = reshape(state(1:nStanceLegs*2), [2 nStanceLegs]);
% xyExtraMat = reshape(state(nStanceLegs*2+1 : nStanceLegs*2 + nPhases*nExtraLegs ), [nPhases/2, nExtraLegs*2]);
planexyc = state(end-2:end);

xyz = zeros(3,6);
xyz(1:2,stanceLegs) = xyStep; 
% set the feet z positions to be the projection on to the plane
% planex*x + planey*y + z +planec = 0 -->% z = -planex*x - planey*y -planec
xyz(3,stanceLegs) = ...
    - xyz(1,stanceLegs)*planexyc(1) - xyz(2,stanceLegs)*planexyc(2) - planexyc(3);
% leave extralegs alone for now


xyzContact = xyz(:,stanceLegs);

ineq_violations = zeros(1,nStanceLegs); 
% ineq_violations = zeros(1,2*nStanceLegs); 


Rmax = .3; % was .25 in version 3
for k = 1:nStanceLegs
    dFromBody = [xyStep(:,k)- stanceLegBaseXY(:,k); xyzContact(3,k)];
    % make sure the xyz positions stay within a circle of the base
% dist <= Rmax --> dist - Rmax <=0
   ineq_violations(k) =...
      dFromBody.'*dFromBody - Rmax^2;
  % make sure the xyz positions are not too close
% % dist >= Rmin --> Rmin - dist <=0
%    ineq_violations(k+nStanceLegs) =...
%       Rmin^2 -  dFromBody*dFromBody.';
end





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
eq_violations= [];

end
