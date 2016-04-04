function  [ineq_violations,eq_violations]=nonlinconWalk1(state)
% nonlinear constriants
% C(X) <= 0 
global kin xyzExtra nLegs stanceLegs extraLegs masses params zd
nStanceLegs = length(stanceLegs);
xyStep = state(1:end-1);


xyz = zeros(3,6);
xyz(:,extraLegs) =xyzExtra;
xyz(1:2,stanceLegs) = reshape(xyStep, [2,nStanceLegs]); 
xyz(3,stanceLegs) = state(end);
xyzContact = xyz(:,stanceLegs);
ineq_violations = []; 

% make sure that the feet are "in order" so they don't overlap
oddInds =find(mod(stanceLegs,2)==1); % the indexes of the odd legs in stanceLegs
evenInds= find(mod(stanceLegs,2)==0);
ineq_violations = [ineq_violations diff(xyzContact(2,oddInds))+.02 diff(xyzContact(2,evenInds))+.02];

% % make sure the FK and IK match... slow and crappy.
%  thIK = kin.getIK(xyz);
%  xyzFK = kin.getLegPositions(thIK);
% xyz(:,stanceLegs) - xyzFK(:,stanceLegs)
eq_violations= [];

end
