function  [ineq_violations,eq_violations]=nonlincon(th)
% nonlinear constriants
% C(X) <= 0 
% in this case C will be a distance from the ground

global kin thExtra nLegs stanceLegs extraLegs zd %masses params
nStanceLegs = length(stanceLegs);
thMat = reshape(th, [3,nStanceLegs]);
thFull = zeros(3,nLegs);
thFull(:,extraLegs) = thExtra;
thFull(:, stanceLegs) = thMat;

xyz = kin.getLegPositions(thFull);
xyzContact = xyz(:,stanceLegs);

ineq_violations = xyzContact(3,:)+zd;

% make sure that the feet are "in order" so they don't overlap
oddInds =find(mod(stanceLegs,2)==1); % the indexes of the odd legs in stanceLegs
evenInds= find(mod(stanceLegs,2)==0);
ineq_violations = [ineq_violations diff(xyzContact(2,oddInds))+.02 diff(xyzContact(2,evenInds))+.02];


eq_violations = [];
end
