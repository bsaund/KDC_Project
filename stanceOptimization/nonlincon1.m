function  [ineq_violations,eq_violations]=nonlincon1(th)
% nonlinear constriants
% C(X) <= 0 
% in this case C will be a distance from the ground

global kin thExtra nLegs stanceLegs extraLegs zd masses params
nStanceLegs = length(stanceLegs);
thMat = reshape(th, [3,nStanceLegs]);
thFull = zeros(3,nLegs);
thFull(:,extraLegs) = thExtra;
thFull(:, stanceLegs) = thMat;

xyz = kin.getLegPositions(thFull);
xyzContact = xyz(:,stanceLegs);

 % get the COM:
 CoMs = kin.getCenterOfMasses(thFull);
bodyCoM = ...
 [ reshape(CoMs, [3,30]) zeros(3,1)]*...
 [reshape(masses, [30,1]); params.robotMass] / sum([reshape(masses, [30,1]); params.robotMass]);
% %  the support polygon
 K = convhull(xyzContact(1:2,:).');
 xOrdered = xyzContact(:,K.');
inSupport = inpolygon(bodyCoM(1,end),bodyCoM(2,end),xOrdered(1,:),xOrdered(2,:));

% all stance legs are below some threshold:
ineq_violations = xyzContact(3,:)+zd; 

% all stance feet are roughly at the same height:
ineq_violations = [ineq_violations (mean(xyzContact(3,:)) - xyzContact(3,:)).^2 - .02^2];



% make sure that the feet are "in order" so they don't overlap
oddInds =find(mod(stanceLegs,2)==1); % the indexes of the odd legs in stanceLegs
evenInds= find(mod(stanceLegs,2)==0);
ineq_violations = [ineq_violations diff(xyzContact(2,oddInds))+.02 diff(xyzContact(2,evenInds))+.02];


eq_violations = double([~inSupport]);
end
