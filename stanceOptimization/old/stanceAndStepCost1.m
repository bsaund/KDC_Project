function f  = stanceAndStepCost1(th)

% stanceAndStepOpt
% find a stance such that if each leg is lifted the 
% distToLine is maximized

global kin thExtra nLegs stanceLegs extraLegs masses params
nStanceLegs = length(stanceLegs);
thMat = reshape(th, [3,nStanceLegs]);
thFull = zeros(3,nLegs);
thFull(:,extraLegs) = thExtra;
thFull(:, stanceLegs) = thMat;

xyz = kin.getLegPositions(thFull);
xyzContact = xyz(:,stanceLegs);
z0 = mean(xyzContact(3,:));
 % get the COM:
 CoMs = kin.getCenterOfMasses(thFull);
bodyCoM = ...
 [ reshape(CoMs, [3,30]) zeros(3,1)]*...
 [reshape(masses, [30,1]); params.robotMass] / sum([reshape(masses, [30,1]); params.robotMass]);
bodyCoM = [bodyCoM(1:2,:); z0];

legLiftedLineDists = zeros(1,nStanceLegs);
distToCentroid = zeros(1,nStanceLegs);

for i = 1:nStanceLegs
%     legLifted = stanceLegs(i);
    newStanceLegs = stanceLegs([1:i-1, i+1:end]);
    xyzNewContact = xyz(:,newStanceLegs);
%  the support polygon
 K = convhull(xyzNewContact(1:2,:).');
 xOrdered = xyzNewContact(:,K.');
 xCentroid = mean(xOrdered(:,1:end-1),2);

%    find the min distance to a line
 distToLine = zeros(1,length(K)-1);
 for j = 1:length(K)-1
%      d = abs(cross(Q2-Q1,P-Q1))/abs(Q2-Q1)
     distToLine(j) = norm(cross(xOrdered(:,j+1)-xOrdered(:,j), bodyCoM-xOrdered(:,j)))...
         /norm(xOrdered(:,j+1)-xOrdered(:,j));
 end
 
 legLiftedLineDists(i) = sum(distToLine.^2);
%  legLiftedLineDists(i) = 1/prod(distToLine*10);
distToCentroid(i) = sqrt(sum((bodyCoM(1:2) - xCentroid(1:2)).^2));
end

f = -sum(legLiftedLineDists);


% this criterion encourages the COM close to the
% centroid
f = f + 2*sum(distToCentroid);

% encourage the points to be far away, but more so in the y direction.
% If the feet are going to be stuck to the ground, then we can also assume
% the body is flat. So we throw out the z component.
delta_xy = repmat(bodyCoM(1:2,:), [1,nStanceLegs]) - xyzContact(1:2,:);
% distToFeet = sum(delta_xy.^2,1); 
wX= .1; wY = .5;
f = f - delta_xy(1,:)*wX*delta_xy(1,:).' - delta_xy(2,:)*wX*delta_xy(2,:).';

end





%{
% optimize stance and step order, for a given step direction and magnitude.
% x = [thInit stepOrder] s.t. distToLine(CoM, xyzContact) is minimized for
% all the step waypoints (start of step, midstep, end of step)

% set a step direction and magnitude
stepDirection = 0; % deviation away from straight, radians
stepMag = .1; % how far to move

 % specify which legs will be used for walking
walkingLegs = [ 2 3 4 5 6];
nWalkingLegs = length(walkingLegs);

% initial conditions for optimizer
stepOrder0 = sort(walkingLegs);
th0 = zeros(1, 3*nWalkingLegs);
state0 = [th0 stepOrder0]; %% this has a discrete component so it wont'
work......
%}
