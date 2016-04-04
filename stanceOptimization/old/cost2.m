function f = cost2(th)
% cost of having joint angles th for the stance legs
% also has torques, assuming all stance feet are on the ground at the same
% height

global kin thExtra nLegs stanceLegs extraLegs masses params
nStanceLegs = length(stanceLegs);
thMat = reshape(th, [3,nStanceLegs]);
thFull = zeros(3,nLegs);
thFull(:,extraLegs) = thExtra;
thFull(:, stanceLegs) = thMat;

xyz = kin.getLegPositions(thFull);
xyzContact = xyz(:,stanceLegs);

% %  the support polygon
K = convhull(xyzContact(1:2,:).');
xOrdered = xyzContact(:,K.');
xCentroid = mean(xOrdered(:,1:end-1),2);



% get the COM:
CoMs = kin.getCenterOfMasses(thFull);
bodyCoM = ...
    [ reshape(CoMs, [3,30]) zeros(3,1)]*...
    [reshape(masses, [30,1]); params.robotMass] / sum([reshape(masses, [30,1]); params.robotMass]);

% finding torques on stance legs
bodyMass = (params.robotMass +sum(masses(1,:))/2);
legTorques = zeros(3,nStanceLegs);
% the commanded foot force based on a best fit to the force distribution
cmdFootForce = pinv(xyzContact) * [0; 0; bodyMass];
J = kin.getLegJacobians(thFull);
for i=1:nStanceLegs
    footWrench = [0;0;cmdFootForce(i); zeros(3,1)];
    legTorques(:,i) =  legTorques(:,i) + J(:,:,stanceLegs(i)).'*footWrench;
end


% If the feet are going to be stuck to the ground, then we can also assume
% the body is flat. So we throw out the z component.
delta_xy = repmat(bodyCoM(1:2,:), [1,nStanceLegs]) - xyzContact(1:2,:);
distToFeet = sum(delta_xy.^2,1);

%  % find the min distance to a line
%  distToLine = zeros(1,length(K));
%  for i = 1:length(K)-1
% %      d = abs(cross(Q2-Q1,P-Q1))/abs(Q2-Q1)
%      distToLine(i) = norm(cross(xOrdered(:,i+1)-xOrdered(:,i), bodyCoM-xOrdered(:,i)))...
%          /norm(xOrdered(:,i+1)-xOrdered(:,i));
%  end

f=0;
% this criterion encourages the points to be far away
f = f - sum(distToFeet); % seems to help

% this criterion penalizes the minimum distance
% f = -min(dists); % does not work

% this criterion encourages the points to be far away and close to the
% centroid
distToCentroid = sqrt(sum((bodyCoM(1:2) - xCentroid(1:2)).^2));
f = f + distToCentroid;

% % this criterion penalizes distance to the nearest line
% f = -min(distToLine); % Too many local min

% penalize large joint angles
bigAngleWeight = .005;
% bigAngleWeight = 0;
f = f + bigAngleWeight*sum(th.^2);

% penalize high torques
bigTorqueWeight = .5;
f = f + bigTorqueWeight*sum(sum(legTorques.^2));

end