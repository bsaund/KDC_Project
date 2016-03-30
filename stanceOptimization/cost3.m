function f = cost3(th)
% cost of having joint angles th for the stance legs
% add a cost penalizing angles that make the legs unstable, hacky way.


global kin thExtra nLegs stanceLegs extraLegs masses params
nStanceLegs = length(stanceLegs);
thMat = reshape(th, [3,nStanceLegs]);
thFull = zeros(3,nLegs);
thFull(:,extraLegs) = thExtra;
thFull(:, stanceLegs) = thMat;

xyz = kin.getLegPositions(thFull);
xyzContact = xyz(:,stanceLegs);
z0 = mean(xyzContact(3,:));

% %  the support polygon
 K = convhull(xyzContact(1:2,:).');
 xOrdered = xyzContact(:,K.');
 xCentroid = mean(xOrdered(:,1:end-1),2);
 

 
 % get the COM:
 CoMs = kin.getCenterOfMasses(thFull);
bodyCoM = ...
 [ reshape(CoMs, [3,30]) zeros(3,1)]*...
 [reshape(masses, [30,1]); params.robotMass] / sum([reshape(masses, [30,1]); params.robotMass]);
bodyCoM = [bodyCoM(1:2,:); z0];



% If the feet are going to be stuck to the ground, then we can also assume
% the body is flat. So we throw out the z component.
delta_xy = repmat(bodyCoM(1:2,:), [1,nStanceLegs]) - xyzContact(1:2,:);
distToFeet = sum(delta_xy.^2,1); 

 
f=0;


% % this criterion encourages the points to be far away 
% bigDistWeight = .5;
% f = f -bigDistWeight*sum(distToFeet); % seems to help

% encourage the points to be far away, but more so in the y direction.
wX= .1; wY = 1;
f = f - delta_xy(1,:)*wX*delta_xy(1,:).' - delta_xy(2,:)*wX*delta_xy(2,:).';

% this criterion encourages the COM close to the
% centroid
distToCentroid = sqrt(sum((bodyCoM(1:2) - xCentroid(1:2)).^2));
f = f + distToCentroid;

% % % this criterion penalizes distance to the nearest line
%  % find the min distance to a line
%  distToLine = zeros(1,length(K));
%  for i = 1:length(K)-1
% %      d = abs(cross(Q2-Q1,P-Q1))/abs(Q2-Q1)
%      distToLine(i) = norm(cross(xOrdered(:,i+1)-xOrdered(:,i), bodyCoM-xOrdered(:,i)))...
%          /norm(xOrdered(:,i+1)-xOrdered(:,i));
%  end
% % f = -min(distToLine); % bad!
% f = f-sum(distToLine);

% penalize large joint angles
bigAngle1Weight = .005;
% bigAngleWeight = 0;
% f = f + bigAngleWeight*sum(th.^2);
f = f + bigAngle1Weight*sum(th(1:3:end).^2); % only penalize the first joint

% penalize angles that make the last joint above zero.
bigAngle3Weight = .01;
f = f + sum(th(3:3:end).*(th(3:3:end)>=0));


end