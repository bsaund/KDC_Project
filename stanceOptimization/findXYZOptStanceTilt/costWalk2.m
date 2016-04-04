function f = costWalk2(state)

global kin xyzExtra  stanceLegs extraLegs
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

 thIK = kin.getIK(xyz);
 
 
planeNormal = [planexyc(1); planexyc(2); 1];
  bodyCoM = kin.getSnakeMonsterCoM(thIK);
    % project it onto the plane:
    dp = [0; 0; -planexyc(3)] - bodyCoM;% difference between origins of plane and point
%     planeNormal = [planexyc(1); planexyc(2); 1];% relative position of point on normal's line
    temp = sum(planeNormal.*dp)/sum(planeNormal.^2);
    projBodyCoM = bodyCoM + temp*planeNormal;% add relative difference to project point back to plane
    
   % find the rotation matrix that maps the points in the foot plane into
   % an xy plane
   [n_theta] = vrrotvec(planeNormal,[0;0;1]);
    R = rotmat(n_theta(1:3).', n_theta(end));
    % find the polygon and the distances in this rotated plane
    xyzContactRot = R*xyz(:,stanceLegs);
    K = convhull(xyzContactRot(1:2,:).');
    xOrdered = xyzContactRot(:,K.');
%     xCentroid = mean(xOrdered(:,1:end-1),2);
    projBodyCoMRot = R*projBodyCoM;
%     distToLine = p_poly_dist(projBodyCoMRot(1),projBodyCoMRot(2), xOrdered(1,:) ,xOrdered(2,:),  1);
 
 
 f=0;
% % this criterion minimizes the distance to the nearest line, using
% % p_poly_dist
% distToLine = p_poly_dist(projBodyCoM(1),projBodyCoM(2), xOrdered(1,:) ,xOrdered(2,:),  1);
    distToLine = p_poly_dist(projBodyCoMRot(1),projBodyCoMRot(2), xOrdered(1,:) ,xOrdered(2,:),  1);
f = f + distToLine; % returns a negative number if inside the polygon.


% add a small penalty for large tilts
f = f + (planexyc(1:2)*planexyc(1:2).')*.1;


% criterion minimizing the distances to the nearest line:
% % http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
% distToLines = zeros(1,length(K)-1);
% for k = 1:(length(K)-1)
%     distToLines(k) = norm(cross(projBodyCoM - xOrdered(:,k), projBodyCoM - xOrdered(:,k+1)))/...
%         norm(xOrdered(:,k+1) - xOrdered(:,k));
% end
% % take only the closest one, and multiply if be wether its in the support
% minDistToLine = min(distToLines)*inpolygon(projBodyCoM(1),projBodyCoM(2),xOrdered(1,:),xOrdered(2,:));
% f = f + minDistToLine;

% % % this criterion encourages the COM close to the centroid
% distToCentroid = sqrt(sum((bodyCoM(1:2) - xCentroid(1:2)).^2));
% f = f + distToCentroid;

% % % encourage the points to be far away
% delta_xyz = repmat(projBodyCoM, [1,nStanceLegs]) - xyzContact;
% wX= .5; wY = .5; wZ = 0.5;
% f = f ...
%     - delta_xyz(1,:)*wX*delta_xyz(1,:).' ...
%     - delta_xyz(2,:)*wY*delta_xyz(2,:).'...
%     - delta_xyz(3,:)*wZ*delta_xyz(3,:).';

 
end




%{
% % get the COM:
%  CoMs = kin.getCenterOfMasses(thIK);
% bodyCoM = ...
%  [ reshape(CoMs, [3,30]) zeros(3,1)]*...
%  [reshape(masses, [30,1]); params.robotMass] / sum([reshape(masses, [30,1]); params.robotMass]);
% 
% % project it onto the plane:
% % project it onto the plane:
% difference between origins of plane and point
% dp = bsxfun(@minus, origins, point);
% dp = [0; 0; -planexyc(3)] - bodyCoM;
% % relative position of point on normal's line
% % t = bsxfun(@rdivide, sum(bsxfun(@times,normals,dp),2), sum(normals.^2,2));
% normals = [planexyc(1); planexyc(2); 1];
% temp = sum(normals.*dp)/sum(normals.^2);
% % add relative difference to project point back to plane
% % projBodyCoM = bsxfun(@plus, point, bsxfun(@times, t, normals));
% projBodyCoM = bodyCoM + temp*normals;

% % %  the support polygon
%  xyzContact = xyz(:,stanceLegs);
%  K = convhull(xyzContact(1:2,:).');
%  xOrdered = xyzContact(:,K.');
%  xCentroid = mean(xOrdered(:,1:end-1),2);



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

 
f=0;


% % this criterion encourages the points to be far away 
% bigDistWeight = .5;
% f = f -bigDistWeight*sum(distToFeet); % seems to help

% encourage the points to be far away, but more so in the y direction.
% If the feet are going to be stuck to the ground, then we can also assume
% the body is flat. So we throw out the z component.
delta_xy = repmat(bodyCoM(1:2,:), [1,nStanceLegs]) - xyzContact(1:2,:);
% distToFeet = sum(delta_xy.^2,1); 
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
% f = f-sum(distToLine); % meh

% this criterion minimizes the distance to the nearest line, using
% % p_poly_dist
% distToLine = p_poly_dist(bodyCoM(1),bodyCoM(2), xOrdered(1,:) ,xOrdered(2,:),  1);
% f = f + distToLine; % returns a negative number if inside the polygon.

% penalize large joint angles
bigAngle1Weight = .005;
% bigAngleWeight = 0;
% f = f + bigAngleWeight*sum(th.^2);
f = f + bigAngle1Weight*sum(th(1:3:end).^2); % only penalize the first joint

% penalize angles that make the last joint above zero.
bigAngle3Weight = .01;
f = f + sum(th(3:3:end).*(th(3:3:end)>=0));


end
%}