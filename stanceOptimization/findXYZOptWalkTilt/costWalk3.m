function f = costWalk3(state)

global kin xyzExtra stanceLegs extraLegs  params plt
global stepDirection phasesToTest swingAtPhasesToTest stepOrder
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

% take the forwards direction, project onto the plane.
stepDirVector = R_z(stepDirection)*[0;1;0];
planeNormal = [planexyc(1); planexyc(2); 1];
stepDirOnPlane = stepDirVector - dot(stepDirVector, planeNormal)/dot(planeNormal, planeNormal) * planeNormal;
stepDirOnPlane = stepDirOnPlane/norm(stepDirOnPlane); % normalize to length one

%% For each of the phasesToTest evaluate a cost of projectedCoM and distance
% from centroid. Add them up.
% Harshly penalize being outside the polygon? y=x+exp(x) ?

% stepPeriod = 2*pi*fractionStep;  % how long the stepping lasts
% % solve for coefficients to create trajectory with min jerk
% jerkCoeffs = minimumJerk( 0, 0, 0, ... % Starting Phase/Vel/Accel
%     1, 0, 0, ... % Ending Phase/Vel/Accel
%     stepPeriod);  % Time to touchdown
a_forward = params.L/3; % 
a_back = params.L/3; % step length = a_forward  + a_back
% b = .06; % step height = b. .06 works.
fractionStep = 1/nStanceLegs;
nPhases = size(phasesToTest,1);
costPhases = zeros(1,nPhases);
xyz0 = xyz;
thIK0 = kin.getIK(xyz0); % base stance angles to compare to

for k = 1:nPhases
%     xyz = xyz0;
    % current xyz: move legs to position in the phase.
    for i = 1:nStanceLegs
        leg = stepOrder(i);
        xyz(:,leg) = a_forward*stepDirOnPlane - ...
            (a_forward + a_back)*stepDirOnPlane*...
            (phasesToTest(k,i)/(2*pi - 2*pi*fractionStep) - 2*pi*fractionStep/(2*pi - 2*pi*fractionStep)).'...
            + xyz0(:,leg);
    end
    currentStanceLegs = stanceLegs(~swingAtPhasesToTest(k,:));
    thIK = kin.getIK(xyz);
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
    xyzContactRot = R*xyz(:,currentStanceLegs);
    K = convhull(xyzContactRot(1:2,:).');
    xOrderedRot = xyzContactRot(:,K.');
%     xCentroid = mean(xOrdered(:,1:end-1),2);
    projBodyCoMRot = R*projBodyCoM;
    distToLine = p_poly_dist(projBodyCoMRot(1),projBodyCoMRot(2), xOrderedRot(1,:) ,xOrderedRot(2,:),  1);

    % add this cost to the cost matrix, which will be summed at the end
    costPhases(k) = distToLine + exp(distToLine*100);
    
    % % encourage the points to be far away
delta_xyz = repmat(projBodyCoM, [1,nStanceLegs]) - xyz(:,stanceLegs);
wX= .1; wY = .1; wZ = 0.1;
 costPhases(k) =  costPhases(k) ...
    - delta_xyz(1,:)*wX*delta_xyz(1,:).' ...
    - delta_xyz(2,:)*wY*delta_xyz(2,:).'...
    - delta_xyz(3,:)*wZ*delta_xyz(3,:).';
    
% discourage large flips in angles
 costPhases(k) =  costPhases(k) ...
               + (thIK - thIK0)*(thIK - thIK0).';

%     plt.plot(thIK);
end

f = sum(costPhases);



% thIK = kin.getIK(xyz);

% get the COM:
% CoMs = kin.getCenterOfMasses(thIK);
% bodyCoM = ...
%     [ reshape(CoMs, [3,30]) zeros(3,1)]*...
%     [reshape(masses, [30,1]); params.robotMass] / sum([reshape(masses, [30,1]); params.robotMass]);
% % project it onto the plane:
% % project it onto the plane:
% % difference between origins of plane and point
% % dp = bsxfun(@minus, origins, point);
% dp = [0; 0; -planexyc(3)] - bodyCoM;
% % relative position of point on normal's line
% % t = bsxfun(@rdivide, sum(bsxfun(@times,normals,dp),2), sum(normals.^2,2));
% normals = [planexyc(1); planexyc(2); 1];
% temp = sum(normals.*dp)/sum(normals.^2);
% % add relative difference to project point back to plane
% % projBodyCoM = bsxfun(@plus, point, bsxfun(@times, t, normals));
% projBodyCoM = bodyCoM + temp*normals;

% % %  the support polygon
% xyzContact = xyz(:,stanceLegs);
% K = convhull(xyzContact(1:2,:).');
% xOrdered = xyzContact(:,K.');
% xCentroid = mean(xOrdered(:,1:end-1),2);

% f=0;
% % % this criterion minimizes the distance to the nearest line, using
% % % p_poly_dist
% distToLine = p_poly_dist(projBodyCoM(1),projBodyCoM(2), xOrdered(1,:) ,xOrdered(2,:),  1);
% f = f + distToLine; % returns a negative number if inside the polygon.

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

% % encourage the points to be far away
% delta_xyz = repmat(projBodyCoM, [1,nStanceLegs]) - xyzContact;
% wX= .5; wY = .5; wZ = 0.5;
% f = f ...
%     - delta_xyz(1,:)*wX*delta_xyz(1,:).' ...
%     - delta_xyz(2,:)*wY*delta_xyz(2,:).'...
%     - delta_xyz(3,:)*wZ*delta_xyz(3,:).';


end

