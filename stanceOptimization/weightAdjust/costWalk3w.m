function f = costWalk3w(state)

global kin xyzExtra stanceLegs extraLegs  plt A evals stepHeight extraMass extraMassXYZ
global stepDirection phasesToTest swingAtPhasesToTest stepOrder stepLength masses
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

  
a_forward = stepLength/2; % params.L/3
a_back = stepLength/2; % step length = a_forward  + a_back
% b = .06; % step height = b. .06 works.
fractionStep = 1/nStanceLegs;
nPhases = size(phasesToTest,1);

stepPeriod = 2*pi*fractionStep;  % how long the stepping lasts
stepWayPoints = [-a_back*stepDirOnPlane.';  0 0 stepHeight; a_forward*stepDirOnPlane.'];
jerkCoeffs = minimumJerk( 0, 0, 0, ... % Starting Phase/Vel/Accel
    1, 0, 0, ... % Ending Phase/Vel/Accel
    stepPeriod);  % Time to touchdown
planeNormal = [planexyc(1); planexyc(2); 1]/norm([planexyc(1); planexyc(2); 1]);
[n_theta] = vrrotvec([0;0;1], planeNormal);
  RFromPlane = rotmat(n_theta(1:3).', n_theta(end));
  
costPhases = zeros(1,nPhases);
xyz0 = xyz;
% thIK0 = kin.getIK(xyz0); % base stance angles to compare to

for k = 1:nPhases
%     xyz = xyz0;
    % current xyz: move legs to position in the phase.
    for i = 1:nStanceLegs
        leg = stepOrder(i);        
          xyz(:,leg) = minJerkStepGait3(stepWayPoints, jerkCoeffs, xyz0(:,leg), RFromPlane, fractionStep, phasesToTest(k,i));
   
    end
    currentStanceLegs = stanceLegs(~swingAtPhasesToTest(k,:));
    thIK = kin.getIK(xyz);
    bodyCoM = kin.getSnakeMonsterCoM(thIK);
    
% adjust CoM to take an extra weight 
fullCoM = (bodyCoM*(sum(sum(masses))+2.37) + extraMass*extraMassXYZ )...
    /(sum(sum(masses))+2.37 + extraMass);


    % project it onto the plane:
    dp = [0; 0; -planexyc(3)] - fullCoM;% difference between origins of plane and point
%     planeNormal = [planexyc(1); planexyc(2); 1];% relative position of point on normal's line
    temp = sum(planeNormal.*dp)/sum(planeNormal.^2);
    projBodyCoM = fullCoM + temp*planeNormal;% add relative difference to project point back to plane
    
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

    
%  costPhases(k) = heaviside((distToLine+.02))*((distToLine+.02)*200)^2; % work well
 costPhases(k) = 10*heaviside((distToLine+.02))*((distToLine+.02)*500)^2; % ?
    

%{
% discourage large flips in angles
%  costPhases(k) =  costPhases(k) ...
%                + (thIK - thIK0)*(thIK - thIK0).';
  %}
 

 % discourage the joints from coming too close together.
 CoMs = kin.getCenterOfMasses(thIK);
 %  http://stackoverflow.com/questions/19360047/how-to-build-a-distance-matrix-without-loop
% x = [xl(:)'; yl(:)'; zl(:)'];
% IP = x' * x;
% d = sqrt(bsxfun(@plus, diag(IP), diag(IP)') - 2 * IP);
 % even side:
%  CoMsInterest = CoMs(:,2:end-1,[2 4 6]);
 CoMsInterest = CoMs(:,:,[2 4 6]);
 sizeCoMs = size(CoMsInterest);
x = reshape(permute(CoMsInterest, [3, 2, 1]), [sizeCoMs(2)*sizeCoMs(3),sizeCoMs(1)]).'; % x;y;z for all points
IP = x' * x;
dEven = sqrt(bsxfun(@plus, diag(IP), diag(IP)') - 2 * IP); % matrix with distance from point i to point j
% includes the distance to i to i.
% pointDistCost = 1000*sigmf(-dEven+.04,[100 0]);
pointDistCostEven =heaviside(-(dEven-.06)).*10000.*(dEven-.06).^2;
pointDistCostEven(1:sizeCoMs(3)*sizeCoMs(2)+1:end) = 0; % don't penalize the link being itself!
costPhases(k) =  costPhases(k) ...
    + sum(sum(pointDistCostEven));
% odd side:
%  CoMsInterest = CoMs(:,2:end-1,[1 3 5]); % I only care about collisions near center of leg
 CoMsInterest = CoMs(:,:,[1 3 5]); % 
 sizeCoMs = size(CoMsInterest);
x = reshape(permute(CoMsInterest, [3, 2, 1]), [sizeCoMs(2)*sizeCoMs(3),sizeCoMs(1)]).'; % x;y;z for all points
IP = x' * x;
dOdd = sqrt(bsxfun(@plus, diag(IP), diag(IP)') - 2 * IP); % matrix with distance from point i to point j
% includes the distance to i to i.
% pointDistCost = 1000*sigmf(-dOdd+.04,[100 0]);
pointDistCostOdd =heaviside(-(dOdd-.06)).*10000.*(dOdd-.06).^2;
pointDistCostOdd(1:sizeCoMs(3)*sizeCoMs(2)+1:end) = 0; % don't penalize the link being itself!
costPhases(k) =  costPhases(k) ...
    + sum(sum(pointDistCostOdd));




% plt.plot(thIK);

end
% plt.plot(thIK);
f = sum(costPhases);
   
evals = evals + 1;
 
if ~mod(evals, 500)
    plt.plot(thIK);
end



end
