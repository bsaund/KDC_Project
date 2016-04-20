function  [ineqViolations,eqViolations]=nonlinconWalk10(state)
% nonlinear constriants
% C(X) <= 0
global   stanceLegs    A kin  xyzExtra  extraLegs  swingAtPhasesToTest
global stepDirection phasesToTest stepOrder stepLength nLegs test
nStanceLegs = length(stanceLegs);

% test = state;


xyStep = state(1:2*nStanceLegs);
transforms = state(2*nStanceLegs+1:end);

xyz = zeros(3,6);
xyz(1:2,stanceLegs) = reshape(xyStep, [2,nStanceLegs]);
% foot positions on the plane are zero.
xyz(3,stanceLegs) = 0;

% get the forwards direction vector
stepDirVector = R_z(stepDirection)*[0;1;0];


%% For each of the phasesToTest evaluate a cost of projectedCoM and distance
% from centroid. Add them up.
% Harshly penalize being outside the polygon


a_forward = stepLength/2; % params.L/3
a_back = stepLength/2; % step length = a_forward  + a_back
b = .06; % step height = b. .06 works.

fractionStep = 1/nStanceLegs;
stepPeriod = 2*pi*fractionStep;  % how long the stepping lasts
% solve for coefficients to create trajectory with min jerk
jerkCoeffs = minimumJerk( 0, 0, 0, ... % Starting Phase/Vel/Accel
    1, 0, 0, ... % Ending Phase/Vel/Accel
    stepPeriod);  % Time to touchdown
stepWayPoints = [-a_back*stepDirVector.';  0 0 b; a_forward*stepDirVector.'];

nPhases = size(phasesToTest,1);

% ineqViolationsMat1 = zeros(nStanceLegs,nPhases/2); % for max radius
% ineqViolationsMat2 = zeros(nStanceLegs,nPhases/2); % for min radius
ineqViolationsMat1 =zeros(1,nPhases); 
ineqViolationsMat2=[];

oddInds =find(mod(stanceLegs,2)==1); % the indexes of the odd legs in stanceLegs
evenInds= find(mod(stanceLegs,2)==0);

ineqViolationsMat3 = zeros(length(oddInds)-1 + length(evenInds)-1, nPhases); % for foot overlap
ineqViolationsMat4 = zeros(nStanceLegs ,nPhases); % for max foot height
ineqViolationsMat5 = zeros(nStanceLegs ,nPhases); % for FK matching

xyz0 = xyz;

transformsMat = reshape(transforms, [5 nPhases]);


for k = 1:nPhases
    
    % current xyz: move feet to position in the phase. In plane.
    for i = 1:nStanceLegs
        leg = stepOrder(i);
          xyz(:,leg) = minJerkStepGait3(stepWayPoints, jerkCoeffs, xyz0(:,leg), eye(3), fractionStep, phasesToTest(k,i));
    end
    
    % find the positions of the feet in the body frame.
    % transformation of Body viewed from Plane
    rB_P = transformsMat(3:end, k); % vector of B wrt P,
    % (body frame wrt plane origin)
    thetaX = transformsMat(1, k);
    thetaY = transformsMat(2, k);
    RB_P = R_y(thetaY)*R_x(thetaX); % rotation of the frame B wrt P frame
    % (body frame coordinate system as viewed in plane frame)
    TB_P = [RB_P rB_P; 0 0 0 1]; % transformation to the body frame from the plane frame
    TP_B = inv(TB_P);
    xyzBh = TP_B*[xyz; ones(1,nLegs)];
    xyzBContact =xyzBh(1:3,stanceLegs);
    
    xyzB =xyzBh(1:3,:);
    xyzB(:,extraLegs) = xyzExtra;

    
    currentStanceLegs = stanceLegs(~swingAtPhasesToTest(k,:));
    thIK = kin.getIK(xyzB);
    bodyCoM = kin.getSnakeMonsterCoM(thIK);
       % project it onto the plane:
    dp = TP_B(1:3,4) - bodyCoM;% difference between origins of plane and point
    planeNormal = TP_B(1:3,3); % z direction on plane frame viewed from body frame
    temp = sum(planeNormal.*dp)/sum(planeNormal.^2);
    projBodyCoM = bodyCoM + temp*planeNormal;% add relative difference to project point back to plane
    % projBodyCoM is in body frame

    % find the polygon and the distances in this rotated plane
    xyzContactRot = xyz(:,currentStanceLegs);
    K = convhull(xyzContactRot(1:2,:).');
    xOrderedRot = xyzContactRot(:,K.');
%     xOrdered = TP_B*[xOrderedRot; ones(1, size(xOrderedRot,2))];
    %     xCentroid = mean(xOrdered(:,1:end-1),2);
    projBodyCoMRot = TB_P*[projBodyCoM;1];
    distToLine = p_poly_dist(projBodyCoMRot(1),projBodyCoMRot(2), xOrderedRot(1,:) ,xOrderedRot(2,:),  1);
    
    ineqViolationsMat1(k) = distToLine*100;
    
    
    % make sure the legs don't overlap
    % penalize having a foot in front of another during the gait
    xyStepT = reshape(xyz(1:2,stanceLegs), [1,2*nStanceLegs]);
    footOverlap= A(1:length(oddInds)-1 + length(evenInds)-1 ,:)...
        *[xyStepT zeros(1,5*nPhases)].';
    ineqViolationsMat3(:,k) = footOverlap + .05; % min foot separation
    
    % disallow the feet to go over the body.
    ineqViolationsMat4(:,k) = xyzBContact(3,:).' +.08; % min foot height
    
%     % disallow the feet to go under the body?
%     oddInds = find(isOdd(stanceLegs));
%     evenInds = find(~isOdd(stanceLegs));
%     ineqViolationsMat5(oddInds,k) = xyzBContact(1,oddInds).' +.1;
%     ineqViolationsMat5(evenInds,k) = .1 - xyzBContact(1,evenInds).';
    
% force the FK to match the xyzB desired points, within a centimeter
    thIK = kin.getIK(xyzBh(1:3,:));
    xyzFK = kin.getLegPositions(thIK);
   errs = xyzBContact - xyzFK(:,stanceLegs);
  ineqViolationsMat5(:,k) = sqrt(sum(errs.^2)) - .01;
    
%   plt.plot(thIK);
end


ineqViolations = [ineqViolationsMat1(:).' ineqViolationsMat2(:).'...
    ineqViolationsMat3(:).' ineqViolationsMat4(:).' ineqViolationsMat5(:).'];


eqViolations= [];

end
