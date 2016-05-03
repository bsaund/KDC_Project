function f = costWalk10cmaes(state)

global kin xyzExtra stanceLegs extraLegs  plt A evals nLegs
global stepDirection phasesToTest swingAtPhasesToTest stepOrder stepLength
nStanceLegs = length(stanceLegs);

xyStep = state(1:2*nStanceLegs);
transforms = state(2*nStanceLegs+1:end);

xyz = zeros(3,6);
% xyz(:,extraLegs) =xyzExtra;
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
costPhases = zeros(1,nPhases);
xyz0 = xyz;
oddInds =find(mod(stanceLegs,2)==1); % the indexes of the odd legs in stanceLegs
evenInds= find(mod(stanceLegs,2)==0);
% turn transforms into a matrix that is the number of phases

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
    xyzB =xyzBh(1:3,:);
    xyzB(:,extraLegs) = xyzExtra;
       xyzBContact =xyzBh(1:3,stanceLegs);
    
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
    xOrdered = TP_B*[xOrderedRot; ones(1, size(xOrderedRot,2))];
    %     xCentroid = mean(xOrdered(:,1:end-1),2);
    projBodyCoMRot = TB_P*[projBodyCoM;1];
    distToLine = p_poly_dist(projBodyCoMRot(1),projBodyCoMRot(2), xOrderedRot(1,:) ,xOrderedRot(2,:),  1);
    
    % add this cost to the cost matrix, which will be summed at the end
 costPhases(k) = heaviside((distToLine+.06))*((distToLine+.06)*500)^2; % ?  

    % discourage the joints from coming too close together.
    CoMs = kin.getCenterOfMasses(thIK);
    %  http://stackoverflow.com/questions/19360047/how-to-build-a-distance-matrix-without-loop
%     CoMsInterest = CoMs(:,:,stanceLegs(~isOdd(stanceLegs)));    
    CoMsInterest = CoMs(:,:,[2 4 6]);
    sizeCoMs = size(CoMsInterest);
    x = reshape(permute(CoMsInterest, [3, 2, 1]), [sizeCoMs(2)*sizeCoMs(3),sizeCoMs(1)]).'; % x;y;z for all points
    IP = x' * x;
    dEven = sqrt(bsxfun(@plus, diag(IP), diag(IP)') - 2 * IP); % matrix with distance from point i to point j
    % includes the distance to i to i.
    pointDistCostEven =heaviside(-(dEven-.07)).*500.*(dEven-.07).^2;
    pointDistCostEven(1:sizeCoMs(3)*sizeCoMs(2)+1:end) = 0; % don't penalize the link being itself!
    costPhases(k) =  costPhases(k) ...
        + sum(sum(pointDistCostEven));
    % odd side:
    %  CoMsInterest = CoMs(:,2:end-1,[1 3 5]); % I only care about collisions near center of leg
%     CoMsInterest = CoMs(:,:,stanceLegs(isOdd(stanceLegs)));
    CoMsInterest = CoMs(:,:,[1 3 5]);
    sizeCoMs = size(CoMsInterest);
    x = reshape(permute(CoMsInterest, [3, 2, 1]), [sizeCoMs(2)*sizeCoMs(3),sizeCoMs(1)]).'; % x;y;z for all points
    IP = x' * x;
    dOdd = sqrt(bsxfun(@plus, diag(IP), diag(IP)') - 2 * IP); % matrix with distance from point i to point j
    % includes the distance to i to i.
    pointDistCostOdd =heaviside(-(dOdd-.07)).*500.*(dOdd-.07).^2;
    pointDistCostOdd(1:sizeCoMs(3)*sizeCoMs(2)+1:end) = 0; % don't penalize the link being itself!
    costPhases(k) =  costPhases(k) ...
        + sum(sum(pointDistCostOdd));
    
    


    % make sure the legs don't overlap
    % penalize having a foot in front of another during the gait
    xyStepT = reshape(xyz(1:2,stanceLegs), [1,2*nStanceLegs]);
    footOverlap= A(1:length(oddInds)-1 + length(evenInds)-1 ,:)...
        *[xyStepT zeros(1,5*nPhases)].';
    if any((footOverlap > -.05))
        costPhases(k) = NaN; % min foot separation 
    end
    
    % force the FK to match the xyzB desired points, within a centimeter
    xyzFK = kin.getLegPositions(thIK);
    errs = xyzBContact - xyzFK(:,stanceLegs);
    if any(sqrt(sum(errs.^2)) > .01)
        costPhases(k) = NaN; 
    end
    
    
end
% 
f = sum(costPhases);

% add a small cost for big deviations from the center, but not z.
f = f + sum(sum(transformsMat(1:4,:).^2))*50;

% add a cost to large differences in transforms
transitionDiffs =  diff([transformsMat, transformsMat(:,1)],1,2);
f = f+ sum(sum(transitionDiffs.^2))*200; % *10 worked well. 200 good too.


evals = evals + 1;

if ~mod(evals, 300)
    plt.plot(thIK);
end


end
