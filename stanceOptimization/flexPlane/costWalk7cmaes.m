function f = costWalk7cmaes(state)

global kin xyzExtra stanceLegs extraLegs  plt A evals nLegs
global stepDirection phasesToTest swingAtPhasesToTest stepOrder stepLength
nStanceLegs = length(stanceLegs);

xyStep = state(1:2*nStanceLegs);
transforms = state(2*nStanceLegs+1:end);

xyz = zeros(3,6);
xyz(1:2,stanceLegs) = reshape(xyStep, [2,nStanceLegs]);
% foot positions on the plane are zero.
xyz(3,stanceLegs) = 0;

% get the forwards direction vector
stepDirVector = R_z(stepDirection)*[0;1;0];


%% For each of the phasesToTest evaluate a cost of projectedCoM 
% Harshly penalize being outside the polygon

a_forward = stepLength/2; % params.L/3
a_back = stepLength/2; % step length = a_forward  + a_back
% b = .06; % step height = b. .06 works.
fractionStep = 1/nStanceLegs;
nPhases = size(phasesToTest,1);
costPhases = zeros(1,nPhases);
xyz0 = xyz;
% make pattern:  1     2     2     3     3     4     4     5     5     1
transformPhaseOrder = circshift(ceil((1:nPhases)/2),[1 -1]);
transformsMat = reshape(transforms, [5 nPhases/2]);

for k = 1:nPhases
    
    % current xyz: move feet to position in the phase. In plane.
    for i = 1:nStanceLegs
        leg = stepOrder(i);
        xyz(:,leg) = a_forward*stepDirVector - ...
            (a_forward + a_back)*stepDirVector*...
            (phasesToTest(k,i)/(2*pi - 2*pi*fractionStep) - 2*pi*fractionStep/(2*pi - 2*pi*fractionStep)).'...
            + xyz0(:,leg);
    end
    
    % find the positions of the feet in the body frame.
    % transformation of Body viewed from Plane
    rB_P = transformsMat(3:end, transformPhaseOrder(k)); % vector of B wrt P,
    % (body frame wrt plane origin)
    thetaX = transformsMat(1, transformPhaseOrder(k));
    thetaY = transformsMat(2, transformPhaseOrder(k));
    RB_P = R_y(thetaY)*R_x(thetaX); % rotation of the frame B wrt P frame
    % (body frame coordinate system as viewed in plane frame)
    TB_P = [RB_P rB_P; 0 0 0 1]; % transformation to the body frame from the plane frame
    TP_B = inv(TB_P);
    xyzBh = TP_B*[xyz; ones(1,nLegs)];
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
    %     xCentroid = mean(xOrdered(:,1:end-1),2);
    projBodyCoMRot = TB_P*[projBodyCoM;1];
    distToLine = p_poly_dist(projBodyCoMRot(1),projBodyCoMRot(2), xOrderedRot(1,:) ,xOrderedRot(2,:),  1);
    
    % add this cost to the cost matrix, which will be summed at the end
    costPhases(k) = heaviside((distToLine+.02))*((distToLine+.02)*500)^2; % ?
    
    
    
    % discourage the joints from coming too close together.
    CoMs = kin.getCenterOfMasses(thIK);
    %  http://stackoverflow.com/questions/19360047/how-to-build-a-distance-matrix-without-loop
    CoMsInterest = CoMs(:,:,[2 4 6]);
    sizeCoMs = size(CoMsInterest);
    x = reshape(permute(CoMsInterest, [3, 2, 1]), [sizeCoMs(2)*sizeCoMs(3),sizeCoMs(1)]).'; % x;y;z for all points
    IP = x' * x;
    dEven = sqrt(bsxfun(@plus, diag(IP), diag(IP)') - 2 * IP); % matrix with distance from point i to point j
    % includes the distance to i to i.
    pointDistCostEven =heaviside(-(dEven-.1)).*10000.*(dEven-.1).^2;
    pointDistCostEven(1:sizeCoMs(3)*sizeCoMs(2)+1:end) = 0; % don't penalize the link being itself!
    costPhases(k) =  costPhases(k) ...
        + sum(sum(pointDistCostEven))*300;
    % odd side:
    %  CoMsInterest = CoMs(:,2:end-1,[1 3 5]); % I only care about collisions near center of leg
    CoMsInterest = CoMs(:,:,[1 3 5]); %
    sizeCoMs = size(CoMsInterest);
    x = reshape(permute(CoMsInterest, [3, 2, 1]), [sizeCoMs(2)*sizeCoMs(3),sizeCoMs(1)]).'; % x;y;z for all points
    IP = x' * x;
    dOdd = sqrt(bsxfun(@plus, diag(IP), diag(IP)') - 2 * IP); % matrix with distance from point i to point j
    % includes the distance to i to i.
    pointDistCostOdd =heaviside(-(dOdd-.1)).*10000.*(dOdd-.1).^2;
    pointDistCostOdd(1:sizeCoMs(3)*sizeCoMs(2)+1:end) = 0; % don't penalize the link being itself!
    costPhases(k) =  costPhases(k) ...
        + sum(sum(pointDistCostOdd))*300;
    
    
    
%     % penalize having a foot in front of another during the gait
%     xyStepT = reshape(xyz(1:2,stanceLegs), [1,2*nStanceLegs]);
%     footOverlap= A*[xyStepT zeros(1,5*nPhases/2)].';
%     costPhases(k) =  costPhases(k) ...
%         + sum(exp((footOverlap+.01)*100))*100;
    
%     plt.plot(thIK);


% nonlinear parts turned into costs:
% FK matching
  xyzFK = kin.getLegPositions(thIK);
   errs = xyzB(:,stanceLegs) - xyzFK(:,stanceLegs);
   costPhases(k) =  costPhases(k) + sum(sum(errs.^2))*10000;
    
      % make sure the legs don't overlap
    % penalize having a foot in front of another during the gait
    xyStepT = reshape(xyz(1:2,stanceLegs), [1,2*nStanceLegs]);
    footOverlap= A*[xyStepT zeros(1,5*nPhases/2)].';
    costPhases(k) =  costPhases(k) + sum(footOverlap)*10000;
    
   
end
% plt.plot(thIK);
f = sum(costPhases);

evals = evals + 1;

if ~mod(evals, 300)
    plt.plot(thIK);
end


end

