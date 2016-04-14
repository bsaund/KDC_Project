function  [ineqViolations,eqViolations]=nonlinconWalk7(state)
% nonlinear constriants
% C(X) <= 0
global  xyzExtra stanceLegs extraLegs   stanceLegBaseXY A kin
global stepDirection phasesToTest stepOrder stepLength nLegs
nStanceLegs = length(stanceLegs);

xyStep = state(1:2*nStanceLegs);
transforms = state(2*nStanceLegs+1:end);

xyz = zeros(3,6);
xyz(:,extraLegs) =xyzExtra;
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
% b = .06; % step height = b. .06 works.
fractionStep = 1/nStanceLegs;
nPhases = size(phasesToTest,1);

% ineqViolationsMat1 = zeros(nStanceLegs,nPhases/2); % for max radius
% ineqViolationsMat2 = zeros(nStanceLegs,nPhases/2); % for min radius
ineqViolationsMat1 =[]; ineqViolationsMat2=[];

ineqViolationsMat3 = zeros(size(A,1), nPhases/2); % for foot overlap
ineqViolationsMat4 = zeros(nStanceLegs ,nPhases/2); % for max foot height
ineqViolationsMat5 = zeros(nStanceLegs ,nPhases/2); % for FK matching

xyz0 = xyz;
% make pattern:  1     2     2     3     3     4     4     5     5     1
transformPhaseOrder = circshift(ceil((1:nPhases)/2),[1 -1]);
transformsMat = reshape(transforms, [5 nPhases/2]);

Rmax = .2;
Rmin = .1;

for k = 1:nPhases/2
    
    % current xyz: move feet to position in the phase. In plane.
    for i = 1:nStanceLegs
        leg = stepOrder(i);
        xyz(:,leg) = a_forward*stepDirVector - ...
            (a_forward + a_back)*stepDirVector*...
            (phasesToTest(k*2-1,i)/(2*pi - 2*pi*fractionStep) - 2*pi*fractionStep/(2*pi - 2*pi*fractionStep)).'...
            + xyz0(:,leg);
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
    
    % % make sure the foot posiitons are in a sphere around the joint
% %     for j = 1:nStanceLegs
% %         dFromBody = xyzBContact(:,j) - [stanceLegBaseXY(:,j); 0 ];
% %         % make sure the xyz positions stay within a circle of the base
% %         % dist <= Rmax --> dist - Rmax <=0
% %         ineqViolationsMat1(j,k) =...
% %             dFromBody.'*dFromBody - Rmax^2;
% %         % make sure the xyz positions are not too close
% %         % % dist >= Rmin --> Rmin - dist <=0
% %         ineqViolationsMat2(j,k) =...
% %             Rmin^2 -  dFromBody.'*dFromBody;
% %     end
    
    % make sure the legs don't overlap
    % penalize having a foot in front of another during the gait
    xyStepT = reshape(xyz(1:2,stanceLegs), [1,2*nStanceLegs]);
    footOverlap= A*[xyStepT zeros(1,5*nPhases/2)].';
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
    
end


ineqViolations = [ineqViolationsMat1(:).' ineqViolationsMat2(:).'...
    ineqViolationsMat3(:).' ineqViolationsMat4(:).' ineqViolationsMat5(:).'];


eqViolations= [];

end
