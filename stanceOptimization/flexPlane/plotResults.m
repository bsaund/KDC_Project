% plotOptResultsStatic

close all;
     plt = SnakeMonsterPlotter(); 
     kin = SnakeMonsterKinematics();
     
nCycles = 1;
nStanceLegs = length(stanceLegs);
xyStep = stateOpt(1:2*nStanceLegs);
transforms = stateOpt(2*nStanceLegs+1:end);

xyz = zeros(3,6);
xyz(:,extraLegs) =xyzExtra;
xyz(1:2,stanceLegs) = reshape(xyStep, [2,nStanceLegs]);
% foot positions on the plane are zero.
xyz(3,stanceLegs) = 0;
xyzStance0 = xyz; % save original configuration for modification

% get the forwards direction vector
stepDirVector = R_z(stepDirection)*[0;1;0];
a_forward = stepLength/2; % params.L/3
a_back = stepLength/2; % step length = a_forward  + a_back
b = .06; % step height = b. .06 works.
fractionStep = 1/nStanceLegs;
swingLegs = zeros(1,6); % 1 indicates leg is in the air
swingLegs(extraLegs) = 2;

nWaypoints = 100;
t_span = linspace(0,2*pi*nCycles,nWaypoints*nCycles);
dt = (2*pi*nCycles-0)/(nWaypoints*nCycles-1);

stepPeriod = 2*pi*fractionStep;  % how long the stepping lasts
% solve for coefficients to create trajectory with min jerk
jerkCoeffs = minimumJerk( 0, 0, 0, ... % Starting Phase/Vel/Accel
    1, 0, 0, ... % Ending Phase/Vel/Accel
    stepPeriod);  % Time to touchdown
stepWayPoints = [-a_back*stepDirVector.';  0 0 b; a_forward*stepDirVector.'];

% make pattern:  1     2     2     3     3     4     4     5     5     1
transformPhaseOrder = circshift(ceil((1:nPhases)/2),[1 -1]);
transformsMat = reshape(transforms, [5 nPhases/2]);

set(gcf, 'position', [10 100 500 500]);  
plt.plot(zeros(1,18)); hold on;
projectedCOM = scatter3(0,0,0,'k', 'filled'); 
supportLines = plot3(0,0,0,'k');


for t = t_span
% get the foot placement points in plane
    t_leg = t+2*pi/nStanceLegs*(nStanceLegs:-1:1);
    for i = 1:nStanceLegs
        leg = stepOrder(i);
        xyz(:,leg) = minJerkStepGait3(stepWayPoints, jerkCoeffs, xyzStance0(:,leg), eye(3), fractionStep, t_leg(i));
        
        if (mod(t_leg(i), 2*pi)<2*pi*fractionStep)
            swingLegs(leg) = 1;
        else
            swingLegs(leg) = 0;
        end
    end
    
    % transform points to the Body frame
    % use interpolation from matrix
       transformNow = interp1((0:2*pi/nStanceLegs:2*pi),transformsMat(:,[1:end, 1]).',...
           mod(t,2*pi), 'spline').';

   
    % find the positions of the feet in the body frame.
    % transformation of Body viewed from Plane
    rB_P = transformNow(3:end); % vector of B wrt P,
    % (body frame wrt plane origin)
    thetaX = transformNow(1);
    thetaY = transformNow(2);
    RB_P = R_y(thetaY)*R_x(thetaX); % rotation of the frame B wrt P frame
    % (body frame coordinate system as viewed in plane frame)
    TB_P = [RB_P rB_P; 0 0 0 1]; % transformation to the body frame from the plane frame
    TP_B = inv(TB_P);
    xyzBh = TP_B*[xyz; ones(1,nLegs)];
    xyzB =xyzBh(1:3,:);
    xyzB(:,extraLegs) = xyzExtra;
    
    % plot this new config
    thIK = kin.getIK(xyzB);
xyzFK = kin.getLegPositions(thIK);
% %  the support polygon
 xyzContact = xyzFK(:,~swingLegs);
 K = convhull(xyzContact(1:2,:).');
 xOrdered = xyzContact(:,K.');
 xCentroid = mean(xOrdered(:,1:end-1),2);
 
     bodyCoM = kin.getSnakeMonsterCoM(thIK);
     
     % project it onto the plane:
    dp = -rB_P - bodyCoM;% difference between origins of plane and point
    planeNormal = TP_B(1:3,3); % z direction on plane frame viewed from body frame
    temp = sum(planeNormal.*dp)/sum(planeNormal.^2);
    projBodyCoM = bodyCoM + temp*planeNormal;% add relative difference to project point back to plane
    % projBodyCoM is in body frame
    
     % find the polygon and the distances in this rotated plane
         currentStanceLegs = stanceLegs(~swingLegs(stanceLegs));
    xyzContactRot = xyz(:,currentStanceLegs);
    K = convhull(xyzContactRot(1:2,:).');
    xOrderedRot = xyzContactRot(:,K.');
    %     xCentroid = mean(xOrdered(:,1:end-1),2);
    projBodyCoMRot = RB_P\projBodyCoM;
    
      plt.plot(thIK); hold on;
  set(supportLines, 'xdata', xOrdered(1,:), 'ydata',xOrdered(2,:), ...
     'zdata', xOrdered(3,:));
  inSupport = inpolygon(projBodyCoMRot(1),projBodyCoMRot(2),xOrderedRot(1,:),xOrderedRot(2,:));
  set(projectedCOM, 'xdata', projBodyCoM(1), 'ydata',projBodyCoM(2), 'zdata', projBodyCoM(3),...
     'markerFaceColor', [1 0 0]*~inSupport);
%   set(scatterCentroid, 'xdata', xCentroid(1), 'ydata',xCentroid(2), 'zdata',xCentroid(3));
                scatter3(xyzFK(1,:), xyzFK(2,:), xyzFK(3,:), 'r');
                scatter3(xyzB(1,:), xyzB(2,:), xyzB(3,:), [], swingLegs, 'filled');
  
%   if ~inSupport
%      disp('out') 
%   end
%   
%   if makeVideo
%      writeVideo(v,getframe);
%       
%   end
  
  
end

