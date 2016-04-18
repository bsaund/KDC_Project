% runGait
% takes the parameters from the gait optimization and plays them out.

makeVideo = 0; % video recording flag

nCycles = 1;
nWaypoints = 100;
t_span = linspace(0,2*pi*nCycles,nWaypoints*nCycles);
dt = (2*pi*nCycles-0)/(nWaypoints*nCycles-1);
a_forward = stepLength/2; % 
a_back = stepLength/2; % step length = a_forward  + a_back
b = .06; % step height = b. .06 works. 
 
stepPeriod = 2*pi*fractionStep;  % how long the stepping lasts
% solve for coefficients to create trajectory with min jerk
jerkCoeffs = minimumJerk( 0, 0, 0, ... % Starting Phase/Vel/Accel
    1, 0, 0, ... % Ending Phase/Vel/Accel
    stepPeriod);  % Time to touchdown
% take the forwards direction, project onto the plane.
stepDirVector = R_z(stepDirection)*[0;1;0];
planeNormal = [planexyc(1); planexyc(2); 1];
stepDirOnPlane = stepDirVector - dot(stepDirVector, planeNormal)/dot(planeNormal, planeNormal) * planeNormal;
stepDirOnPlane = stepDirOnPlane/norm(stepDirOnPlane); % normalize to length one
%  stepWayPoints = [0 -a_back 0; 0 0 b; 0 a_forward 0]; % determine which points the feet pass through in plane
stepWayPoints = [-a_back*stepDirOnPlane.';  0 0 b; a_forward*stepDirOnPlane.'];

[n_theta] = vrrotvec([0;0;1], planeNormal);
  RFromPlane = rotmat(n_theta(1:3).', n_theta(end));
fractionStep = 1/nStanceLegs;
swingLegs = zeros(1,6); % 1 indicates leg is in the air
swingLegs(extraLegs) = 2;
 xyz(1:2,stanceLegs) = reshape(xyStepOpt, [2,nStanceLegs]); 
% set the feet z positions to be the projection on to the plane
% planex*x + planey*y + z +planec = 0
% z = -planex*x - planey*y -planec
xyz(3,stanceLegs) = ...
    - xyz(1,stanceLegs)*planexyc(1) - xyz(2,stanceLegs)*planexyc(2) - planexyc(3);
xyzStance0 = xyz; % save original configuration for modification

 if makeVideo
     v = VideoWriter(['fourLegLongStep4_7_1pm.avi']);
      open(v);
  end

for t = t_span
% get the foot placement points
    t_leg = t+2*pi/nStanceLegs*(nStanceLegs:-1:1);
    for i = 1:nStanceLegs
        leg = stepOrder(i);
        xyz(:,leg) = minJerkStepGait3(stepWayPoints, jerkCoeffs, xyzStance0(:,leg), RFromPlane, fractionStep, t_leg(i));
        
        if (mod(t_leg(i), 2*pi)<2*pi*fractionStep)
            swingLegs(leg) = 1;
        else
            swingLegs(leg) = 0;
        end
    end
    
    % plot this new config
    thIK = kin.getIK(xyz);
xyzFK = kin.getLegPositions(thIK);
% %  the support polygon
 xyzContact = xyzFK(:,~swingLegs);
 K = convhull(xyzContact(1:2,:).');
 xOrdered = xyzContact(:,K.');
 xCentroid = mean(xOrdered(:,1:end-1),2);
 
     bodyCoM = kin.getSnakeMonsterCoM(thIK);
     
     % adjust CoM to take an extra weight 
fullCoM = (bodyCoM*(sum(sum(masses))+2.37) + extraMass*extraMassXYZ )...
    /(sum(sum(masses))+2.37 + extraMass);
     
    % project it onto the plane:
    dp = [0; 0; -planexyc(3)] - fullCoM;% difference between origins of plane and point
%     planeNormal = [planexyc(1); planexyc(2); 1];% relative position of point on normal's line
    temp = sum(planeNormal.*dp)/sum(planeNormal.^2);
    projBodyCoM = fullCoM + temp*planeNormal;% add relative difference to project point back to plane
 
   [n_theta] = vrrotvec(planeNormal,[0;0;1]);
    R = rotmat(n_theta(1:3).', n_theta(end));
    % find the polygon and the distances in this rotated plane
    currentStanceLegs = stanceLegs(~swingLegs(stanceLegs));
    xyzContactRot = R*xyz(:,currentStanceLegs);
    K = convhull(xyzContactRot(1:2,:).');
    xOrderedRot = xyzContactRot(:,K.');
%     xCentroid = mean(xOrdered(:,1:end-1),2);
    projBodyCoMRot = R*projBodyCoM;
    
      plt.plot(thIK); hold on;
  set(supportLines, 'xdata', xOrdered(1,:), 'ydata',xOrdered(2,:), ...
     'zdata', xOrdered(3,:));
  inSupport = inpolygon(projBodyCoMRot(1),projBodyCoMRot(2),xOrderedRot(1,:),xOrderedRot(2,:));
  set(projectedCOM, 'xdata', projBodyCoM(1), 'ydata',projBodyCoM(2), 'zdata', projBodyCoM(3),...
     'markerFaceColor', [1 0 0]*~inSupport);
%   set(scatterCentroid, 'xdata', xCentroid(1), 'ydata',xCentroid(2), 'zdata',xCentroid(3));
                scatter3(xyzFK(1,:), xyzFK(2,:), xyzFK(3,:), 'r');
                scatter3(xyz(1,:), xyz(2,:), xyz(3,:), [], swingLegs, 'filled');
  
  if ~inSupport
     disp('out') 
  end
  
  if makeVideo
     writeVideo(v,getframe);
      
  end
  
  
end
 if makeVideo
   close(v)  
      
  end
