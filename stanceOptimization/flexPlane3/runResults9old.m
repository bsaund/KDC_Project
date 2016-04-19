% plotOptResultsStatic
addpath(genpath('C:\Users\Julian\Box Sync\CMU sem 1+2\snakeMonster\KDC_project'));
addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));

sendCommands = 0;

nCycles = 1;

% initialize robot
% command structure for if sending commands
if sendCommands
    cmd = CommandStruct();
cmd.position = nan(1,18);
cmd.velocity = nan(1,18);
cmd.torque = nan(1,18);
setupSnakeMonsterGroup; % makes the snakeMonster hebi group object
    setGainsSMHigh;
nCycles = 2;
end

close all;
     plt = SnakeMonsterPlotter(); 
     set(gcf, 'position', [10 100 700 700]);  
plt.plot(zeros(1,18)); hold on;
projectedCOM = scatter3(0,0,0,'k', 'filled'); 
supportLines = plot3(0,0,0,'k');
planeArrow = quiver3(0,0,0,0,0,0,'b');

     kin = SnakeMonsterKinematics();
     
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
if sendCommands
    nWaypoints = 50;
end

% t_span1 = linspace(0,2*pi*nCycles,5)+.01;
% t_span2 = linspace(0,2*pi*nCycles,5)-.01;
% t_span = zeros(1,length(t_span1)*2);
% t_span(1:2:end) = t_span1;
% t_span(2:2:end) = t_span2;


dt = (2*pi*nCycles-0)/(nWaypoints*nCycles-1);

stepPeriod = 2*pi*fractionStep;  % how long the stepping lasts
% solve for coefficients to create trajectory with min jerk
jerkCoeffs = minimumJerk( 0, 0, 0, ... % Starting Phase/Vel/Accel
    1, 0, 0, ... % Ending Phase/Vel/Accel
    stepPeriod);  % Time to touchdown
stepWayPoints = [-a_back*stepDirVector.';  0 0 b; a_forward*stepDirVector.'];

transformsMat = reshape(transforms, [5 nPhases]);

timeInt = phasesToTest(:,1).';
% timeInt(end) = 2*pi;
timeInt(1:2:end) = timeInt(1:2:end)+.001; % just took off
timeInt(2:2:end) = timeInt(2:2:end)-.001; % about to land
t_span = timeInt;
% t_span = linspace(0,2*pi*nCycles,nWaypoints*nCycles);


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
       transformNow = interp1(timeInt,transformsMat(:,[1:end]).',...
           t, 'spline').';
%        transformNow = interp1((0:2*pi/nStanceLegs:2*pi),transformsMat(:,[1:end, 1]).',...
%            mod(t,2*pi)).';
   
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
    dp = TP_B(1:3,4) - bodyCoM;% difference between origins of plane and point
    planeNormal = TP_B(1:3,3); % z direction on plane frame viewed from body frame
    temp = sum(planeNormal.*dp)/sum(planeNormal.^2);
    projBodyCoM = bodyCoM + temp*planeNormal;% add relative difference to project point back to plane
    % projBodyCoM is in body frame
    
     % find the polygon and the distances in this rotated plane
         currentStanceLegs = stanceLegs(~swingLegs(stanceLegs));
    xyzContactRot = xyz(:,currentStanceLegs); % contact in plane
    K = convhull(xyzContactRot(1:2,:).');
    xOrderedRot = xyzContactRot(:,K.');
    %     xCentroid = mean(xOrdered(:,1:end-1),2);
    projBodyCoMRot = TB_P*[projBodyCoM;1];
    
      plt.plot(thIK); hold on;
  set(supportLines, 'xdata', xOrdered(1,:), 'ydata',xOrdered(2,:), ...
     'zdata', xOrdered(3,:));
  inSupport = inpolygon(projBodyCoMRot(1),projBodyCoMRot(2),xOrderedRot(1,:),xOrderedRot(2,:));
  set(projectedCOM, 'xdata', projBodyCoM(1), 'ydata',projBodyCoM(2), 'zdata', projBodyCoM(3),...
     'markerFaceColor', [1 0 0]*~inSupport);
%   set(scatterCentroid, 'xdata', xCentroid(1), 'ydata',xCentroid(2), 'zdata',xCentroid(3));
 set(planeArrow,...
     'udata', TP_B(1,3)*.1,'vdata', TP_B(2,3)*.1,'wdata', TP_B(3,3)*.1,...
     'xdata', TP_B(1,4),'ydata', TP_B(2,4),'zdata', TP_B(3,4));
 
% scatter3(xyzFK(1,:), xyzFK(2,:), xyzFK(3,:), 'r');
%                 scatter3(xyzB(1,:), xyzB(2,:), xyzB(3,:), [], swingLegs, 'filled');
                
  if ~inSupport
     disp('out') 
  end
%   
%   if makeVideo
%      writeVideo(v,getframe);
%       
%   end
  

   if sendCommands
        cmd.position = [reshape(thIK,[1,18])];
        snakeMonster.set(cmd);
    end
  
end

%{
clawAngle = [];
    cmd.position = nan(1,18+length(clawAngle));
    cmd.velocity = nan(1,18+length(clawAngle));
    cmd.torque = nan(1,18+length(clawAngle));
    snakeMonster.set(cmd);
%}