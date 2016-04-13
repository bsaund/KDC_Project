% generalWalking
% load a previously made gait stance and parameters, and walk with it based
% on those parameters and inupt from joystick
% including some flag to tell it if its stepping or not.
% this will only work for +/- the stepDirection that the stance was
% optimized for. Other directions will probably fall over.

% load previous parameters
sendCommands = 1; % 1 to turn on commands to real robot
logging = 0; % Writes a hebi log
plotting = 0;
withClaw = 1;
close all; clc;
addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));
joy = vrjoystick(1);

% initialize robot
% command structure for if sending commands
cmd = CommandStruct();
cmd.position = nan(1,18);
cmd.velocity = nan(1,18);
cmd.torque = nan(1,18);
if sendCommands
    if withClaw
    setupSnakeMonsterGroupClaw; % makes the snakeMonster hebi group object
    else
        setupSnakeMonsterGroup; % makes the snakeMonster hebi group object
    end
    setGainsSM;
    if logging
        snakeMonster.startLog();
    end
end

% testing: start with a previously found stance
% load('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project\stanceOptimization\findXYZOptWalkTilt\fourLegOk4_11_4pm.mat');
% load('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project\stanceOptimization\findXYZOptWalkTilt\fiveLegGreat4_6_10am.mat');
% load('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project\stanceOptimization\findXYZOptWalkTilt\sixLegGood4_6_10am.mat');
load('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project\stanceOptimization\findXYZOptWalkTilt\fiveLegLongStep4_12_3pm.mat');


    if withClaw
        plt = SnakeMonsterPlotter('gripper', 1);
    else
        plt = SnakeMonsterPlotter;
    end


kin = SnakeMonsterKinematics(); % need to remake kin object
masses = kin.getLegMasses();
xyz(:,extraLegs) = xyzExtra;
xyz(1:2,stanceLegs) = reshape(xyStepOpt, [2,nStanceLegs]);
xyz(3,stanceLegs) = ...
    - xyz(1,stanceLegs)*planexyc(1) - xyz(2,stanceLegs)*planexyc(2) - planexyc(3);
xyzBase = xyz;
thFinal = kin.getIK(xyz);

%% make all of the gait variables
aForward = stepLength/2; %
aBack = stepLength/2; % step length = a_forward  + a_back
aBackBase = aBack;
aForwardBase = aForward;
% stepHeight = .06; % step height = b. .06 works.
stepHeight = .08; % step height = b. .06 works.

legPhaseDiffs = 2*pi/nStanceLegs*(nStanceLegs:-1:1);
fractionStep = 1/nStanceLegs;
% alternating tripod stuff
if nStanceLegs ==6
    legPhaseDiffs = pi*[1 0 0 1 1 0];
    fractionStep = 1/2;
    stepOrder = 1:6;
end

stepPeriod = 2*pi*fractionStep;  % how long the stepping lasts
% stepPeriod = .6;  % how long the stepping lasts
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
stepWayPoints = [-aBack*stepDirOnPlane.';  0 0 stepHeight; aForward*stepDirOnPlane.'];
[n_theta] = vrrotvec([0;0;1], planeNormal);
RFromPlane = rotmat(n_theta(1:3).', n_theta(end));
swingLegs = zeros(1,6); % 1 indicates leg is in the air
swingLegs(extraLegs) = 2;
gravity = -9.81*planeNormal/norm(planeNormal);


% get intitial leg pose
t=0;
t_leg = t+legPhaseDiffs;
for i = 1:nStanceLegs
    leg = stepOrder(i);
    xyz(:,leg) = minJerkStepGait3(stepWayPoints, jerkCoeffs, xyzStance0(:,leg), RFromPlane, fractionStep, t_leg(i));
end
thIK = kin.getIK(xyz);
% bodyMass = (params.robotMass +sum(sum(masses)));
bodyMass = (params.robotMass);
    
if withClaw
    clawAngle = 0;
else
    clawAngle = [];
end

  plt.plot([thIK clawAngle]); hold on;
if plotting
    
    % plotting start up
    projectedCOM = scatter3(0,0,0,'k', 'filled');
    supportLines = plot3(0,0,0,'k');
end

% flags
running = 1; % flag for running
stepping = 0; % flag for currently taking a step
direction = 1; % 1= forwards direction. -1 = backwards.
% dt = pi*fractionStep/10;
stickDeadZone = .08;

tic; % start clock
tocLast = 0;

while running
    
    tocNow = toc;   % get elapsed time now
    dt = tocNow - tocLast;  % period of while loop [s]
    
    % read joystick
    [swivel, buttons, povs] = read( joy );
        modifiedStickVal = swivel;
        modifiedStickVal(abs(modifiedStickVal)<stickDeadZone) = 0;  % set values in deadzone to 0
%         modifiedStickVal = sign(modifiedStickVal) .* modifiedStickVal.^2; % squares stick signal for enhanced control
        % axes(1) is the forwards/back (-1 -> 1)
        % axes(2) is the left/right (-1 -> 1)
        stepDirection = -atan2(modifiedStickVal(1), -modifiedStickVal(2));
        % make sure step direction is only in the first sphere
          stepDirection(stepDirection>pi/2) = stepDirection - pi;
         stepDirection(stepDirection<-pi/2) = stepDirection + pi;
        aBack = aBackBase * sqrt(sum(modifiedStickVal(1:2).^2)/2);
        aForward = aForwardBase * sqrt(sum(modifiedStickVal(1:2).^2)/2);
%         disp(aForward+aBack)
    if modifiedStickVal(2)<-.02
        stepping = 1;
        direction = 1;
    elseif modifiedStickVal(2)>.02
        stepping = 1;
        direction = -1;
    else
        stepping = 0;
    end
    
    
    
    if sendCommands
    % get feedback
    fbk = snakeMonster.getNextFeedback();
        thLast = fbk.position(1:end-length(clawAngle));
    else
        thLast = thIK;
    end
    
    stepDirVector = R_z(stepDirection)*[0;1;0];
planeNormal = [planexyc(1); planexyc(2); 1];
stepDirOnPlane = stepDirVector - dot(stepDirVector, planeNormal)/dot(planeNormal, planeNormal) * planeNormal;
stepDirOnPlane = stepDirOnPlane/norm(stepDirOnPlane); % normalize to length one
%  stepWayPoints = [0 -a_back 0; 0 0 b; 0 a_forward 0]; % determine which points the feet pass through in plane
stepWayPoints = [-aBack*stepDirOnPlane.';  0 0 stepHeight; aForward*stepDirOnPlane.'];
    
    if stepping
        t = t + dt*direction*1.5;
        % increase/decrease t_leg by a single dt
        t_leg = t + legPhaseDiffs;
        for i = 1:nStanceLegs
            leg = stepOrder(i);
            xyz(:,leg) = minJerkStepGait3(stepWayPoints, jerkCoeffs, xyzStance0(:,leg), RFromPlane, fractionStep, t_leg(i));
            if (mod(t_leg(i), 2*pi)<2*pi*fractionStep)
                swingLegs(leg) = 1;
            else
                swingLegs(leg) = 0;
            end
        end
    end % end if stepping
    
    if ~isempty(extraLegs)
        thIKMatLast = reshape(thLast, [3 6]);
        thClawLast = thIKMatLast(:,extraLegs(1));
    end

        % moving the Claw arm
        % axes3 - left right
        % axes4 - front back
        % povs: buttons6 up, buttons8 down
        factor = 2;
        dxClaw = modifiedStickVal(3)*.01;
        dyClaw = -modifiedStickVal(4)*.01;
%          dphiClaw = modifiedStickVal(3)*.01;
%         dRClaw = -modifiedStickVal(4)*.01;
        dzClaw = .005*(buttons(6) - buttons(8));
        xyzClaw = xyz(:,extraLegs(1))+ [dxClaw; dyClaw; dzClaw]*factor;
%         xyzClaw = xyz(:,extraLegs(1))+ [dRClaw*cos(dphiClaw); dRClaw*sin(dphiClaw); dzClaw]*factor;
        % cap limits
        xyzClaw = min(xyzClaw,[.2; .4; .3]);
        xyzClaw = max(xyzClaw,[0; .1; -planexyc(3)-.02]);
        xyz(:,extraLegs(1)) =  xyzClaw;
   if ~isempty(extraLegs)&&withClaw
        % buttons(7) opens
        % buttons(5) closes
        clawOpenFactor = .3;
        clawAngle = clawAngle + (buttons(5)- buttons(7))*clawOpenFactor;
        clawAngle = min(clawAngle, pi/8);
        clawAngle = max(clawAngle, -pi/2);
   end
   

     
    thIK = kin.getIK(xyz);

%     % make sure extra arms elbows don't flip
%     thIKMat = reshape(thIK, [3 6]);
%     if ~isempty(extraLegs)
%         thClaw = thIKMat(:,extraLegs(1));
%         if norm(thClaw-thClawLast)>pi/2
%             thIKMat(:,extraLegs(1))= thClawLast;
%         end
%         thIK = reshape(thIKMat, [1 18]);
%     end

    
    xyzFK = kin.getLegPositions(thIK);
    
    % find the expected torques
    % jacobian in world frame for each leg:
    J = kin.getLegJacobians(thIK);
    % gravity compensation torques: (assumes fixed base)
    legTorques = kin.getLegGravCompTorques(thIK, gravity);
    % the force on each foot is the chassis weight plus the fixed module
    % segments, in the direction opposite gravity, divided with pinv
    xyzContact = xyzFK(:,~swingLegs);
    cmdFootForce = zeros(6,1);
    % the commanded foot force based on a best fit to the force distribution
    cmdFootForce(~swingLegs) = pinv(xyzContact) * -gravity*(bodyMass + 0);
    for i=1:6
        footWrench = [0;0;cmdFootForce(i); zeros(3,1)];
        if ~swingLegs(i) % legs in the air just have grav comp. Stance legs have some weight on them:
            legTorques(:,i) =  legTorques(:,i) + J(:,:,i).'*footWrench;
        end
    end
    
    % calculate velocity
    thDot = (thIK-thLast(1:18))/dt;
    thDot(abs(thDot)<.2) = NaN; % erase small velocities
    
    % sending commands
    if sendCommands
        cmd.position = [reshape(thIK,[1,18]) clawAngle+pi/4+pi/8];
        cmd.torque= [reshape(legTorques, [1,18]) NaN(size(clawAngle))];
        cmd.velocity = [thDot, NaN(size(clawAngle))];
        snakeMonster.set(cmd);
    end
    
        plt.plot([thIK clawAngle]); % needs this for time delay!

    if plotting
        
        bodyCoM = kin.getSnakeMonsterCoM([thIK clawAngle]);
        % project it onto the plane:
        dp = [0; 0; -planexyc(3)] - bodyCoM;% difference between origins of plane and point
        %     planeNormal = [planexyc(1); planexyc(2); 1];% relative position of point on normal's line
        temp = sum(planeNormal.*dp)/sum(planeNormal.^2);
        projBodyCoM = bodyCoM + temp*planeNormal;% add relative difference to project point back to plane
        [n_theta] = vrrotvec(planeNormal,[0;0;1]);
        R = rotmat(n_theta(1:3).', n_theta(end));
        % find the polygon and the distances in this rotated plane
        currentStanceLegs = stanceLegs(~swingLegs(stanceLegs));
        xyzContactRot = R*xyz(:,currentStanceLegs);
        K = convhull(xyzContactRot(1:2,:).');
        xOrderedRot = xyzContactRot(:,K.');
        xOrdered = xyzContact(:,K.');
        projBodyCoMRot = R*projBodyCoM;
        
       
        set(supportLines, 'xdata', xOrdered(1,:), 'ydata',xOrdered(2,:), ...
            'zdata', xOrdered(3,:));
        inSupport = inpolygon(projBodyCoMRot(1),projBodyCoMRot(2),xOrderedRot(1,:),xOrderedRot(2,:));
        set(projectedCOM, 'xdata', projBodyCoM(1), 'ydata',projBodyCoM(2), 'zdata', projBodyCoM(3),...
            'markerFaceColor', [1 0 0]*~inSupport);
        scatter3(xyzFK(1,:), xyzFK(2,:), xyzFK(3,:), 'r');
        scatter3(xyz(1,:), xyz(2,:), xyz(3,:), [], swingLegs, 'filled');
    end
    
    if buttons(10)
        running = 0;
    end
        
 
    tocLast = tocNow;
    
end


if sendCommands
    if logging
        snakeMonster.stopLog();
    end
    pause(3)
    % go limp
%     clawAngle = []
    cmd.position = nan(1,18+length(clawAngle));
    cmd.velocity = nan(1,18+length(clawAngle));
    cmd.torque = nan(1,18+length(clawAngle));
    snakeMonster.set(cmd);
end