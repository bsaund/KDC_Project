% generalWalking3
% walking with smoothness improved

% flags
sendCommands = 0; % 1 to turn on commands to real robot
logging = 0; % Writes a hebi log
plotting = 1;
withClaw = 0;
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
    setupSnakeMonsterGroup; % makes the snakeMonster hebi group object
    setGainsSM;
    if logging
        snakeMonster.startLog();
    end
end

% testing: start with a previously found stance
% load('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project\stanceOptimization\findXYZOptWalkTilt\fourLegGood4_6_10am2.mat');
% load('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project\stanceOptimization\findXYZOptWalkTilt\fiveLegGreat4_6_10am.mat');
load('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project\stanceOptimization\findXYZOptWalkTilt\sixLegGood4_6_10am.mat');

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
xyzDesired = xyz;
thFinal = kin.getIK(xyz);
thNew = zeros(1,18);


%% make all of the gait variables
a_forward = stepLength/2; %
a_back = stepLength/2; % step length = a_forward  + a_back
b = .06; % step height = b. .06 works.

legPhaseDiffs = 2*pi/nStanceLegs*(nStanceLegs:-1:1);
fractionStep = 1/nStanceLegs;
% alternating tripod stuff
if nStanceLegs ==6
    legPhaseDiffs = pi*[1 0 0 1 1 0];
    fractionStep = 1/2;
    stepOrder = 1:6;
end

% stepPeriod = 2*pi*fractionStep;  % how long the stepping lasts
stepPeriod = 5;  % how long the stepping lasts
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
swingLegs = zeros(1,6); % 1 indicates leg is in the air
swingLegs(extraLegs) = 2;
gravity = -9.81*planeNormal/norm(planeNormal);
bodyMass = (params.robotMass +sum(sum(masses)));

if withClaw
    clawAngle = 0;
else
    clawAngle = [];
end

plt.plot([thNew clawAngle]); hold on;
if plotting
    
    % plotting start up
    projectedCOM = scatter3(0,0,0,'k', 'filled');
    supportLines = plot3(0,0,0,'k');
end

footStepThres = .02; % distance the feet are allowed to be to not step
currentStepInd = 1;

% flags
running = 1; % flag for running
stepping = 0; % flag for currently taking a step
direction = 1; % 1= forwards direction. -1 = backwards.
% dt = pi*fractionStep/10;

tic; % start clock
tocLast = 0;

while running
    
    tocNow = toc;   % get elapsed time now
    dt = tocNow - tocLast;  % period of while loop [s]
    
%     % read joystick
    [axes, buttons, povs] = read( joy );

    
if sendCommands
    % get feedback
    fbk = snakeMonster.getNextFeedback();
    thNow = fbk.position(1:18);    
else
 thNow = thNew;
end

    plt.plot([thNow clawAngle])
    % get current foot positions in body frame
    xyzNow = kin.getLegPositions(thNow);
    
    % if the foot positions aren't quite right, then we want to take a step.
    steppingFeet =  sqrt(sum((xyzNow-xyzDesired).^2)) > footStepThres;
    if any(steppingFeet) && ~stepping % initiate a new step
        stepping = 1;
        xyzLift = xyzNow;
        xyzLand = xyzDesired;
        tocLift = tocNow;
    end
    % take a step
    if stepping
        stepTime = tocNow - tocLift;
        legTimeDiffs = stepTime + stepPeriod/nStanceLegs*(nStanceLegs:-1:1);
        for i = 1:nStanceLegs
           leg = stepOrder(i);
            xyz(:,leg) =  minJerkStepGait4(xyzLift(:,leg), xyzLand(:,leg), ...
                jerkCoeffs, b, fractionStep, stepPeriod, legTimeDiffs(leg));
        end
        swingLegs = mod(legTimeDiffs,stepPeriod*fractionStep)<(stepPeriod*fractionStep);
        
%         stepOrder(currentStepInd)
        thNew = kin.getIK(xyz);
        
        if stepTime >= stepPeriod*fractionStep
            stepping = false;
        end
    end
    
     if buttons(10)
        running = 0;
     end
    
    
     
     tocLast = tocNow;
    
end
