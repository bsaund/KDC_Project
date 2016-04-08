% generalWalking
% load a previously made gait stance and parameters, and walk with it based
% on those parameters and inupt from joystick
% including some flag to tell it if its stepping or not.
% this will only work for +/- the stepDirection that the stance was
% optimized for. Other directions will probably fall over.

% load previous parameters
sendCommands = 0; % 1 to turn on commands to real robot
logging = 0; % Writes a hebi log
close all; clc;

addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));

% plt.plot(th0);
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
    cmd.position = reshape(thNow,[1,18]);
    snakeMonster.set(cmd);
    disp('press any key to start')
    pause;
    if logging
        snakeMonster.startLog();
    end
end

% testing: start with a previously found stance
load('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project\stanceOptimization\findXYZOptWalkTilt\fourLegGood4_6_10am2.mat');
% load('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project\stanceOptimization\findXYZOptWalkTilt\fiveLegGreat4_6_10am.mat');
% load('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project\stanceOptimization\findXYZOptWalkTilt\sixLegGood4_6_10am.mat');

plt = SnakeMonsterPlotter();
kin = SnakeMonsterKinematics(); % need to remake kin object
xyz(:,extraLegs) = xyzExtra;
xyz(1:2,stanceLegs) = reshape(xyStepOpt, [2,nStanceLegs]);
xyz(3,stanceLegs) = ...
    - xyz(1,stanceLegs)*planexyc(1) - xyz(2,stanceLegs)*planexyc(2) - planexyc(3);
xyzBase = xyz;
thFinal = kin.getIK(xyz);

%% make all of the gait variables
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

% get intitial leg pose
t=0;
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
thIK = kin.getIK(xyz);
plt.plot(thIK);


running = 1; % flag for running
stepping = 0; % flag for currently taking a step
direction = 1; % 1= forwards direction. -1 = backwards.
tic;
while running
    
    [axes, buttons, povs] = read( joy );
    if axes(2)<-.05
        stepping = 1;
        direction = 1;
    elseif axes(2)>.05
        stepping = 1; 
        direction = -1;
    end
    
    if stepping
        
        for tAdd = 0:.1:2*pi/nStanceLegs
            % get the foot placement points
            t_leg = t + tAdd*direction + 2*pi/nStanceLegs*(nStanceLegs:-1:1);
            for i = 1:nStanceLegs
                leg = stepOrder(i);
                xyz(:,leg) = minJerkStepGait3(stepWayPoints, jerkCoeffs, xyzStance0(:,leg), RFromPlane, fractionStep, t_leg(i));
                
                if (mod(t_leg(i), 2*pi)<2*pi*fractionStep)
                    swingLegs(leg) = 1;
                else
                    swingLegs(leg) = 0;
                end
            end
            
            [axes, buttons, povs] = read( joy );
            if buttons(10) % to do: replace with joystick button
                running = 0;
                break;
            end
            thIK = kin.getIK(xyz);
            plt.plot(thIK);
        end
        t = t + 2*pi/nStanceLegs*direction;
        stepping= 0;
    else
    
    
    % plot old config
%     thIK = kin.getIK(xyz);
    plt.plot(thIK);
    end
    
%     xyzFK = kin.getLegPositions(thIK);
%     % %  the support polygon
%     xyzContact = xyzFK(:,~swingLegs);
%     K = convhull(xyzContact(1:2,:).');
%     xOrdered = xyzContact(:,K.');
%     xCentroid = mean(xOrdered(:,1:end-1),2);
%     
%     bodyCoM = kin.getSnakeMonsterCoM(thIK);
%     % project it onto the plane:
%     dp = [0; 0; -planexyc(3)] - bodyCoM;% difference between origins of plane and point
%     %     planeNormal = [planexyc(1); planexyc(2); 1];% relative position of point on normal's line
%     temp = sum(planeNormal.*dp)/sum(planeNormal.^2);
%     projBodyCoM = bodyCoM + temp*planeNormal;% add relative difference to project point back to plane
%     
%     [n_theta] = vrrotvec(planeNormal,[0;0;1]);
%     R = rotmat(n_theta(1:3).', n_theta(end));
%     % find the polygon and the distances in this rotated plane
%     currentStanceLegs = stanceLegs(~swingLegs(stanceLegs));
%     xyzContactRot = R*xyz(:,currentStanceLegs);
%     K = convhull(xyzContactRot(1:2,:).');
%     xOrderedRot = xyzContactRot(:,K.');
%     projBodyCoMRot = R*projBodyCoM;
    
    
    
    %   set(supportLines, 'xdata', xOrdered(1,:), 'ydata',xOrdered(2,:), ...
    %      'zdata', xOrdered(3,:));
    %   inSupport = inpolygon(projBodyCoMRot(1),projBodyCoMRot(2),xOrderedRot(1,:),xOrderedRot(2,:));
    %   set(projectedCOM, 'xdata', projBodyCoM(1), 'ydata',projBodyCoM(2), 'zdata', projBodyCoM(3),...
    %      'markerFaceColor', [1 0 0]*~inSupport);
    
    
    
    if buttons(10) % to do: replace with joystick button
        running = 0;
    end
    
end

