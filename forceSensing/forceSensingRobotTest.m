% forceSensing Test
% go to a stationary stance, then plot forces and things.

logging = 0; % Writes a hebi log
close all; clc;

addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));
% addpath(genpath('C:\Users\Julian\Box Sync\CMU sem 1+2\snakeMonster\KDC_project'));
load('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project\stanceOptimization\findXYZOptWalkTilt\fiveLegGreat4_6_10am.mat');
%  load('C:\Users\Julian\Box Sync\CMU sem 1+2\snakeMonster\KDC_project\stanceOptimization\findXYZOptWalkTilt\fiveLegGreat4_6_10am.mat');

plt = SnakeMonsterPlotter;
pltFig = gcf;
kin = SnakeMonsterKinematics(); % need to remake kin object

pose = SnakeMonsterPose();

masses = kin.getLegMasses();
xyz(:,extraLegs) = xyzExtra;
xyz(1:2,stanceLegs) = reshape(xyStepOpt, [2,nStanceLegs]);
xyz(3,stanceLegs) = ...
    - xyz(1,stanceLegs)*planexyc(1) - xyz(2,stanceLegs)*planexyc(2) - planexyc(3);
xyzBase = xyz;
thIK = kin.getIK(xyz);
xyzFK = kin.getLegPositions(thIK);
gravity = -9.81*planeNormal/norm(planeNormal);
bodyMass = params.robotMass;

% % adjust swinglegs
% swingLegs(swingLegs ==1) = 0;

plt.plot(thIK)

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

% go to position
% initialize robot
% command structure for if sending commands
cmd = CommandStruct();
% cmd.position = nan(1,18);
% cmd.velocity = nan(1,18);
% cmd.torque = nan(1,18);
setupSnakeMonsterGroup; % makes the snakeMonster hebi group object
setGainsSM;
if logging
    snakeMonster.startLog();
end

% set initial stance
cmd.position = thIK;
cmd.torque = reshape(legTorques, [1 18]);
snakeMonster.set(cmd);

% start records
t=[];
thFbkRecord = [];
thCmdRecord = [];
torqueFbkRecord = [];
torqueCmdRecord = [];
fRecord = [];
plottingFig = figure('position', [50 50 600 900]);
nPlots = 5;
subplot(nPlots,1,1);
for i =1:18
thFbkPlot(i) = plot(0,0); hold on;
end
ylabel('\theta _f_b_k');
subplot(nPlots,1,2);
for i =1:18
thCmdPlot(i) = plot(0,0);hold on;
end
ylabel('\theta _c_m_d');
subplot(nPlots,1,3);
for i =1:18
torqueFbkPlot(i) = plot(0,0);hold on;
end
ylabel('\tau _f_b_k');
subplot(nPlots,1,4);
for i =1:18
torqueCmdPlot(i) = plot(0,0);hold on;
end
ylabel('\tau _c_m_d');
subplot(nPlots,1,5);
for i =1:3
fPlot(i) = plot(0,0);hold on;
end
ylabel('force guess');

running = 1;
joy = vrjoystick(1);
contactLegs = find(~swingLegs);

tic;
while running
    
    [swivel, buttons, povs] = read( joy );
    fbk = snakeMonster.getNextFeedback();
    
%         plt.plot(fbk.position)
%       figure(pltFig);
      try
%     pose.plotEnvironment(fbk);
    [poly, contactLegs] = pose.getSupportPolygon(fbk);
%     disp(contactLegs.');
      catch
         disp('not enough contacts for pose filter') 
      end
          
     
        % find the expected torques
% jacobian in world frame for each leg:
J = kin.getLegJacobians( fbk.position );
% gravity compensation torques: (assumes fixed base)
legTorques = kin.getLegGravCompTorques( fbk.position , gravity);
% the force on each foot is the chassis weight plus the fixed module
% segments, in the direction opposite gravity, divided with pinv
xyzContact = xyzFK(:,contactLegs);
cmdFootForce = zeros(6,1);
% the commanded foot force based on a best fit to the force distribution
cmdFootForce(contactLegs) = pinv(xyzContact) * -gravity*(bodyMass + 0);
for i=1:6
    footWrench = [0;0;cmdFootForce(i); zeros(3,1)];
    if any(contactLegs == i) % legs in the air just have grav comp. Stance legs have some weight on them:
        legTorques(:,i) =  legTorques(:,i) + J(:,:,i).'*footWrench;
    end
end
% cmd.torque = reshape(legTorques, [1 18]);
       
    
%     n = sum(~swingLegs);
    n =length(contactLegs);
    torqueAppliedTotal = fbk.torque - cmd.torque;
    torqueAppliedTotalMat = reshape(torqueAppliedTotal, [3 6]);
    torqueAppliedMat = torqueAppliedTotalMat(:,contactLegs);
    fGuess = -n* pinv(reshape(J(:,:,contactLegs), [6,3*n]).')*reshape(torqueAppliedMat, [3*n,1]);

t = [t; toc];    
thFbkRecord = [thFbkRecord; fbk.position];
thCmdRecord = [thCmdRecord; cmd.position];
torqueFbkRecord = [torqueFbkRecord; fbk.torque];
torqueCmdRecord = [torqueCmdRecord; cmd.torque];
fRecord = [fRecord; fGuess.'];

% figure(plottingFig);
for k = 1:18
set(thFbkPlot(k), 'xdata', t, 'ydata', thFbkRecord(:,k));
set(thCmdPlot(k), 'xdata', t, 'ydata', thCmdRecord(:,k));
set(torqueFbkPlot(k), 'xdata', t, 'ydata', torqueFbkRecord(:,k));
set(torqueCmdPlot(k), 'xdata', t, 'ydata', torqueCmdRecord(:,k));
end
for k = 1:3
set(fPlot(k), 'xdata', t, 'ydata', fRecord(:,k));
end
    
    if buttons(10)
        running = 0;
    end
    snakeMonster.set(cmd);
    pause(.001);
    
    
end


snakeMonster.stopLog();