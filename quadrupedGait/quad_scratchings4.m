% quadruped gait testing with HebiKinematics
% With simulation and sendCommands option.
% Can plot torques
% uses minJerk coefficients (although possibly incorrectly)

simulation = 1; %0 to turn off simulation
sendCommands = 1; % 1 to turn on commands to real robot
plotting =0;

addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));
% addpath(genpath('C:\Users\Julian\Box Sync\CMU sem 1+2\snakeMonster\KDC_project'));

th0 = zeros(3,6); % joint angles: each column is a leg (proximal to distal)
params = SMPhysicalParameters();
SMData = makeSMData(params);
inch2m = 0.0254;
gravity = [0 0 -9.81];
odd = [1,3,5]; even= [2 4 6];

kin = SnakeMonsterKinematics; % does all the kinamatics

xyz0 = kin.getLegPositions(th0);
xyz = xyz0;

% move all feet in towards chassis
xyz(1,odd) = xyz(1,odd)-.1;
xyz(1,even) = xyz(1,even)+.1;
xyz(3,:) = xyz(3,:)+.05;


 % walking states: which legs are walking, swining, extra.
   swingLegs = zeros(1,6); % 1 indicates leg is in the air 
 walkingLegs = [ 2 3 4 5 6]; % specify which legs will be used for walking
 nWalkingLegs = length(walkingLegs);
 fractionStep = 1/nWalkingLegs;
%  stepOrderBase = [1 2 3 4 5 6]; % in order
%  stepOrderBase = [6 4 2 5 3 1]; % back to front
%   stepOrderBase = [1 3 5 2 4 6]; % fornt to back
%    stepOrderBase = [1 2 3 4 6 5]; % works ok
%    stepOrderBase = [1 4 3 6 2 5]; % works ok
% stepOrderBase = [1 4 2 3 6  5]; % works ok
stepOrderBase = [1 3 4 2 6  5]; % works best

 stepOrder = [];
 extraLegs = [];
 % remove the non-walking legs from the step order
 for i = 1:6
    if sum(walkingLegs == stepOrderBase(i))<1
         extraLegs = [extraLegs stepOrderBase(i)];
    else
        stepOrder = [stepOrder stepOrderBase(i)];
    end
 end
 swingLegs(extraLegs) = 2;
 
 % set the non-walking legs to be up in the air
 for i = 1:(6-nWalkingLegs) 
     xyz(3,extraLegs(i)) =.1;
     xyz(2,extraLegs(i)) =.5*sign(xyz0(2,extraLegs(i)));
     xyz(1,extraLegs(i)) =.1*sign(xyz0(1,extraLegs(i)));
 end
 
t = 0;
a = params.L/3; % step length = 2*a
 b = .06; % step height = b
 z0 = ones(1,6)*-.15;
 y0 = [params.L/2 params.L/2 0 0 -params.L/2 -params.L/2];
%  y0(odd)=y0(odd)+ [0 2*a 0];  y0(even) =  y0(even) +[3*a 0 -3*a]; % works ok
 y0(odd)=y0(odd)+ [0 2*a 0];  y0(even) =  y0(even) +[3*a 0 -3*a];
 
stepPeriod = 2*pi*fractionStep;  % how long the stepping lasts
 % solve for coefficients to create trajectory with min jerk
jerkCoeffs = minimumJerk( 0, 0, 0, ... % Starting Phase/Vel/Accel
                          1, 0, 0, ... % Ending Phase/Vel/Accel
                          stepPeriod);  % Time to touchdown
stepWayPoints = [-a 0; 0 b; a 0];

                      
  % get initial IK
   t_leg = t+2*pi/nWalkingLegs*(nWalkingLegs:-1:1);
    % find the point on the eliptical path
    for i = 1:nWalkingLegs
        leg = stepOrder(i);
%         [xyz(2,leg),xyz(3,leg)] = ellipticalGait(a,b,y0(leg),z0(leg), fractionStep, t_leg(i));
        [xyz(2,leg),xyz(3,leg)] =...
            minJerkStepGait(stepWayPoints, jerkCoeffs, y0(leg), z0(leg),fractionStep, t_leg(i));

        if (mod(t_leg(i), 2*pi)<2*pi*fractionStep)&&(mod(t_leg(i), 2*pi)>0)
            swingLegs(leg) = 1;
        else
            swingLegs(leg) = 0;
        end
    end
th = kin.getIK(xyz);
effectors = kin.getLegPositions(th);
CoMs = kin.getCenterOfMasses(th);
masses = kin.getLegMasses();
legCoM = zeros(3,7);
for i = 1:6
  legCoM(:,i) = CoMs(:,:,i)*masses(:,i)/sum(masses(:,i));
end
 legCoM(:,end) = legCoM*[sum(masses) params.robotMass].' /...
     sum([sum(masses) params.robotMass]); % full body COM
     
 close all;
if simulation
plt = SnakeMonsterPlotter(); hold on; 
simAx = get(gcf,'children');
simFig = gcf;
set(simFig, 'position', [100 100 800 800]);
scatterCoM= scatter3(legCoM(1,:), legCoM(2,:), legCoM(3,:), 'k');
projectedCOM = scatter3(0,0,0,'k', 'filled');
supportLines = plot3(0,0,0,'k');
scatterCentroid = scatter3(0,0,0,'b');
view([0 0 -1]);
 plt.plot(reshape(th,[1,18]));
end
if plotting
   plotFig = figure;
   torqueFbkPlot = zeros(1,18);
   torqueCmdPlot = zeros(1,18);
   torquePlotColors = {'k' 'k' 'k' 'r' 'r' 'r' 'b' 'b' 'b'...
                       'g' 'g' 'g' 'm' 'm' 'm' 'c' 'c' 'c' };
   
   subplot(2,1,1);
   for k =1:18
   torqueFbkPlot(k) = plot(0,0, torquePlotColors{k});   hold on;
   end
 subplot(2,1,2);
   for k =1:18
   torqueCmdPlot(k) = plot(0,0, torquePlotColors{k}); hold on;
   end
legend(torqueCmdPlot(1:3:end),'1','2','3','4','5','6')
end

 % command structure for if sending commands
      cmd = CommandStruct();
        cmd.position = [];
        cmd.velocity = [];
        cmd.torque = [];
if sendCommands
         setupSnakeMonsterGroup; % makes the snakeMonster hebi group object
         
          cmd.position = reshape(th,[1,18]);
      snakeMonster.set(cmd);
disp('press any key to start')
      pause;
      snakeMonster.startLog();

 end

  
  nCycles = 2;
t_span = linspace(0,2*pi*nCycles,25*nCycles);
tic;
tStart = 0;
tRecord=[];
torqueFbkRecord = [];
torqueCmdRecord=[];

 for t = t_span
%  % find the jacobian
%  legKin{6}.getJacobian('EndEffector', th(:,i))
%  % find the gravity compensation torques
%    legKin{6}.getGravCompTorques(th(:,i),gravity)

if sendCommands
    fbk = snakeMonster.getNextFeedback();
   thLast = fbk.position;
else
  thLast = th;
end

   t_leg = t+2*pi/nWalkingLegs*(nWalkingLegs:-1:1);
    % find the point on the eliptical path
    for i = 1:nWalkingLegs
        leg = stepOrder(i);
%         [xyz(2,leg),xyz(3,leg)] = ellipticalGait(a,b,y0(leg),z0(leg), fractionStep, t_leg(i));  
        [xyz(2,leg),xyz(3,leg)] =...
            minJerkStepGait(stepWayPoints, jerkCoeffs, y0(leg), z0(leg),fractionStep, t_leg(i));
        if (mod(t_leg(i), 2*pi)<2*pi*fractionStep)&&(mod(t_leg(i), 2*pi)>0)
            swingLegs(leg) = 1;
        else
            swingLegs(leg) = 0;
        end
    end

th = kin.getIK(xyz);
% th = zeros(1,18); swingLegs = ones(1,6); % for testing only
effectors = kin.getLegPositions(th);
CoMs = kin.getCenterOfMasses(th);
for i = 1:6
  legCoM(:,i) = CoMs(:,:,i)*masses(:,i)/sum(masses(:,i));
end
 legCoM(:,end) = legCoM*[sum(masses) params.robotMass].' /...
     sum([sum(masses) params.robotMass]); % full body COM
 
 % find the expected torques
 % jacobian in world frame for each leg:
 J = kin.getLegJacobians(th);
 % gravity compensation torques: (assumes fixed base)
 legTorques = kin.getLegGravCompTorques(th, gravity);
 for i=1:6
     % the force on each foot is the chassis weight plus the fixed module
     % segments, in the direction opposite gravity, divided evenly among
     % stance legs.
    footWrench = -[gravity zeros(1,3)].' * (params.robotMass +sum(masses(1,:))/2)/sum(~swingLegs);
     if ~swingLegs(i) % legs in the air just have grav comp. Stance legs have some weight on them:
      legTorques(:,i) =  legTorques(:,i) + J(:,:,i).'*footWrench;
     end 
 end
 
 % find the expected joint velocities
 tEnd = toc;
 thDot = (th-thLast)/(tEnd - tStart);
 tStart = toc;
 
 
 % plot the support polygon
 xContact = effectors(:,~swingLegs);
 K = convhull(xContact(1:2,:).');
 xOrdered = xContact(1:2,K.');
 % check if COM in the support poluygon
 inSupport = inpolygon(legCoM(1,end),legCoM(2,end),xOrdered(1,:),xOrdered(2,:));
 % find the centroid of the support polygon
 xCentroid = mean(xOrdered(:,1:end-1),2);
 

 
  if simulation
%  set(0, 'CurrentFigure', simFig)
 plt.plot(reshape(th,[1,18]));
axes(simAx); % change current ax to the one in the sm plotter
 scatter3(effectors(1,:), effectors(2,:), effectors(3,:), 'r');
 scatter3(xyz(1,:), xyz(2,:), xyz(3,:), [], swingLegs, 'filled');
 set(scatterCoM, 'xdata', legCoM(1,:), 'ydata',legCoM(2,:), 'zdata',legCoM(3,:));
 set(supportLines, 'xdata', xOrdered(1,:), 'ydata',xOrdered(2,:), ...
     'zdata',ones(1,size(xOrdered,2))*mean(z0));
  set(scatterCoM, 'xdata', legCoM(1,:), 'ydata',legCoM(2,:), 'zdata',legCoM(3,:));
 set(projectedCOM, 'xdata', legCoM(1,end), 'ydata',legCoM(2,end), 'zdata',mean(z0),...
     'markerFaceColor', [1 0 0]*~inSupport);
 set(scatterCentroid, 'xdata', xCentroid(1), 'ydata',xCentroid(2), 'zdata',mean(z0));
% view([0 0 -1]);
  end
   
 if plotting
     if sendCommands
         torqueFbkRecord = [torqueFbkRecord; fbk.torque];
     else
         torqueFbkRecord = [torqueFbkRecord; zeros(1,18)];
     end
     torqueCmdRecord = [torqueCmdRecord; reshape(legTorques, [1,18])];
     tRecord = [tRecord; tEnd];
     for k = 1:18
         set(torqueFbkPlot(k), 'xdata', tRecord, 'ydata', torqueFbkRecord(:,k) );
         set(torqueCmdPlot(k), 'xdata', tRecord, 'ydata', torqueCmdRecord(:,k) );
     end
 end
  if sendCommands
      cmd.position = reshape(th,[1,18]);
%       cmd.torque=reshape(legTorques, [1,18]);
%       cmd.velocity = thDot;
      snakeMonster.set(cmd);
  end
 pause(0.01);
 end

 if sendCommands
     snakeMonster.stopLog();
     pause(3)
 % go limp
 cmd.position = nan(1,18);
  cmd.velocity = nan(1,18);
   cmd.torque = nan(1,18);
   snakeMonster.set(cmd);
 end
   
%    th = [0 0 0 0 0 0; ...
%         0 0 0 0 0 0; ...
%         0 0 0 0 0 0];
    
   