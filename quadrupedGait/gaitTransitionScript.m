% gaitTransitionScript
% start with some arbitrary stance with all 6 legs on the ground.
% move the legs 1 at a time until they are in the stable stance


sendCommands = 1; % 1 to turn on commands to real robot
logging = 0; % Writes a hebi log

addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));

% mainStanceOpt; % runs the stance optimization
% we now have a kinematics object, stanceLegs, and a full set of angles
% (thFull) to send as a desired pose.

xyzInit = xyz0; % the zero joint position
xyzInit(2,:) = xyzInit(2,:) + [.15 .15 0 0 -.15 -.15];
thInit = kin.getIK(xyzInit);

figure;
plt2 = SnakeMonsterPlotter();
plt2.plot(thInit); hold on;
set(gcf, 'position', [10 800 800 700]);

tTransition = .5;
thNow = reshape(thInit, [3 6]);


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


nTimes = 20;
% do stance legs one at a time
for leg = stanceLegs  % move stance legs one at a time
    for t = linspace(0,tTransition, nTimes)
        thInterp = interp1([0; tTransition], [thInit; reshape(thFull, [1 18])], t); % interpolate for current value
        
        thInterpMat = reshape(thInterp, [3,6]); % back to matrix
        thNow(:,leg) = thInterpMat(:,leg);
        %      disp(thNow);
        plt2.plot(thNow);
        if sendCommands
            cmd.position = reshape(thNow,[1,18]);
            snakeMonster.set(cmd);
        end
        %      pause(.001);
    end
end

% plotting
%   plt = SnakeMonsterPlotter();
plt2.plot(thNow); hold on;
%   set(gcf, 'position', [10 100 800 700]);
%  scatterCoM= scatter3(legCoM(1,:), legCoM(2,:), legCoM(3,:), 'k');
projectedCOM = scatter3(0,0,0,'k', 'filled');
supportLines = plot3(0,0,0,'k');
scatterCentroid = scatter3(0,0,0,'b');
set(supportLines, 'xdata', xOrdered(1,:), 'ydata',xOrdered(2,:), ...
    'zdata', xOrdered(3,:));
set(projectedCOM, 'xdata', bodyCoM(1), 'ydata',bodyCoM(2), 'zdata', z0,...
    'markerFaceColor', [1 0 0]*~inSupport);
set(scatterCentroid, 'xdata', xCentroid(1), 'ydata',xCentroid(2), 'zdata',z0);


% lift all extra legs at once
for t = linspace(0,tTransition, nTimes)
    thInterp = interp1([0; tTransition], [thInit; reshape(thFull, [1 18])], t); % interpolate for current value
    
    thInterpMat = reshape(thInterp, [3,6]); % back to matrix
    thNow(:,extraLegs) = thInterpMat(:,extraLegs);
    %      disp(thNow);
    plt2.plot(thNow);
    if sendCommands
        cmd.position = reshape(thNow,[1,18]);
        snakeMonster.set(cmd);
    end
    %      pause(.001);
end

    if logging
        snakeMonster.stopLog();
    end

