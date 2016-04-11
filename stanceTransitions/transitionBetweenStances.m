% transition from first stance to second stance, one leg at a time

sendCommands = 1; % 1 to turn on commands to real robot
logging = 0; % Writes a hebi log
close all; clc;

addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));

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

% set starting pose
standingUp = 0;
if exist('thFinal' , 'var')
   thInit = thFinal;
else
    if standingUp&&sendCommands
        fbk = snakeMonster.getNextFeedback();
        thInit = fbk.position;
    else
        thInit = zeros(1,18);
    end
end
   
% set final pose

    % testing: start with a previously found stance
%     load('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project\stanceOptimization\findXYZOptWalkTilt\fourLegGood4_6_10am2.mat');
%     load('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project\stanceOptimization\findXYZOptWalkTilt\fiveLegGreat4_6_10am.mat');
    load('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project\stanceOptimization\findXYZOptWalkTilt\sixLegGood4_6_10am.mat');
    kin = SnakeMonsterKinematics(); % need to remake kin object
    xyz(:,extraLegs) = xyzExtra;
    xyz(1:2,stanceLegs) = reshape(xyStepOpt, [2,nStanceLegs]);
    xyz(3,stanceLegs) = ...
        - xyz(1,stanceLegs)*planexyc(1) - xyz(2,stanceLegs)*planexyc(2) - planexyc(3);
    thFinal = kin.getIK(xyz);

%     thFinal = zeros(1,18);


    
    plt = SnakeMonsterPlotter();
    tTransition = .5; % time it takes to transition each leg
    nTimes = 5; % number of waypoints to set
    
    % legOrder = 6:-1:1; % back to front for now
    legOrder = [6 5 2 1 3 4]; % back front middle
    
    thInit = reshape(thInit, [1 18]);
    thFinal = reshape(thFinal, [1 18]);
    thNow = reshape(thInit, [3,6]);
    plt.plot(thInit); hold on;
    
    
    
    
    % take a starting pose in terms of angles thInit
    % transition, with a specified leg order, to the stance with angles thFinal
    
    for leg = legOrder  % move stance legs one at a time
        for t = linspace(0,tTransition, nTimes)
            thInterp = interp1([0; tTransition], [thInit; thFinal], t); % interpolate for current value
            
            thInterpMat = reshape(thInterp, [3,6]); % back to matrix
            thNow(:,leg) = thInterpMat(:,leg);
            %      disp(thNow);
            plt.plot(thNow);
            if sendCommands
                cmd.position = reshape(thNow,[1,18]);
                snakeMonster.set(cmd);
            end
            %      pause(.001);
        end
    end
    
    
    if logging
        snakeMonster.stopLog();
    end
