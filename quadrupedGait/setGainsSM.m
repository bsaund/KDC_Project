%% Read and Set Gains
% Iterate thru all the groups and set the gains to default
      
    gains = GainStruct(); 

    numModules = snakeMonster.getNumModules();

    % Maximum velocity of SEA joints is 33 rpm; we convert to rad/s here.
    maxVelocityRadS = (33 * 2 * pi) / 60;
    
    maxTorque = 12; % N-m
gains.controlStrategy= ones(1, numModules) *4; % strategy 4
    gains.positionKp = ones(1, numModules) * 10.0;% past value: 6
    gains.positionKi = ones(1, numModules) * 0;
    gains.positionKd = ones(1, numModules) * 30; % past value: 30
    gains.positionIClamp = ones(1, numModules) * 0;
    gains.positionFF = ones(1, numModules) * 0;
    gains.positionPunch = ones(1, numModules) * 0;
    gains.positionDeadZone = ones(1, numModules) * 0.00;
    gains.positionMinOutput = ones(1, numModules) * -maxTorque;
    gains.positionMaxOutput = ones(1, numModules) * maxTorque;
    gains.positionMinTarget = ones(1, numModules) * -(pi/2 + 0.2);
    gains.positionMaxTarget = ones(1, numModules) * (pi/2 + 0.2);
    gains.positionTargetLowpassGain = ones(1, numModules) * 1;
    gains.positionOutputLowpassGain = ones(1, numModules) * 1;
    gains.positionDOnError = ones(1, numModules) * 1;

    gains.velocityKp = ones(1, numModules) * 0.2;
    gains.velocityKi = ones(1, numModules) * .000;
    gains.velocityIClamp = ones(1, numModules) * .0;
    gains.velocityKd = ones(1, numModules) * 0;
    gains.velocityFF = ones(1, numModules) * 1/maxVelocityRadS;
    gains.velocityDeadZone = ones(1, numModules) * 0.05;
    gains.velocityPunch = ones(1, numModules) * 0.00;
    gains.velocityMinOutput = ones(1, numModules) * -1;
    gains.velocityMaxOutput = ones(1, numModules) * 1;
    gains.velocityMinTarget = ones(1, numModules) * -maxVelocityRadS;
    gains.velocityMaxTarget = ones(1, numModules) * maxVelocityRadS;
    gains.velocityTargetLowpassGain = ones(1, numModules) * 1;
    gains.velocityOutputLowpassGain = ones(1, numModules) * 1;
    gains.velocityDOnError = ones(1, numModules) * 1;

    gains.torqueKp = ones(1, numModules) * .5; % past value .5
    gains.torqueKi = ones(1, numModules) * 0;
    gains.torqueIClamp = ones(1, numModules) * 0;
    gains.torqueKd = ones(1, numModules) * 25;
    gains.torqueFF = ones(1, numModules) * 0.15;
    gains.torqueDeadZone = ones(1, numModules) * 0.01;
    gains.torquePunch = ones(1,numModules) * 0;
    gains.torqueMinTarget = ones(1, numModules) * -maxTorque;
    gains.torqueMaxTarget = ones(1, numModules) * maxTorque;
    gains.torqueMinOutput = ones(1, numModules) * -1;
    gains.torqueMaxOutput = ones(1, numModules) * 1;
    gains.torqueTargetLowpassGain = ones(1, numModules) * 1;
    gains.torqueOutputLowpassGain = ones(1, numModules) * .25;
    gains.torqueDOnError = ones(1, numModules) * 0;
   
    % Set the Gains (NO PERSIST)
    snakeMonster.set('gains', gains);
    
snakeMonster.setCommandLifetime(0);
%     % Set the Gains (PERSIST)
%     gainGroup.set('gains', gains, 'persist', true, 'led', 'y');

% end

pause(2.0);

