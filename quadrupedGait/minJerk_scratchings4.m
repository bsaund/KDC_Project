% parametric testing
% playing with the parameters in the min jerk gait

% time t = 0:2*pi parameter for step length
close all;
figure;
plot3(0,0,0); hold on;
xlabel('x'); ylabel('y'); zlabel('z');
stepLength = .1; stepHeight = .06;
axis([-1 1 -1 1 -1 1]*stepLength*2);
fractionStep = 1/5; % how long is it in the air out of 1 second
stepPeriod = .6;  % how long the stepping lasts
% solve for coefficients to create trajectory with min jerk
jerkCoeffs = minimumJerk( 0, 0, 0, ... % Starting Phase/Vel/Accel
                          1, 0, 0, ... % Ending Phase/Vel/Accel
                          stepPeriod*fractionStep);  % Time to touchdown

dt = .01;
xyzLift = [0;0;0];
xyzLand = [0;stepLength;.15];

for t = 0:dt:stepPeriod % full cycle

    xyz = minJerkStepGait4(xyzLift, xyzLand, jerkCoeffs, stepHeight, fractionStep, stepPeriod, t);

    if (t>=0 && t<stepPeriod*fractionStep) % swing phase
     scatter3(xyz(1),xyz(2),xyz(3),'r' );   
else % stance phase
        scatter3(xyz(1),xyz(2),xyz(3),'k' );   
    end
    pause(.05);
end
