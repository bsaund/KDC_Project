% parametric testing
% playing with the parameters in the min jerk gait

% making an open loop step pattern
a = .05; % step length = 2*a
b = .05; % step height = b
% time t = 0:2*pi parameter for step length
close all;
figure;
plot(0,0); hold on;
xlim([-a a]*1.5);
ylim([-b b ]*1.5);
fractionStep = 1/5; % how long is it in the air out of 1 second
stepPeriod = 2*pi*fractionStep;  % how long the stepping lasts
 % solve for coefficients to create trajectory with min jerk
jerkCoeffs = minimumJerk( 0, 0, 0, ... % Starting Phase/Vel/Accel
                          1, 0, 0, ... % Ending Phase/Vel/Accel
                          stepPeriod);  % Time to touchdown
stepWayPoints = [-a 0; 0 b; a 0];
                      
for t = 0:.1:2*pi % full cycle of step is 2*pi

if (t>=0 && t<2*pi*fractionStep) % swing phase
    
            phase = jerkCoeffs(4)*t.^3 + ...
                    jerkCoeffs(5)*t.^4 + ...
                    jerkCoeffs(6)*t.^5;
                
   yz= interp1([0 .5 1], stepWayPoints, phase, 'spline');
      y = yz(1); z = yz(2);          
                
%     z = -b*sin(t/fractionStep/2 - pi);
%     y = a*cos(t/fractionStep/2 - pi);
    scatter(y,z);
else % stance phase
%       y = a - 2*a*(t/(2*pi - 2*pi*fractionStep) - 2*pi*fractionStep/(2*pi - 2*pi*fractionStep));
       y = stepWayPoints(end,1) - ...
         diff(stepWayPoints([1 end],1))*(t/(2*pi - 2*pi*fractionStep) - 2*pi*fractionStep/(2*pi - 2*pi*fractionStep));
  
z = 0;
      scatter(y,z,'filled');
end


pause(.05);
end
