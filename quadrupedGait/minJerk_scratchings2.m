% parametric testing
% playing with the parameters in the min jerk gait

% making an open loop step pattern
a = .05; % step length = 2*a
b = .05; % step height = b
% time t = 0:2*pi parameter for step length
close all;
figure;
plot3(0,0,0); hold on;
xlabel('x'); ylabel('y'); zlabel('z');
xlim([-a a]*1.5);
ylim([-b b ]*1.5);
fractionStep = 1/5; % how long is it in the air out of 1 second
stepPeriod = 2*pi*fractionStep;  % how long the stepping lasts
% solve for coefficients to create trajectory with min jerk
jerkCoeffs = minimumJerk( 0, 0, 0, ... % Starting Phase/Vel/Accel
    1, 0, 0, ... % Ending Phase/Vel/Accel
    stepPeriod);  % Time to touchdown
stepWayPoints = [0 -a 0; 0 0 b; 0 a 0]; % determine which points the feet pass through
tPhases= linspace(0,1,size(stepWayPoints,1));
dt = .3;
xyzStep0 = [0;0;0];

for stepDirection = pi/4:pi/4:pi

% stepDirection=0;


for t = 0:dt:2*pi % full cycle of step is 2*pi
   R =  R_z(stepDirection);
    if (t>=0 && t<2*pi*fractionStep) % swing phase
        
        phase = jerkCoeffs(4)*t.^3 + ...
            jerkCoeffs(5)*t.^4 + ...
            jerkCoeffs(6)*t.^5;
        phaseLast = jerkCoeffs(4)*(t-dt).^3 + ...
            jerkCoeffs(5)*(t-dt).^4 + ...
            jerkCoeffs(6)*(t-dt).^5;
        phaseNext = jerkCoeffs(4)*(t+dt).^3 + ...
            jerkCoeffs(5)*(t+dt).^4 + ...
            jerkCoeffs(6)*(t+dt).^5;
        
     planePoint= interp1(linspace(0,1,size(stepWayPoints,1)), stepWayPoints, phase, 'spline');
     rotPlanePoint = R*planePoint.';
        
%         yz= interp1(tPhases, stepWayPoints, phase, 'spline');
%         yzLast= interp1(tPhases, stepWayPoints, phaseLast, 'spline');
%         yzNext= interp1(tPhases, stepWayPoints, phaseNext, 'spline');
        
%         y = yz(1); z = yz(2);
%         v = (yzNext-yzLast)/dt;
%         a = (yzNext + yzLast - 2*yz)/(dt^2);
        
        xyz = xyzStep0 + rotPlanePoint;
 
        scatter3(xyz(1),xyz(2),xyz(3), 'r');
%         quiver(y,z,v(1)*dt, v(2)*dt, 'b')
%         quiver(y,z,a(1)*dt*dt, a(2)*dt*dt, 'g')
    else % stance phase
        %       y = a - 2*a*(t/(2*pi - 2*pi*fractionStep) - 2*pi*fractionStep/(2*pi - 2*pi*fractionStep));
%         xyz(2)= stepWayPoints(end,1) - ...
%             diff(stepWayPoints([1 end],1))*(t/(2*pi - 2*pi*fractionStep) - 2*pi*fractionStep/(2*pi - 2*pi*fractionStep));
        xyz = R*(stepWayPoints(end,:) - ...
              diff(stepWayPoints([1 end],:))...
              *(t/(2*pi - 2*pi*fractionStep) - 2*pi*fractionStep/(2*pi - 2*pi*fractionStep))).';
        
%         xyz(3) = xyzStep0(3);

        scatter3(xyz(1),xyz(2),xyz(3),'k' );
    end
    
    
    pause(.05);
end
end