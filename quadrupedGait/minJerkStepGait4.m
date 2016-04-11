function [xyz] = minJerkStepGait4(xyzLift, xyzLand, jerkCoeffs, stepHeight, fractionStep, stepPeriod, t)

% minumum jerk step waypoint generator
% takes the starting position of the foot, the end position, and the
% current time. Returns a point on the step trajectory

  t= mod(t,stepPeriod);
      phase = jerkCoeffs(4)*t.^3 + ...
              jerkCoeffs(5)*t.^4 + ...
              jerkCoeffs(6)*t.^5;
          vect = xyzLand-xyzLift; % vector from start to end
          
stepWayPoints = [xyzLift, xyzLift+vect/2+[0;0;stepHeight], xyzLand];
 nPts = size(stepWayPoints, 2); 
          
if (t>=0 && t<stepPeriod*fractionStep) % swing phase
     xyz = interp1(linspace(0,1,nPts), stepWayPoints.', phase, 'spline').';

else % stance phase
    
%         xyz = RFromPlane*(stepWayPoints(end,:) - ...
%               diff(stepWayPoints([1 end],:))...
%               *(t/(2*pi - 2*pi*fractionStep) - 2*pi*fractionStep/(2*pi - 2*pi*fractionStep))).'...
%               + xyzStep0;
          xyz = xyzLand - vect*...
              (t/stepPeriod/(1 - fractionStep) - fractionStep/(1 - fractionStep)).';
          
end

end