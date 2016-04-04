function [xyz] = minJerkStepGait3(stepWayPoints, jerkCoeffs, xyzStep0, RFromPlane, fractionStep, t)
% minJerkStepGait3(stepWayPoints, jerkCoeffs, xyzStance0(:,leg), RFromPlane, fractionStep, t_leg(i));
        
% minumum jerk step waypoint generator
% now with a direction
% now with tilt plane

  t= mod(t,2*pi);
              phase = jerkCoeffs(4)*t.^3 + ...
                    jerkCoeffs(5)*t.^4 + ...
                    jerkCoeffs(6)*t.^5;
% making an open loop step pattern
if (t>=0 && t<2*pi*fractionStep) % swing phase
     planePoint= interp1(linspace(0,1,size(stepWayPoints,1)), stepWayPoints, phase, 'spline').';
     rotPlanePoint = RFromPlane*planePoint;
     
 xyz = xyzStep0 + rotPlanePoint;
 
else % stance phase
    
        xyz = RFromPlane*(stepWayPoints(end,:) - ...
              diff(stepWayPoints([1 end],:))...
              *(t/(2*pi - 2*pi*fractionStep) - 2*pi*fractionStep/(2*pi - 2*pi*fractionStep))).'...
              + xyzStep0;
          
end

end