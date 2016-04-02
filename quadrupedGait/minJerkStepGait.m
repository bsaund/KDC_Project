function [y,z] = minJerkStepGait(stepWayPoints, jerkCoeffs, y0, z0,fractionStep, t)

  t= mod(t,2*pi);
              phase = jerkCoeffs(4)*t.^3 + ...
                    jerkCoeffs(5)*t.^4 + ...
                    jerkCoeffs(6)*t.^5;
                
% making an open loop step pattern
if (t>=0 && t<2*pi*fractionStep) % swing phase
     yz= interp1(linspace(0,1,size(stepWayPoints,1)), stepWayPoints, phase, 'spline');
     
    y = yz(1) + y0;
    z = yz(2) + z0;
else % stance phase
    
     y = stepWayPoints(end,1) - ...
         diff(stepWayPoints([1 end],1))*(t/(2*pi - 2*pi*fractionStep) - 2*pi*fractionStep/(2*pi - 2*pi*fractionStep)) +y0;
     z = z0;
end

end