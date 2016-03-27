function [y,z] = ellipticalGait(a,b,y0, z0,fractionStep, t)

  t= mod(t,2*pi);
% making an open loop step pattern
if (t>=0 && t<2*pi*fractionStep) % swing phase
    z = -b*sin(t/fractionStep/2 - pi) + z0;
    y = a*cos(t/fractionStep/2 - pi) + y0;
else % stance phase
     y =a - 2*a*(t/(2*pi - 2*pi*fractionStep) - 2*pi*fractionStep/(2*pi - 2*pi*fractionStep)) +y0;
      z = z0;
end

end