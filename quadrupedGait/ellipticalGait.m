function [y,z] = ellipticalGait(a,b,y0, z0, t)

  t= mod(t,2*pi);
% making an open loop step pattern
if (t>=0 && t<pi/2) % swing phase
    z = -b*sin(2*t - pi) + z0;
    y = a*cos(2*t - pi) + y0;
else % stance phase
     y = a - 2*a*(t*(2/3/pi) - 1/3) +y0;
      z = z0;
end

end