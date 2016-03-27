% parametric test

% making an open loop step pattern
a = .05; % step length = 2*a
b = .05; % step height = b
% time t = 0:2*pi parameter for step length
% one quarter of the time it's in the air, 3/4 on ground.
close all;
figure;
plot(0,0); hold on;
xlim([-a a]*1.5);
ylim([-b b ]*1.5);
fractionStep = 1/2;

for t = 0:.1:2*pi

if (t>=0 && t<2*pi*fractionStep) % swing phase
    z = -b*sin(t/fractionStep/2 - pi);
    y = a*cos(t/fractionStep/2 - pi);
    scatter(y,z);
else % stance phase
      y = a - 2*a*(t/(2*pi - 2*pi*fractionStep) - 2*pi*fractionStep/(2*pi - 2*pi*fractionStep));
      z = 0;
      scatter(y,z,'filled');
end


pause(.05);
end
