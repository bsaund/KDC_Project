function th = IK(params, SMData, xd)
% calculates the IK for a simple snake monster leg.
 
l1 = params.l(1); l2 = params.l(2); l3 = params.l(3); l4 = params.l(4); l5 = params.l(5);
% l1 = moduleProxLength;
% l2 = moduleDistLength+moduleProxLength;
% l3 = moduleDistLength+staticLength1+moduleProxLength;
% l4 = moduleDistLength+staticLength2;
% l5 = staticLength3+footLength;
th = zeros(3,6);

d = sqrt(l4^2 + l5^2); % the hypotenuse of the shin triangle
delta = atan2(l5,l4); % the angle of the shin triangle

for leg = 1:2:5
% find the points in the s frames
    xd_leg = SMData.gs(:,:,leg)\[xd(:,leg); 1];
% view point from the first joint
    xds = xd_leg - [l1;0;0;0];
    % convert to cylindircal coordinates
radius = sqrt(sum(xds(1:2,:).^2,1)) -l2;
z = xds(3,:);
phi = atan2(xds(2,:), xds(1,:));
th(1,leg) = phi; % first angle sets the cylindrical angle
dist = sqrt(radius.^2 + z.^2);
th(2,leg) = atan2(z,radius) + acos( (dist.^2 + l3^2 - d^2)./(2*l3*dist) );
gamma = pi + acos( (l3^2 + d^2 - dist.^2)/(2*l3*d) );
th(3,leg) = delta+gamma;

th(2,leg) = -th(2,leg);
th(3,leg) = -th(3,leg);

end

for leg = 2:2:6
% find the points in the s frames
    xd_leg = SMData.gs(:,:,leg)\[xd(:,leg); 1];
% view point from the first joint
    xds = xd_leg + [l1;0;0;0];
    xds(1) = -xds(1); % change for flipped axis
    % convert to cylindircal coordinates
radius = sqrt(sum(xds(1:2,:).^2,1)) -l2;
z = xds(3,:);
phi = atan2(xds(2,:), xds(1,:));
th(1,leg) = phi; % first angle sets the cylindrical angle
dist = sqrt(radius.^2 + z.^2);
th(2,leg) = atan2(z,radius) + acos( (dist.^2 + l3^2 - d^2)./(2*l3*dist) );
gamma = pi + acos( (l3^2 + d^2 - dist.^2)/(2*l3*d) );
th(3,leg) = delta+gamma;

end

% %% IK for the legs
% % convert to cylindircal coordinates
% r = sqrt(sum(xd(1:2,:).^2,1))-l1;
% z = xd(3,:);
% phi = atan2(xd(2,:), xd(1,:));
% phi([1 3 5]) = phi([1 3 5])-pi; % flip the axis
% % match the first joint to the cylinder angle
% th(1,:) = phi;
% % in the r,z plane do 2link IK
% d = sqrt(l3^2 + l4^2);
% dist = sqrt(r.^2 + z.^2);
% th(2,:) = atan2(z,r) + acos( (dist.^2 + l2^2 - d^2)./(2*l2*dist) );
% gamma = pi + acos( (l2^2 + d^2 - dist.^2)/(2*l2*d) );
% delta = atan2(l4,l3);
% th(3,:) = delta-gamma;
% th(1,[1 3 5]) = th(1,[1 3 5])*-1;
% th(2,[1 3 5]) = th(2,[1 3 5])*-1;
% th(3,[2 4 6]) = th(3,[2 4 6])*-1;

end