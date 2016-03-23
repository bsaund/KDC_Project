function SMData = makeSMData(params)
% SMData contains information for FK that doesnt change.

W = params.W; % chassis width
L = params.L; % chassis length
% l = [.02 .06*2 .05+.06*2 .1+.06*1 0]; % link lengths
l1 = params.l(1); l2 = params.l(2);
l3 = params.l(3); l4 = params.l(4); l5 = params.l(5);
r1 = params.r(1); r2 = params.r(2);
r3 = params.r(3); r4 = params.r(4); r5 = params.r(5);

% frames for leg attachement sites
gs = zeros(4,4,6);
gs(:,:,1) = [R_y(deg2rad(1.5)) [W/2; L/2; 0]; [0 0 0 1]];
gs(:,:,2) = [R_y(deg2rad(-1.5)) [-W/2; L/2; 0]; [0 0 0 1]];
gs(:,:,3) = [R_y(deg2rad(1.5)) [W/2; 0; 0]; [0 0 0 1]];
gs(:,:,4) = [R_y(deg2rad(-1.5)) [-W/2; 0; 0]; [0 0 0 1]];
gs(:,:,5) = [R_y(deg2rad(1.5)) [W/2; -L/2; 0]; [0 0 0 1]];
gs(:,:,6) = [R_y(deg2rad(-1.5)) [-W/2; -L/2; 0]; [0 0 0 1]];
% frames for feet at zero config, as seen in s frame
gst_0 = zeros(4,4,6);
gst_0(:,:,1) = [eye(3) [ l1+l2+l3+l4; 0; -l5]; [0 0 0 1]];
gst_0(:,:,2) = [eye(3) [-l1-l2-l3-l4; 0; -l5]; [0 0 0 1]];
gst_0(:,:,3) = gst_0(:,:,1);
gst_0(:,:,4) = gst_0(:,:,2);
gst_0(:,:,5) = gst_0(:,:,1);
gst_0(:,:,6) = gst_0(:,:,2);
% frames for joints 2 and joints 3 (location of joint 1 is fixed)
gsj2_0 = zeros(4,4,6);
gsj2_0(:,:,1) = [eye(3) [ l1+l2; 0; 0]; [0 0 0 1]];
gsj2_0(:,:,2) = [eye(3) [-l1-l2; 0; 0]; [0 0 0 1]];
gsj2_0(:,:,3) = gsj2_0(:,:,1);
gsj2_0(:,:,4) = gsj2_0(:,:,2);
gsj2_0(:,:,5) = gsj2_0(:,:,1);
gsj2_0(:,:,6) = gsj2_0(:,:,2);
gsj3_0 = zeros(4,4,6);
gsj3_0(:,:,1) = [eye(3) [ l1+l2+l3; 0; 0]; [0 0 0 1]];
gsj3_0(:,:,2) = [eye(3) [-l1-l2-l3; 0; 0]; [0 0 0 1]];
gsj3_0(:,:,3) = gsj3_0(:,:,1);
gsj3_0(:,:,4) = gsj3_0(:,:,2);
gsj3_0(:,:,5) = gsj3_0(:,:,1);
gsj3_0(:,:,6) = gsj3_0(:,:,2);
% frames for COM of link 2, 3, 4
gsl2_0 = zeros(4,4,6);
gsl2_0(:,:,1) = [eye(3) [ l1+r2; 0; 0]; [0 0 0 1]];
gsl2_0(:,:,2) = [eye(3) [-l1-r2; 0; 0]; [0 0 0 1]];
gsl2_0(:,:,3) = gsl2_0(:,:,1);
gsl2_0(:,:,4) = gsl2_0(:,:,2);
gsl2_0(:,:,5) = gsl2_0(:,:,1);
gsl2_0(:,:,6) = gsl2_0(:,:,2);
gsl3_0 = zeros(4,4,6);
gsl3_0(:,:,1) = [eye(3) [ l1+l2+r3; 0; 0]; [0 0 0 1]];
gsl3_0(:,:,2) = [eye(3) [-l1-l2-r3; 0; 0]; [0 0 0 1]];
gsl3_0(:,:,3) = gsl3_0(:,:,1);
gsl3_0(:,:,4) = gsl3_0(:,:,2);
gsl3_0(:,:,5) = gsl3_0(:,:,1);
gsl3_0(:,:,6) = gsl3_0(:,:,2);
gsl4_0 = zeros(4,4,6);
gsl4_0(:,:,1) = [eye(3) [ l1+l2+l3+r4; 0; -r5]; [0 0 0 1]];
gsl4_0(:,:,2) = [eye(3) [-l1-l2-l3-r4; 0; -r5]; [0 0 0 1]];
gsl4_0(:,:,3) = gsl4_0(:,:,1);
gsl4_0(:,:,4) = gsl4_0(:,:,2);
gsl4_0(:,:,5) = gsl4_0(:,:,1);
gsl4_0(:,:,6) = gsl4_0(:,:,2);


% joint twists, in s frames
% xi = zeros(6,3,6); % 6 elements in a twist x 3 joints per leg x 6 legs
% q is a point on the joint axis at zero config
%   relative to s frames
q = zeros(3,3,6);
q(:,1,1) = [l1 0 0].';
q(:,2,1) = [l1+l2 0 0].';
q(:,3,1) = [l1+l2+l3 0 0].';
q(:,:,3) = q(:,:,1);
q(:,:,5) = q(:,:,1);
q(:,:,2) = -q(:,:,1);
q(:,:,4) = -q(:,:,1);
q(:,:,6) = -q(:,:,1);
% omega is the joint axis direction
w = zeros(3,3,6);
w(:,1,1) = [0 0 1].';
w(:,2,1) = [0 1 0].';
w(:,3,1) = [0 1 0].';
w(:,:,2) = w(:,:,1);
w(:,:,3) = w(:,:,1);
w(:,:,4) = w(:,:,1);
w(:,:,5) = w(:,:,1);
w(:,:,6) = w(:,:,1);
xi = [-cross(w,q,1); w];


SMData.xi = xi; 
SMData.gs = gs;
SMData.gst_0 = gst_0;
SMData.gsj2_0 = gsj2_0;
SMData.gsj3_0 = gsj3_0;
SMData.gsl2_0 = gsl2_0;
SMData.gsl3_0 = gsl3_0;
SMData.gsl4_0 = gsl4_0;

end