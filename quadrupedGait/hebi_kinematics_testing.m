% hebi_kinematics_testing
addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));

% lengths, from Simon's "SMLegKinematics.m"
moduleProxLength = .0254 *1.441; % m, length of the proximal end of module
moduleDistLength = .0254 *1.075; % m, length of the distal end of module
staticLength1 =  .0254 *2.480; % m, length of the thigh static link 
staticLength2 =  .0254 *1.823; % m, length of the shin-outpointing static link 
staticLength3 =  .0254 *5.418; % m, length of the shin-downpointing static link 
footLength =  .0254 *1.863; % m, length of the foot
params.W = .093*2;
params.L = .097*2;

% chassis_bend= 1.5; % degrees
chassis_bend= 0; % degrees
baseTransform1 = [R_y(deg2rad(90-chassis_bend)) [params.W/2; params.L/2; 0]; [0 0 0 1]];
% eye(4); % transformation from the origin to the leg frame

       inch2m = 0.0254;
       kin = HebiKinematics();
       kin.setBaseFrame(baseTransform1);
       kin.addBody('FieldableElbowJoint');
       kin.addBody('FieldableElbowJoint');
       kin.addBody('FieldableStraightLink', ...
           'ext', staticLength1, 'twist', -pi/2);
       kin.addBody('FieldableElbowJoint');
       kin.addBody('FieldableElbowLink', ...
           'ext1', staticLength2, 'twist1', 0, ...
           'ext2', staticLength3, 'twist2', 0);
       
       % test FK
       th = zeros(1,3);
       frames = kin.getForwardKinematics('output', th);
       CoMs = kin.getForwardKinematics('CoM', th);
       effector = kin.getForwardKinematics('EndEffector', th);
       
       % test IK
       xyz = [.1 0 0];
       th_IK = kin.getInverseKinematics('xyz', xyz);

       
       