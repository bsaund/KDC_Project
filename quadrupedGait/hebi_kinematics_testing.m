% hebi_kinematics_testing
       
% lengths, from Simon's "SMLegKinematics.m"
moduleProxLength = .0254 *1.441; % m, length of the proximal end of module
moduleDistLength = .0254 *1.075; % m, length of the distal end of module
staticLength1 =  .0254 *2.480; % m, length of the thigh static link 
staticLength2 =  .0254 *1.823; % m, length of the shin-outpointing static link 
staticLength3 =  .0254 *5.418; % m, length of the shin-downpointing static link 
footLength =  .0254 *1.863; % m, length of the foot

       inch2m = 0.0254;
       kin = HebiKinematics();
       kin.addBody('FieldableElbowJoint');
       kin.addBody('FieldableElbowJoint');
       kin.addBody('FieldableStraightLink', ...
           'ext', staticLength1, 'twist', -pi/2);
       kin.addBody('FieldableElbowJoint');
       kin.addBody('FieldableElbowLink', ...
           'ext1', staticLength2, 'twist1', pi/2, ...
           'ext2', staticLength3, 'twist2', pi);
       
       th = zeros(1,3);
       frames = kin.getForwardKinematics('output', th);
       CoMs = kin.getForwardKinematics('CoM', th);
       effector = kin.getForwardKinematics('EndEffector', th);