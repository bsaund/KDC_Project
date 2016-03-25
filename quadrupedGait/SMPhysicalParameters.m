function params = SMPhysicalParameters()
% returns a structure with all of the physical parameters for snake
% monster. Uses values found throughout Simon's snakeMonsterWalking code.
% Julian Whitman 3/22/2016

%% masses, from Simon's "setParameters.m"
params.legMasses = [.205 .205 .09 .205 .20 .20];
% mass kg of [hip module, hip 2 module,femur, knee module,tibia(shin),foot]
% Chassis Weight
params.robotMass = 8;  % kg
params.payloadMass = 0.0;  % kg


% lengths, from Simon's "SMLegKinematics.m"
moduleProxLength = .0254 *1.441; % m, length of the proximal end of module
moduleDistLength = .0254 *1.075; % m, length of the distal end of module
params.staticLength1 =  .0254 *2.480; % m, length of the thigh static link 
params.staticLength2 =  .0254 *1.823; % m, length of the shin-outpointing static link 
params.staticLength3 =  .0254 *5.418; % m, length of the shin-downpointing static link 
params.footLength =  .0254 *1.863; % m, length of the foot

l1 = moduleProxLength;
l2 = moduleDistLength+moduleProxLength;
l3 = moduleDistLength+params.staticLength1+moduleProxLength;
l4 = moduleDistLength+params.staticLength2;
l5 = params.staticLength3+params.footLength;

params.l = [l1 l2 l3 l4 l5];
% the distances to the COM from the joints
params.r = [l1/2, l2/2, l3/2, l4/2, l5/2]; % THIS IS A BAD APPROX FIX LATER


%% leg base transformations, from Simon's "getSMLegBases.m"
params.W = .093*2;
params.L = .097*2;

% legBases = repmat(eye(4),[1,1,6]);
% 
%     R_leftLegs = [ 0  0 -1;
%                    1  0  0;
%                    0 -1  0 ];
%               
%     R_rightLegs = [ 0  0  1;
%                    -1  0  0;
%                     0 -1  0 ]; 
%      
%     % Account for the draft angle of the chassis            
%     R_leftLegs = R_leftLegs * R_x(-deg2rad(1.5));
%     R_rightLegs = R_rightLegs * R_x(-deg2rad(1.5));
%       
%     % Orientations            
%     legBases(1:3,1:3,1) = R_rightLegs;
%     legBases(1:3,1:3,3) = R_rightLegs;
%     legBases(1:3,1:3,5) = R_rightLegs;
% 
%     legBases(1:3,1:3,2) = R_leftLegs;
%     legBases(1:3,1:3,4) = R_leftLegs;
%     legBases(1:3,1:3,6) = R_leftLegs;
%     
%     
%     % Positions
%     legBases(1:3,4,1) = [.093; .097; 0];
%     legBases(1:3,4,3) = [.093; 0; 0];
%     legBases(1:3,4,5) = [.093; -.097; 0];
%     
%     legBases(1:3,4,2) = [-.093; .097; 0];
%     legBases(1:3,4,4) = [-.093; 0; 0];
%     legBases(1:3,4,6) = [-.093; -.097; 0];
% 
%     params.legBases = legBases;
end