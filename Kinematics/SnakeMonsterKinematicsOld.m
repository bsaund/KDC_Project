classdef SnakeMonsterKinematics < handle
%SNAKEMONSTERKINEMATICS get a kinematics object for snake monster
%
% Documentation is lax because this is just for a KDC project.
    
    methods(Access = public)
        function this = SnakeMonsterKinematics()
        %SNAKEMONSTERKINEMATICS constructor, no arguments necessary
            this.lfLeg = this.getLeg();
            this.lmLeg = this.getLeg();
            this.lbLeg = this.getLeg();
            this.rfLeg = this.getLeg();
            this.rmLeg = this.getLeg();
            this.rbLeg = this.getLeg();
            
            %Set base frame of each joint
            %Note: There is a 1.5 degree rotation
            width = .093; % based on reality
            length = .097;
%             R = this.roty(1.5*pi/180);
            offset = 1.5*pi/180;
            
            this.rfLeg.setBaseFrame(...
                this.trans([width,length,0,pi/2+offset,0,pi/2]) * ...
                this.rotz(pi));
            this.rmLeg.setBaseFrame(...
                this.trans([width,0,0,pi/2+offset,0,pi/2]) * ...
                this.rotz(pi));
            this.rbLeg.setBaseFrame(...
                this.trans([width,-length,0,pi/2+offset,0,pi/2]) * ...
                this.rotz(pi));
            this.lfLeg.setBaseFrame(...
                this.trans([-width,length,0,-pi/2-offset,0,pi/2]));
            this.lmLeg.setBaseFrame(...
                this.trans([-width,0,0,-pi/2-offset,0,pi/2]));
            this.lbLeg.setBaseFrame(...
                this.trans([-width,-length,0,-pi/2-offset,0,pi/2]));

        end
        
        function kin = getLeg(this)
        %Gets a HebiKinematics object for a leg
            
% %             dir = 1;
% %             if(nargin>1 && invertLeg)
% %                 dir = -1;
% %             end
% %             links = {{'FieldableElbowJoint'},
% %                      {'FieldableElbowJoint'},
% %                      {'FieldableStraightLink', 'ext1', .063-.0122, 'twist', pi/2},
% %             %Straight links have a base length of 0.0122
% %                      {'FieldableElbowJoint'},
% %                      {'FieldableElbowLink', ...
% %                       'ext1', 0.0463 - 0.0360, 'twist1', dir*pi/2, ...
% %                       'ext2', 0.046 - 0.0336, 'twist2', pi},
% %             %HEBI kinematics defaults to the elbow joint having
% %             %0.0336m on a side. Ours (apparently) has 0.046
% %                      {'FieldableStraightLink', 'ext1', .092+.022-0.0122, 'twist', 0},
% %             %Desired length + part of foot (for pretty graphing)
% %             % - base length of straight link 
% %                      {'Foot', 'ext1', 0.025, 'twist', 0}};
% %             %Don't compensate for base length of straight link
% %             %here because the sphere foot will not go all the way 
% %             %to the endpoint
            kin = HebiKinematics();
            kin.addBody('FieldableElbowJoint');
            kin.addBody('FieldableElbowJoint');
            kin.addBody('FieldableStraightLink', 'ext1', .063-.0122, ...
                        'twist', pi/2);
            kin.addBody('FieldableElbowJoint');
            kin.addBody('FieldableElbowLink', ...
                        'ext1', 0.0463 - 0.0360, 'twist1', -pi/2, ...
                        'ext2', 0.1849 - 0.0336, 'twist2', pi);
            
            
        end

        function positions = getLegPositions(this, angles)
        %Gets the xyz positions of each leg in the body frame
        %angles is a 18 element vector of joint angles
        %positions is a 3x6 matrix
        
            fk = zeros(4,4,6);
            fk(:,:,1) = this.rfLeg.getFK('EndEffector', angles(1:3));
            fk(:,:,2) = this.lfLeg.getFK('EndEffector', angles(4:6));
            fk(:,:,3) = this.rmLeg.getFK('EndEffector', angles(7:9));
            fk(:,:,4) = this.lmLeg.getFK('EndEffector', angles(10:12));
            fk(:,:,5) = this.rbLeg.getFK('EndEffector', angles(13:15));           
            fk(:,:,6) = this.lbLeg.getFK('EndEffector', angles(16:18));
            
            positions = squeeze(fk(1:3, 4, :));
        end
        
        function CoMs = getCenterOfMasses(this, angles)
        %Gets the xyz positions of the COM of each joint in each leg in the body frame
        %angles is a 18 element vector of joint angles
        %positions is a 3x6 matrix
        
            CoMs = zeros(3,5,6);
            CoMs(:,:,1) = this.getXYZ(this.rfLeg.getFK('CoM', angles(1:3)));
            CoMs(:,:,2) = this.getXYZ(this.lfLeg.getFK('CoM', angles(4:6)));
            CoMs(:,:,3) = this.getXYZ(this.rmLeg.getFK('CoM', angles(7:9)));
            CoMs(:,:,4) = this.getXYZ(this.lmLeg.getFK('CoM', angles(10:12)));
            CoMs(:,:,5) = this.getXYZ(this.rbLeg.getFK('CoM', angles(13:15)));           
            CoMs(:,:,6) = this.getXYZ(this.lbLeg.getFK('CoM', angles(16:18)));
            
        end
        
        function masses = getLegMasses(this)
          %Returns the masses of all of the segments in the legs
          masses = zeros(5,6);
          masses(:,1) = this.rfLeg.getBodyMasses();
          masses(:,2) = this.lfLeg.getBodyMasses();
          masses(:,3) = this.rmLeg.getBodyMasses();
          masses(:,4) = this.lmLeg.getBodyMasses();
          masses(:,5) = this.rbLeg.getBodyMasses();
          masses(:,6) = this.lbLeg.getBodyMasses();
          
        end
        
        function SnakeMonsterCoM = getSnakeMonsterCoM(this,angles)
            %Gets the xyz positions of the COM of the entire Snake Monster in
            %its body frame

            %Pulling the masses and CoMs of the individual leg components
            %(aka bodies)
            bodyMasses = getLegMasses(this);
            bodyCoMs = getCenterOfMasses(this,angles);

            legDim = size(bodyMasses,2);

            %Preallocating memory
            scaledCoMs = zeros(3,legDim);

            %Calculating the Center of Mass of the legs
            for i=1:legDim
              scaledCoMs(:,i) = bodyCoMs(:,:,i)*bodyMasses(:,i);
            end
            legMasses = sum(bodyMasses,1);
            legCoMs = scaledCoMs./repmat(legMasses,[3,1]);

            scaledLegsCoM = sum(legCoMs*legMasses',2);

            legsMass = sum(legMasses);

            %Changed notation from bodyMass to baseMass to remove
            %confusion regarding individual components and the base
            baseMass = 2.37;  %Mass of Snake Monster base only [kg]
            baseCoM = [0;0;0]; %CoM of Snake Monster base only

            %Calculating entire SnakeMonsterCoM
            totalMass = legsMass + baseMass;
            SnakeMonsterCoM = (scaledLegsCoM+baseMass*baseCoM)/totalMass;
          
        end
      
        
       
   
        function J = getLegJacobians(this, angles)
        %Gets the jacobian (world frame) of all the legs 
        %angles is a 18 element vector of joint angles
        
            J = zeros(6,3,6);
            J(:,:,1) = (this.rfLeg.getJacobian('EndEffector', angles(1:3)));
            J(:,:,2) = (this.lfLeg.getJacobian('EndEffector', angles(4:6)));
            J(:,:,3) = (this.rmLeg.getJacobian('EndEffector', angles(7:9)));
            J(:,:,4) = (this.lmLeg.getJacobian('EndEffector', angles(10:12)));
            J(:,:,5) = (this.rbLeg.getJacobian('EndEffector', angles(13:15)));           
            J(:,:,6) = (this.lbLeg.getJacobian('EndEffector', angles(16:18)));
            
        end
        
        function gravCompTorques = getLegGravCompTorques(this, angles, gravity)
        %Gets the gravity compensation torques for the legs
        %angles is a 18 element vector of joint angles
        % gravity is a 1x3 vector which says which way gravity is pointing.
        % returns a 3x6 where each colum is torques for a leg
            gravCompTorques = zeros(3,6);
            gravCompTorques(:,1) = (this.rfLeg.getGravCompTorques(angles(1:3),gravity));
            gravCompTorques(:,2) = (this.lfLeg.getGravCompTorques(angles(4:6),gravity));
            gravCompTorques(:,3) = (this.rmLeg.getGravCompTorques(angles(7:9),gravity));
            gravCompTorques(:,4) = (this.lmLeg.getGravCompTorques(angles(10:12),gravity));
            gravCompTorques(:,5) = (this.rbLeg.getGravCompTorques(angles(13:15),gravity));           
            gravCompTorques(:,6) = (this.lbLeg.getGravCompTorques(angles(16:18),gravity));
            
      end
      
        function angles = getIK(this, xd)
        %Gets the joint angles to position the feed at xd
        %xd is a 3x6 matrix of desired potiions
        %angles is a 18 element vector of joint angles
            angles(1:3) = this.rfLeg.getIK('xyz', xd(:,1));
            angles(4:6) = this.lfLeg.getIK('xyz', xd(:,2));
            angles(7:9) = this.rmLeg.getIK('xyz', xd(:,3));
            angles(10:12) = this.lmLeg.getIK('xyz', xd(:,4));
            angles(13:15) = this.rbLeg.getIK('xyz', xd(:,5));
            angles(16:18) = this.lbLeg.getIK('xyz', xd(:,6));
        end
    end
    
    methods(Access = private, Hidden = true)
        function m = trans(this, xyzrpy)
            m = eye(4);
            m(1:3, 4) = xyzrpy(1:3);
            m = m*this.rotz(xyzrpy(6)) *this.roty(xyzrpy(5))*...
                this.rotx(xyzrpy(4));
        end
        
        function m = roty(this, theta)
        %Homogeneous transform matrix for a rotation about y
            m = [cos(theta),  0, sin(theta), 0;
                 0,           1, 0,          0;
                 -sin(theta), 0, cos(theta), 0;
                 0,           0, 0,          1];
        end
        
        function m = rotx(this, theta)
        %Homogeneous transform matrix for a rotation about y
            m = [1,  0,          0,          0;
                 0,  cos(theta),-sin(theta), 0;
                 0,  sin(theta), cos(theta), 0;
                 0,           0, 0,          1];
        end
        
        function m = rotz(this, theta)
        %Homogeneous transform matrix for a rotation about y
            m = [cos(theta), -sin(theta), 0, 0;
                 sin(theta),  cos(theta), 0, 0;
                 0          , 0, 1,          0;
                 0,           0, 0,          1];
        end
        
        function xyz = getXYZ(this, g)
        %returns the elements for the xyz of the homogeneous tranform g in
        %a 3 by N (where N = size(g,3)) matrix
           xyz = reshape(g(1:3,4,:), [3, size(g,3)]);
        end
        
        
    end
    
    methods(Static,Access = private, Hidden = true)
        function [baseMass,baseCoM] = getBaseMass()
          %Returns the mass of the body only (CoM pretty insignificant
          baseMass = 2.37; % Mass of the body only [kg]
          baseCoM = [0;0;0]; % CoM of Snake Monster is essentially at
                             % the body frame origin
        end
    end
    
    properties(Access = private, Hidden = true)
        lfLeg;
        lmLeg;
        lbLeg;
        rfLeg;
        rmLeg;
        rbLeg;
   
    end
end