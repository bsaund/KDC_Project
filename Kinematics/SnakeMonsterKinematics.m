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
            R = this.roty(1.5*pi/180);
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
            

            links = {{'FieldableElbowJoint'},
                     {'FieldableElbowJoint'},
                     {'FieldableStraightLink', 'ext1', .063-.0122, 'twist', pi/2},
            %Straight links have a base length of 0.0122
                     {'FieldableElbowJoint'},
                     {'FieldableElbowLink', ...
                      'ext1', 0.0463 - 0.0360, 'twist1', -pi/2, ...
                      'ext2', 0.046 - 0.0336, 'twist2', pi},
            %HEBI kinematics defaults to the elbow joint having
            %0.0336m on a side. Ours (apparently) has 0.046
                     {'FieldableStraightLink', 'ext1', .092+.022-0.0122, 'twist', 0},
            %Desired length + part of foot (for pretty graphing)
            % - base length of straight link 
                     {'Foot', 'ext1', 0.025, 'twist', 0}};
            %Don't compensate for base length of straight link
            %here because the sphere foot will not go all the way 
            %to the endpoint
            kin = HebiKinematics();
            kin.addBody('FieldableElbowJoint');
            kin.addBody('FieldableElbowJoint');
            kin.addBody('FieldableStraightLink', 'ext1', .063-.0122, ...
                        'twist', pi/2);
            kin.addBody('FieldableElbowJoint');
            kin.addBody('FieldableElbowLink', ...
                        'ext1', 0.0463 - 0.0360, 'twist1', -pi/2, ...
                        'ext2', 0.1849 - 0.0336, 'twist2', pi)
            
            
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