classdef SnakeMonsterPose < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    methods(Access = public)
        function this = SnakeMonsterPose()
            %SNAKEMONSTERPOSE constructor
            %Arguments:
            %fbk - feedback from HebiGroup (Pass in fbk or entire group?)
            
            this.kin = SnakeMonsterKinematics();
                        
            this.firstRun = true;
            
            
        end
            
        function gravityVector = getGravityVector(this)
            if this.firstRun
                this.oldTime = this.fbk.time;
                this.lastDCM = [];
                %Static legBases function
                this.legBases = this.getSMLegBases;
            end               
            
            %Assumes the body center stays at the orign
            UpVector = [0;0;1];

            snakeMonsterCF(this);
            
            %Gravity vector points in the direction of gravity not in the
            %direction of the opposing acceleration as usually transmitted
            %from the accelerometers
            gravityVector = this.lastDCM*UpVector;
            this.firstRun = false;
        end
        
        function floorNormalVector = getFloor(this,fbk)
           
            this.fbk = fbk;
            
            %Calls function for generating gravity vector with a
            %complimentary filter on the inner 6 modules
            this.gravityVector = getGravityVector(this);
            this.CoM = this.kin.getSnakeMonsterCoM(fbk.position);
            
            %Running the kinematics class with the feedback angles to get
            %the end effector (EE) positions of the legs
            EE = this.kin.getLegPositions(this.fbk.position);
            
            %% Distance Option
%             contactThreshold = 0.05; %Five millimeter contact threshold
%             %Evaluating the distance of each leg tip from the plane normal
%             %to the gravity vector
%             %http://mathworld.wolfram.com/Point-PlaneDistance.html
%             distance = zeros(6,1);
%             for i=1:6
%                 distance(i) = dot(this.gravityVector,EE(1:3,i))/norm(this.gravityVector);
%             end
%             
%             %Sorting the distances in descending order (max - > min)
%             [sorteddist,index] = sort(distance,'descend');
%             
%             %Number of contacts = Three required points + any other legs
%             %within the contact threshold of the last leg
%             %This allows for 4,5,6 leg stances instead of defaulting to 3
%             contacts = 3+sum(sorteddist(4:6)>=sorteddist(3)-contactThreshold);
% 
%             %Combining distance sorting with orientation of the legs to get
%             %correct order of support polygon instead of overlapping lines
%             sortingTable = [1 3 5 6 4 2;1 2 3 4 5 6]';
%             legOrder = sortrows(sortingTable);
%             priority = [1 2 3 4 5 6]';
%             priorityTable = [index priority];
%             legPriority = sortrows(priorityTable);
%             sortingTable = [legPriority(:,2) legOrder];
%             distanceOrder = sortrows(sortingTable);
%             
%             %Zero out any contacts not touching the ground
%             if contacts<6
%                 distanceOrder(contacts+1:end,2) = zeros(6-contacts,1);
%             end
%             
%             %Reorder the columns so that the thresholded values can be
%             %reordered based on their orientation on the snake monster
%             distanceOrder(:,[1 3])=distanceOrder(:,[3 1]);
%             filteredSortingTable = sortrows(distanceOrder);
%             
%             polygon = filteredSortingTable(:,2);
%             filtPoly = polygon(polygon~=0);
%             polyIndex = filtPoly;
            %% End Distance Option/ Start Torque Option
            torque = zeros(6,1);
            for k = 1:6
                p = (k-1)*3+2;
                torque(k) = this.fbk.torque(p);
            end
            
            torqueIndex = zeros(6,1);
            for i = 1:6
                sorting = [1 3 5 6 4 2]';
                if torque(sorting(i))<-0.5
                    torqueIndex(i)=sorting(i);
                end
            end
            polyIndex = torqueIndex(find(torqueIndex));
            contacts = length(polyIndex);

            %% End Torque Option

            %Grabbing the relative contact positions
             point = zeros(3,contacts);
             for k=1:contacts
                 point(:,k) = EE(:,polyIndex(k));
             end
            
            switch contacts
                case 3
                    supPoly = [EE(:,polyIndex(1)),EE(:,polyIndex(2)),...
                        EE(:,polyIndex(3))];
                    
                    %Using the three contact points to get vectors
                    vector12 = point(:,2)-point(:,1);
                    vector13 = point(:,3)-point(:,2);

                    %Generating the normal vector to the plane of interest
                    %(the floor) using the cross product
                    perpVector =  cross(vector13,vector12);
                    floorNormalVector = perpVector/norm(perpVector);
                case 4
                    supPoly = [EE(:,polyIndex(1)),EE(:,polyIndex(2)),...
                        EE(:,polyIndex(3)),EE(:,polyIndex(4))];
                    
                    relPoints = zeros(3,contacts);
                    for i=1:contacts
                        relPoints(1:3,i) = EE(:,polyIndex(i));
                    end
                    floorNormalVector = this.findPlaneFromPoints(relPoints);                   
                    
                case 5
                    supPoly = [EE(:,polyIndex(1)),EE(:,polyIndex(2)),...
                        EE(:,polyIndex(3)),EE(:,polyIndex(4)),EE(:,polyIndex(5))];
                    
                    relPoints = zeros(3,contacts);
                    for i=1:contacts
                        relPoints(1:3,i) = EE(:,polyIndex(i));
                    end
                    floorNormalVector = this.findPlaneFromPoints(relPoints);                  
                    
                case 6
                    supPoly = [EE(:,polyIndex(1)),EE(:,polyIndex(2)),...
                        EE(:,polyIndex(3)),EE(:,polyIndex(4)),...
                        EE(:,polyIndex(5)),EE(:,polyIndex(6))];
                    
                    relPoints = zeros(3,contacts);
                    for i=1:contacts
                        relPoints(1:3,i) = EE(:,polyIndex(i));
                    end
                    floorNormalVector = this.findPlaneFromPoints(relPoints); 
            end
            
            cenOfPlane = mean(supPoly,2);
            
            %Insuring that the floor normal vector is always in the
            %opposite direction as the gravity vector
            floorNormalVector = sign(dot(floorNormalVector,...
                this.gravityVector))*floorNormalVector;

            projPoints = this.projectPointsToPlane(cenOfPlane,...
                floorNormalVector,supPoly);
            
            
            %Plane equation:
            % http://ocw.mit.edu/courses/mathematics/18-02sc-multivariable-calculus-fall-2010/1.-vectors-and-matrices/part-c-parametric-equations-for-curves/session-16-intersection-of-a-line-and-a-plane/MIT18_02SC_we_9_comb.pdf
            % https://www.cs.princeton.edu/courses/archive/fall00/cs426/lectures/raycast/sld017.htm
            % normal(1)*(x-point(1,1))+normal(2)*(y-point(1,2)+normal(3)*(z-point(1,3))=0
            
            t = -dot(this.CoM,floorNormalVector)+dot(floorNormalVector,cenOfPlane)...
                /dot(this.gravityVector,floorNormalVector);
            centerOfPressure = this.CoM+this.gravityVector*t;
            
            upVector = [0;0;1];
            
            floorNormalUnitVector = floorNormalVector/norm(floorNormalVector);
            
            v = cross(floorNormalUnitVector,upVector);
            vhat = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
            s = norm(v);
            c = dot(floorNormalUnitVector,upVector);
            
            %http://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
            R = eye(3)+vhat+vhat^2*((1-c)/(s^2));
            
            planarXYZPoints = R*projPoints;
            planarXYPoints = planarXYZPoints(1:2,:);
            convexHullIndex = convhull(planarXYPoints(1,:),planarXYPoints(2,:));
            
            convexProjPoints = projPoints(:,convexHullIndex);
            
            orthoVectors = zeros(3,size(convexProjPoints,2)-1);
            prox = zeros(size(convexProjPoints,2)-1,1);
            for i = 1:size(convexProjPoints,2)-1
                %Clockwise rotation
                supportVector = convexProjPoints(:,i)-convexProjPoints(:,i+1);
                orthoVectors(:,i) = cross(floorNormalVector,supportVector);
                
                %http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
                prox(i) = norm(cross(supportVector,convexProjPoints(:,i)-...
                    centerOfPressure))/norm(supportVector);
            end
            
            this.orthogonalVectors = orthoVectors;
            this.proximity = prox;
            this.convexProjectedPoints = convexProjPoints;
            this.supportPolygon = supPoly;
            this.floorNormal = floorNormalVector;
            this.CoP = centerOfPressure;
        end
        
        function supportPolygon = getSupportPolygon(this,fbk)
            getFloor(this,fbk);
            supportPolygon = this.supportPolygon;
        end
        
        function centerOfPressure = getCenterOfPressure(this,fbk)
            getFloor(this,fbk);
            centerOfPressure = this.CoP;
        end
        
        function centerOfMass = getCenterOfMass(this,fbk)
            getFloor(this,fbk);
            centerOfMass = this.CoM;
        end
        
        function [orthogonalVectors,proximity] = getDistancesToSupportPoly(this,fbk)
            
            getFloor(this,fbk);
            
            orthogonalVectors = this.orthogonalVectors;
            proximity = this.proximity;

        end

        
        function [] = plotEnvironment(this,fbk)
            if this.firstRun
                hold on;
                %this.normalPlot = plot3(0,0,0,'r');
                this.gravPlot = plot3(0,0,0,'g','LineWidth',3);
                %this.polyPlot = plot3(0,0,0,'r--','LineWidth',5);
                this.planePlot = plot3(0,0,0,'k--','LineWidth',3);
                this.CoPPlot = plot3(0,0,0,'b*','MarkerSize',15,'LineWidth',2);
                this.CoMPlot = plot3(0,0,0,'k*','MarkerSize',10,'LineWidth',1.5);
                this.OrPlot = plot3(0,0,0,'r','LineWidth',2);
            end
            vectorScale = 0.075;
            this.getFloor(fbk);
            
            orthoVector = this.orthogonalVectors;
            
            CenterOfMass = this.CoM;
            CenterOfPressure = this.CoP;
            
            oP = [];
            for i = 1:size(orthoVector,2)
                oP = [oP,CenterOfPressure,CenterOfPressure+this.proximity(i)*orthoVector(:,i)/norm(orthoVector(:,i))];
            end

            gV = [CenterOfMass,CenterOfMass-vectorScale*this.gravityVector];
            sp = this.supportPolygon;
            pP = this.convexProjectedPoints;
            cp = this.CoP;
            cm = this.CoM;
            fN = [CenterOfPressure,CenterOfPressure+vectorScale*this.floorNormal];

            
            set(this.gravPlot,'xdata',gV(1,:),'ydata',gV(2,:),'zdata',gV(3,:))
            %set(this.polyPlot,'xdata',sp(1,:),'ydata',sp(2,:),'zdata',sp(3,:))
            set(this.planePlot,'xdata',pP(1,:),'ydata',pP(2,:),'zdata',pP(3,:))
            set(this.CoPPlot,'xdata',cp(1),'ydata',cp(2),'zdata',cp(3))
            set(this.CoMPlot,'xdata',cm(1),'ydata',cm(2),'zdata',cm(3))
            set(this.OrPlot,'xdata',oP(1,:),'ydata',oP(2,:),'zdata',oP(3,:))
            %set(this.normalPlot,'xdata',fN(1,:),'ydata',fN(2,:),'zdata',fN(3,:))
            
        end
        %getGravityFrame     %Would need to have a base frame adj for SMKin
    end
    
    methods(Access=private, Hidden=true)
        function this = snakeMonsterCF(this)
        % SNAKEMONSTERCF A complimentary filter for the snake monster robot.  This 
        % uses the IMUs from the 6 modules mounted to the chassis and a
        % complimentary filter to estimate the orientation of the chassis.
        %
        % This works with the new 2.0 Matlab API.
        %
        % Dave Rollinson
        % Dec 2014 (based on older code from snake project)

            %%%%%%%%%%%%%%%
            % SETUP STUFF %
            %%%%%%%%%%%%%%%

            % Number of legs on the chassis
            numLegs = size(this.legBases,3);

            % Weight on accelerometer correction term
            accelWeight = .5;

            accelVecModule = nan(3,numLegs);
            accelVecBody = nan(3,numLegs);

            gyroVecModule = nan(3,numLegs);
            gyroVecBody = nan(3,numLegs);
            
            this.dt = this.fbk.time-this.oldTime;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % AVERAGE THE ACCELEROMETERS + GYROS %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Take the readings from the first module on each leg.
            for i=1:numLegs

                % Accelerometers (g)    %Changed to all pos values
                accelVecModule(1,i) = this.fbk.accelX(i*3-2);
                accelVecModule(2,i) = this.fbk.accelY(i*3-2);
                accelVecModule(3,i) = this.fbk.accelZ(i*3-2);

                accelVecBody(:,i) = this.legBases(1:3,1:3,i) * ...
                                                accelVecModule(:,i);

                % Gyros (rad/s)
                gyroVecModule(1,i) = this.fbk.gyroX(i*3-2);
                gyroVecModule(2,i) = this.fbk.gyroY(i*3-2);
                gyroVecModule(3,i) = this.fbk.gyroZ(i*3-2);
                %Transposed both legBases
                gyroVecBody(:,i) = this.legBases(1:3,1:3,i) * ...
                                                gyroVecModule(:,i);
            end

        %     % Remove any data that's NaN
        %     accelVecBody(isnan(accelVecBody)) = [];
        %     gyroVecBody(isnan(gyroVecBody)) = [];

            % Average accelerometers
            accelVecAvg = mean( accelVecBody, 2 );

            % Average gyros
            gyroVecAvg = mean( gyroVecBody, 2 );


            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % CALCULATE THE ORIENTATION %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % ACCELEROMETER GRAVITY VECTOR
            gravityVec = accelVecAvg / norm(accelVecAvg);
            upVec = [0; 0; 1];

            % ROTATION MATRIX OF CURRENT POSE
            if isempty(this.lastDCM)

                  accelAxis = cross( upVec, gravityVec );
                  accelAxis = accelAxis / norm(accelAxis);

                  accelAngle = rad2deg( acos( dot(upVec,gravityVec) ) );

                  q = SpinCalc( 'EVtoQ', ...
                             [accelAxis', accelAngle], ...
                             1E6, 0 )';
            else
                q = SpinCalc( 'DCMtoQ', this.lastDCM, 1E6, 0 )';
            end

            % ESTIMATE NEW ORIENTATION BY FORWARD INTEGRATING GYROS
            w_x = gyroVecAvg(1);
            w_y = gyroVecAvg(2);
            w_z = gyroVecAvg(3);

            q_update = quat_rotate_better( w_x, w_y, w_z, q, this.dt );    
            orientDCM = SpinCalc('QtoDCM', q_update', 1E6, 0 )';

            %gravityVec
            accelGravity = orientDCM' * gravityVec;

            accelAxis = cross( upVec, accelGravity );
            accelAxis = accelAxis / norm(accelAxis);
            accelAngle = rad2deg(real(acos( dot(upVec,accelGravity) )) );


            % MESS W/ THE ACCEL WEIGHT

            % Scale down if gyro readings are large
            gyroMag = norm(gyroVecAvg);
            gyroScale = 1;

            accelWeight = accelWeight / (1 + gyroScale * gyroMag);

            % Scale down if accelerometers deviate from 1g.
            accelMag = norm(accelVecAvg);
            accelThresh = .05;
            
            accelDev = abs(accelMag - 1.01) > accelThresh;
            
            %IMPORTANT:
            %When the firmware gets updated, use the following accelDev
            %accelDev = abs(accelMag - 9.81) > accelThresh;
           
            if accelDev
                accelWeight = 0;
            else
                accelWeight = accelWeight * (1 - accelDev/accelThresh);
            end

                    
            R_error = SpinCalc( 'EVtoDCM', ...
                          [-accelAxis', accelWeight * accelAngle], ...
                           1E6, 0 );

            updatedDCM = R_error' * orientDCM';

            % Spit out the updated orientation and angular velocities
            this.lastDCM = updatedDCM;
            %Angular Velocity of the Chassis if necessary later
            %chassisAngVel = [ w_x; 
            %                  w_y;
            %                  w_z ];
            
            this.oldTime = this.fbk.time;
        end
    end
    
    methods(Static, Access = private, Hidden = true)
        function [ legBases ] = getSMLegBases()
        %Returns the transforms describing the base of each leg of the snake
        %monster in the body frame
        %
        %  Leg numbering / Coordinate convention:
        %  
        %   2 ----- 1     +y
        %       |          ^
        %   4 ----- 3      |
        %       |          o--> +x
        %   6 ----- 5    +z
        %
        % Dave Rollinson
        % Oct 2014

            legBases = repmat(eye(4),[1,1,6]);

            R_leftLegs = [ 0  0 -1;
                           1  0  0;
                           0 -1  0 ];

            R_rightLegs = [ 0  0  1;
                           -1  0  0;
                            0 -1  0 ]; 

            angle = -degtorad(1.5);
            R_x = [ 1     0       0;
                  0  cos(angle) -sin(angle);
                  0  sin(angle) cos(angle)];            

            % Account for the draft angle of the chassis            
            R_leftLegs = R_leftLegs* R_x;
            R_rightLegs = R_rightLegs * R_x;

            % Orientations            
            legBases(1:3,1:3,1) = R_rightLegs;
            legBases(1:3,1:3,3) = R_rightLegs;
            legBases(1:3,1:3,5) = R_rightLegs;

            legBases(1:3,1:3,2) = R_leftLegs;
            legBases(1:3,1:3,4) = R_leftLegs;
            legBases(1:3,1:3,6) = R_leftLegs;


            % Positions
            legBases(1:3,4,1) = [.093; .097; 0];
            legBases(1:3,4,3) = [.093; 0; 0];
            legBases(1:3,4,5) = [.093; -.097; 0];

            legBases(1:3,4,2) = [-.093; .097; 0];
            legBases(1:3,4,4) = [-.093; 0; 0];
            legBases(1:3,4,6) = [-.093; -.097; 0];
        end
 
        function [] = arrow3(O, D, color, linewidth)
            %ARROW3  plots a 3d arrow
            %
            %	ARROW3(O, D)
            %	ARROW3(O, D, COLOR, LINEWIDTH)
            %
            % Draws an arrow with the origin at O, and in direction D.  Thus, the head
            % of the arrow will be at O + D.

            % $Id: arrow3.m,v 1.1 2009-03-17 16:40:18 bradleyk Exp $
            % CoPyright (C) 2005, by Brad Kratochvil

            if nargin < 3
                color = 'k';
            end

            if nargin < 4
                linewidth = 2;
            end

            if ~isequal([3 1], size(O)) || ~isequal([3 1], size(D))  
              error('SCREWS:arrow3', 'requires 3x1 input vector');
            end

            tail_length = 10;
            tail_width = 2;

            len = norm(D);
            off = len/tail_length;

            if 0 == len,
              return;
            end

            % the arrow points along the x-axis for now
            points = [0 0 0; ...
                      len 0 0; ...
                      len-off -off/tail_width 0; ...
                      len-off off/tail_width 0; ...
                      len 0 0;]';

            % build a rotation matrix to make our lives easier
            R(:,1) = D/len;
            R(:,2) = roty(pi/2)*D/len;
            if (max(sum(R(:, 1:2).^2, 2)) > 1)
              R(:,2) = rotx(pi/2)*D/len;  
            end
            R(:,3) = sqrt(ones(3,1) - R(:, 1).^2 - R(:,2).^2) ;

            % rotate the points
            points = R * points + repmat(O, 1, size(points,2));

            hchek = ishold;

            % plot everything
            plot3(points(1, 1:2), points(2, 1:2), points(3, 1:2), ...
                  'color', color, 'linewidth', linewidth);
            hold on;
            h = patch(points(1, 2:end)', points(2, 2:end)', points(3, 2:end)', color);
            set(h, 'LineStyle', 'none');

            if 0 == hchek
               hold off
            end

        
            function R = rotx(phi)
                %ROTX  rotate around X by PHI
                %
                %	R = ROTX(PHI)
                %
                % See also: ROTY, ROTZ, ROT, POS.

                % $ID$
                % CoPyright (C) 2005, by Brad Kratochvil

                R = [1        0         0; ...
                     0 cos(phi) -sin(phi); ...
                     0 sin(phi)  cos(phi)];

                % this just cleans up little floating point errors around 0 
                % so that things look nicer in the display
                if exist('roundn'),
                  R = roundn(R, -15);
                end

            end
            function R = roty(beta)
                %ROTY  rotate around Y by BETA
                %
                %	R = ROTY(BETA)
                %
                % See also: ROTX, ROTZ, ROT, POS.

                % $ID$
                % CoPyright (C) 2005, by Brad Kratochvil

                R = [cos(beta) 0 sin(beta); ...
                             0 1         0; ...
                    -sin(beta) 0 cos(beta)];

                % this just cleans up little floating point errors around 0 
                % so that things look nicer in the display
                if exist('roundn'),
                  R = roundn(R, -15);
                end
            end
            
        end
        
        function [pointsOnPlane] = projectPointsToPlane(planePoint,planeNormal,points)
            %PROJECTPOINTTOPLANE Projects 3D points in space to a plane
            %with inputs:
            %   planePoint - 3D point on the surface of the plane
            %   planeNormal - Vector normal to the plane surface
            %   points - 3D points to project onto the plane
            % Ky Woodard
            %http://stackoverflow.com/questions/8942950/how-do-i-find-the-orthogonal-projection-of-a-point-onto-a-plane
                        
            [n,m] = size(points);
            
            %Normalizing the plane vector
            if norm(planeNormal)~=1;
                planeNormal = planeNormal/norm(planeNormal);
            end
            
            %Correcting the dimensions of the points if they were brought
            %in transposed from the expected
            if n == 3
                transposed = false;
                num = m;
            elseif m == 3
                points = points';
                transposed = true;
                num = n;
            else
                message = 'points must be of the form 3xn';
                error(message)
            end
            
            %Performing the plane projection
            pointsOnPlane = zeros(3,num);
            for i = 1:num
                
                pointsOnPlane(:,i) = points(:,i) - dot(points(:,i)...
                    -planePoint,planeNormal)*planeNormal;
            end
            %Setting the transposed point matrix back to its original form
            if transposed
                pointsOnPlane = pointsOnPlane';
            end
        end
        
        function [planeNormalVector] = findPlaneFromPoints(points)
            %PROJECTPOINTTOPLANE Projects 3D points in space to a plane
            %with inputs:
            %   planePoint - 3D point on the surface of the plane
            %   planeNormal - Vector normal to the plane surface
            %   points - 3D points to project onto the plane
            % Ky Woodard
            % http://www.mathworks.com/matlabcentral/newsreader/view_thread/60567
            [n,m] = size(points);
            %Correcting the dimensions of the points if they were brought
            %in transposed from the expected
            if n == 3
                num = m;
            elseif m == 3
                points = points';
                num = n;
            else
                message = 'points must be of the form 3xn';
                error(message)
            end
            
            A = [points' ones(num,1)];
            [~,S,V] = svd(A);
            
            singVal = diag(S);
            minIndex = find(singVal==min(singVal),1,'first');
            planeVector = V(:,minIndex);
            
            planeNormalVector = planeVector/norm(planeVector(1:3));
            planeNormalVector = planeNormalVector(1:3);
        end
    end
    
    properties (Access = private,Hidden = true)
    %SnakeMonsterPose
        kin
        fbk
    %getGravityVector
        firstRun
        legBases
        gravityVector
        CoP
        CoM
        convexProjectedPoints
        orthogonalVectors
        proximity
        %Complimentary Filter
            dt
            oldTime
            lastDCM
    %getFloor
        supportPolygon
        floorNormal
    %plotEnvironment
        gravPlot    
        polyPlot
        planePlot
        CoPPlot
        CoMPlot
        OrPlot
        normalPlot
    end
end

