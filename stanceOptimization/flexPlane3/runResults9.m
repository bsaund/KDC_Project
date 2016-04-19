% plotOptResultsStatic
addpath(genpath('C:\Users\Julian\Box Sync\CMU sem 1+2\snakeMonster\KDC_project'));
addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));

sendCommands = 1;

nCycles = 1;

% initialize robot
% command structure for if sending commands
if sendCommands
    cmd = CommandStruct();
    cmd.position = nan(1,18);
    cmd.velocity = nan(1,18);
    cmd.torque = nan(1,18);
    setupSnakeMonsterGroup; % makes the snakeMonster hebi group object
    setGainsSMHigh;
    nCycles = 2;
end

close all;
plt = SnakeMonsterPlotter();
set(gcf, 'position', [10 100 700 700]);
plt.plot(zeros(1,18)); hold on;
projectedCOM = scatter3(0,0,0,'k', 'filled');
supportLines = plot3(0,0,0,'k');
planeArrow = quiver3(0,0,0,0,0,0,'b');

kin = SnakeMonsterKinematics();

nStanceLegs = length(stanceLegs);
xyStep = stateOpt(1:2*nStanceLegs);
transforms = stateOpt(2*nStanceLegs+1:end);

xyz = zeros(3,6);
xyz(:,extraLegs) =xyzExtra;
xyz(1:2,stanceLegs) = reshape(xyStep, [2,nStanceLegs]);
% foot positions on the plane are zero.
xyz(3,stanceLegs) = 0;
xyz0 = xyz; % save original configuration for modification

% get the forwards direction vector
stepDirVector = R_z(stepDirection)*[0;1;0];
a_forward = stepLength/2; % params.L/3
a_back = stepLength/2; % step length = a_forward  + a_back
b = .06; % step height = b. .06 works.
fractionStep = 1/nStanceLegs;
swingLegs = zeros(1,6); % 1 indicates leg is in the air
swingLegs(extraLegs) = 2;

nWaypoints = 100;
if sendCommands
    nWaypoints = 50;
end


dt = (2*pi*nCycles-0)/(nWaypoints*nCycles-1);

stepPeriod = 2*pi*fractionStep;  % how long the stepping lasts
% solve for coefficients to create trajectory with min jerk
jerkCoeffs = minimumJerk( 0, 0, 0, ... % Starting Phase/Vel/Accel
    1, 0, 0, ... % Ending Phase/Vel/Accel
    stepPeriod);  % Time to touchdown
stepWayPoints = [-a_back*stepDirVector.';  0 0 b; a_forward*stepDirVector.'];

transformsMat = reshape(transforms, [5 nPhases]);


% phasesTable = zeros(nPhases*3/2, nStanceLegs);
% phasesTable(1:3:end,:) = phasesToTest(1:2:end,:);
% phasesTable(2:3:end,:) = phasesToTest(1:2:end,:);
% phasesTable(3:3:end,:) = phasesToTest(2:2:end,:);
% transformsTable = zeros(nPhases*3/2, 5);
% transformsMatT = transformsMat.';
% transformsTable(1:3:end,:) = transformsMatT(1:2:end,:);
% transformsTable(2:3:end,:) = transformsMatT(2:2:end,:);
% transformsTable(3:3:end,:) = transformsMatT(2:2:end,:);
% timesList = (0:nPhases*3/2-1).';
% stuff = [timesList nan(nPhases*3/2,1) phasesTable nan(nPhases*3/2,1) transformsTable];

phasesTable = phasesToTest;
transformsTable = transformsMat.';
transformsTable = circshift(transformsTable, 1,1);

timesList = (0:nPhases-1).';
stuff = [timesList nan(nPhases,1) phasesTable nan(nPhases,1) transformsTable];

for n = 1:nCycles
%     for t = (0:.1:nPhases*3/2)-1
    for t = 0:.1:(nPhases-1)
        
        % get time and transformation via interpolation
        phaseNow = interp1(timesList,phasesTable, t);
        transformNow = interp1(timesList,transformsTable, t).';
        
        % current xyz: move feet to position in the phase. In plane.
        for i = 1:nStanceLegs
            leg = stepOrder(i);
            xyz(:,leg) = minJerkStepGait3(stepWayPoints, jerkCoeffs, xyz0(:,leg), eye(3), fractionStep, phaseNow(i));
            
            tmod = (mod(phaseNow(i), 2*pi));
            if (tmod<2*pi*fractionStep)&&(tmod>0)
                swingLegs(leg) = 1;
            else
                swingLegs(leg) = 0;
            end
            
        end
        
        
        % find the positions of the feet in the body frame.
        % transformation of Body viewed from Plane
        rB_P = transformNow(3:end); % vector of B wrt P,
        % (body frame wrt plane origin)
        thetaX = transformNow(1);
        thetaY = transformNow(2);
        RB_P = R_y(thetaY)*R_x(thetaX); % rotation of the frame B wrt P frame
        % (body frame coordinate system as viewed in plane frame)
        TB_P = [RB_P rB_P; 0 0 0 1]; % transformation to the body frame from the plane frame
        TP_B = inv(TB_P);
        xyzBh = TP_B*[xyz; ones(1,nLegs)];
        xyzB =xyzBh(1:3,:);
        xyzB(:,extraLegs) = xyzExtra;
        
        currentStanceLegs = find(~swingLegs);
        thIK = kin.getIK(xyzB);
        bodyCoM = kin.getSnakeMonsterCoM(thIK);
        % project it onto the plane:
        dp = TP_B(1:3,4) - bodyCoM;% difference between origins of plane and point
        planeNormal = TP_B(1:3,3); % z direction on plane frame viewed from body frame
        temp = sum(planeNormal.*dp)/sum(planeNormal.^2);
        projBodyCoM = bodyCoM + temp*planeNormal;% add relative difference to project point back to plane
        % projBodyCoM is in body frame
        
        % find the polygon and the distances in this rotated plane
        xyzContactRot = xyz(:,currentStanceLegs);
        K = convhull(xyzContactRot(1:2,:).');
        xOrderedRot = xyzContactRot(:,K.');
        xOrdered = TP_B*[xOrderedRot; ones(1, size(xOrderedRot,2))];
        %     xCentroid = mean(xOrdered(:,1:end-1),2);
        projBodyCoMRot = TB_P*[projBodyCoM;1];
        distToLine = p_poly_dist(projBodyCoMRot(1),projBodyCoMRot(2), xOrderedRot(1,:) ,xOrderedRot(2,:),  1);
        
        
        plt.plot(thIK); hold on;
        set(supportLines, 'xdata', xOrdered(1,:), 'ydata',xOrdered(2,:), ...
            'zdata', xOrdered(3,:));
        inSupport = inpolygon(projBodyCoMRot(1),projBodyCoMRot(2),xOrderedRot(1,:),xOrderedRot(2,:));
        set(projectedCOM, 'xdata', projBodyCoM(1), 'ydata',projBodyCoM(2), 'zdata', projBodyCoM(3),...
            'markerFaceColor', [1 0 0]*~inSupport);
        %   set(scatterCentroid, 'xdata', xCentroid(1), 'ydata',xCentroid(2), 'zdata',xCentroid(3));
        set(planeArrow,...
            'udata', TP_B(1,3)*.1,'vdata', TP_B(2,3)*.1,'wdata', TP_B(3,3)*.1,...
            'xdata', TP_B(1,4),'ydata', TP_B(2,4),'zdata', TP_B(3,4));
        
        % scatter3(xyzFK(1,:), xyzFK(2,:), xyzFK(3,:), 'r');
        %                 scatter3(xyzB(1,:), xyzB(2,:), xyzB(3,:), [], swingLegs, 'filled');
        
        if ~inSupport
            disp('out')
        end
        %
        %   if makeVideo
        %      writeVideo(v,getframe);
        %
        %   end
        
        
        if sendCommands
            cmd.position = [reshape(thIK,[1,18])];
            snakeMonster.set(cmd);
        end
        
    end
end
%{
clawAngle = [];
    cmd.position = nan(1,18+length(clawAngle));
    cmd.velocity = nan(1,18+length(clawAngle));
    cmd.torque = nan(1,18+length(clawAngle));
    snakeMonster.set(cmd);
%}