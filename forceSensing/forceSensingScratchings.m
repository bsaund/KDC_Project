% Force sensing scratchings
% given a static pose and joint torques, try to solve for some kind of
% external forces. This won't be fully possible but any estimate might
% help.


% addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));
% addpath(genpath('C:\Users\Julian\Box Sync\CMU sem 1+2\snakeMonster\KDC_project'));
%  load('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project\stanceOptimization\findXYZOptWalkTilt\fiveLegGreat4_6_10am.mat');
%  load('C:\Users\Julian\Box Sync\CMU sem 1+2\snakeMonster\KDC_project\stanceOptimization\findXYZOptWalkTilt\fiveLegGreat4_6_10am.mat');
 
% plt = SnakeMonsterPlotter;
kin = SnakeMonsterKinematics(); % need to remake kin object

masses = kin.getLegMasses();
xyz(:,extraLegs) = xyzExtra;
xyz(1:2,stanceLegs) = reshape(xyStepOpt, [2,nStanceLegs]);
xyz(3,stanceLegs) = ...
    - xyz(1,stanceLegs)*planexyc(1) - xyz(2,stanceLegs)*planexyc(2) - planexyc(3);
xyzBase = xyz;
thIK = kin.getIK(xyz); 
  xyzFK = kin.getLegPositions(thIK);
  gravity = -9.81*planeNormal/norm(planeNormal);
  bodyMass = (params.robotMass);
% plt.plot(thIK)

 % find the expected torques
    % jacobian in world frame for each leg:
    J = kin.getLegJacobians(thIK);
    % gravity compensation torques: (assumes fixed base)
    legTorques = kin.getLegGravCompTorques(thIK, gravity);
    % the force on each foot is the chassis weight plus the fixed module
    % segments, in the direction opposite gravity, divided with pinv
    xyzContact = xyzFK(:,~swingLegs);
    cmdFootForce = zeros(6,1);
    % the commanded foot force based on a best fit to the force distribution
    cmdFootForce(~swingLegs) = pinv(xyzContact) * -gravity*(bodyMass + 0);
    for i=1:6
        footWrench = [0;0;cmdFootForce(i); zeros(3,1)];
        if ~swingLegs(i) % legs in the air just have grav comp. Stance legs have some weight on them:
            legTorques(:,i) =  legTorques(:,i) + J(:,:,i).'*footWrench;
        end
    end

    % tau = J.' * f
    n = sum(~swingLegs);
    f_applied = [10; 10; 0; 0 ; 0 ;0]; % some wrench applied to the body
    % assume each leg carries it equally?
    currentLegs = stanceLegs(~swingLegs(stanceLegs));
    tau_applied = zeros(3,n);
    for k=1:n
        leg = currentLegs(k);
       tau_applied(:,k) = J(:,:,leg).' * -f_applied/n;
    end
    %in reality that is added onto the full torque, then subtracted.
%     legTorques(:,~swingLegs)....
%   [tau1; tau2; ... taun] = [J1.'; J2.'; ...Jn'] * -f/n
% f = -n* pinv([J1.'; J2.'; ...Jn'] )*[tau1; tau2; ... taun]

f_guess = -n* pinv(reshape(J(:,:,~swingLegs), [6,3*n]).')*reshape(tau_applied, [3*n,1]);
disp(f_guess.')



