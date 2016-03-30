% stepOpt
% pick which leg to move after an initial stance optimization

addpath(genpath('C:\Users\medgroup01\Documents\Julian\snakeMonster\KDC_Project'));
load('stanceOptResults3_30_12pm.mat');
% gives us stanceLegs and thFull

% set a step direction and magnitude
stepDirection = -pi/2; % deviation away from straight, radians
stepMag = .07; % how far to move

xyzNew = xyz;
xyzContact = xyzNew(:,stanceLegs);
z0 = mean(xyzContact(3,:));
projectedCoM = [bodyCoM(1:2); z0];
startBodyCoM = projectedCoM; % Com before step

distToLine = cell(1,nStanceLegs);
hasStepped = []; % dummy intiialization
figure;

minDists = zeros(1,nStanceLegs);

 for stepCount = 1:nStanceLegs

     % each step the CoM moves forwards a bit
projectedCoM = projectedCoM + R_z(stepDirection)*[0; stepMag; 0]/nStanceLegs;
scatter(projectedCoM(1), projectedCoM(2)); hold on;
scatter(xyzNew(1,stanceLegs), xyzNew(2,stanceLegs)) ;hold off;
xlim([-.5 .5]); ylim([-.5 .5]);

for i = 1:nStanceLegs

    %     
    legLifted = stanceLegs(i);
    newStanceLegs = stanceLegs([1:i-1, i+1:end]);
    xyzContactNew = xyz(:,newStanceLegs);
    KNew = convhull(xyzContactNew(1:2,:).');
    xOrdered = xyzContactNew(:,KNew.');
    distToLine{i} = zeros(1,length(KNew)-1);
    
    % find the min distance to a line
    for j = 1:length(KNew)-1
        %      d = abs(cross(Q2-Q1,P-Q1))/abs(Q2-Q1)
        distToLine{i}(j) = norm(cross(xOrdered(:,j+1)-xOrdered(:,j), projectedCoM-xOrdered(:,j)))...
            /norm(xOrdered(:,j+1)-xOrdered(:,j));
        %      line( [xOrdered(1,j+1) xOrdered(1,j)], [xOrdered(2,j+1) xOrdered(2,j)] );
    end    
    
    minDists(i) = min(distToLine{i});
end


% stepLegLast = stepLeg;
% [~, inds] = sort(minDists); % this leg in stanceLegs has the least bad effects of lifting it.
% stepLeg = stanceLegs(inds(end));
% % check if it's the same leg again, if so, do a differnet one.
% if stepLeg==stepLegLast
%    stepLeg = stanceLegs(inds(end-1));
% end

[~, inds] = sort(minDists);
k = nStanceLegs;
while any(stanceLegs(inds(k))== hasStepped)
   k = k-1; 
end
stepLeg = stanceLegs(inds(k));
hasStepped = [hasStepped stepLeg];

% move the step leg in the step direction
xyzNew(:,stepLeg) = xyzNew(:,stepLeg) + R_z(stepDirection)*[0; stepMag; 0];

 end
scatter(projectedCoM(1), projectedCoM(2)); hold on;
scatter(xyzNew(1,stanceLegs), xyzNew(2,stanceLegs)) ;hold off;
xlim([-.5 .5]); ylim([-.5 .5]);

% some results:
% dir= 0--> hasStepped =  4     6     2     5     3
% dir= pi--> hasStepped = 4     2     3     5     6
