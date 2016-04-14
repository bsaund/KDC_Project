% stepOpt
% pick which leg to move after an initial stance optimization

% mainStanceOpt;

% set a step direction and magnitude
stepDirection = 0; % deviation away from straight, radians
stepMag = .1; % how far to move

xyzNew = xyz;

for nCycle = 1:3


xyzContact = xyzNew(:,stanceLegs);
z0 = mean(xyzContact(3,:));
projectedCoM = [bodyCoM(1:2); z0];
startBodyCoM = projectedCoM; % Com before step

distToLine = cell(1,nStanceLegs);
hasStepped = []; % dummy intiialization

minDists = zeros(1,nStanceLegs);

 for stepCount = 1:nStanceLegs

     % each step the CoM moves forwards a bit
projectedCoM = projectedCoM + R_z(stepDirection)*[0; stepMag; 0]/nStanceLegs;

for i = 1:nStanceLegs

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
    end    
    
    minDists(i) = min(distToLine{i});
end

[~, inds] = sort(minDists);
k = nStanceLegs;
while any(stanceLegs(inds(k))== hasStepped)
   k = k-1; 
end
stepLeg = stanceLegs(inds(k));
hasStepped = [hasStepped stepLeg];

% move the step leg in the step direction
xyzNew(:,stepLeg) = xyzNew(:,stepLeg) + R_z(stepDirection)*[0; stepMag; 0];
% move the other legs in the opposite direction by a bit
notStepLegs = stanceLegs(stanceLegs~=stepLeg);
xyzNew(:,notStepLegs) = xyzNew(:,notStepLegs) - repmat(R_z(stepDirection)*[0; stepMag; 0]/nStanceLegs, [1 nStanceLegs-1]);

% recalculate contacts and COM etc
 xyzContact = xyzNew(:,stanceLegs);
 z0 = mean(xyzContact(3,:));
  K = convhull(xyzContact(1:2,:).');
 xOrdered = xyzContact(1:3,K.');
  CoMs = kin.getCenterOfMasses(thFull);
bodyCoM = ...
 [ reshape(CoMs, [3,30]) zeros(3,1)]*...
 [reshape(masses, [30,1]); params.robotMass] / sum([reshape(masses, [30,1]); params.robotMass]);
 inSupport = inpolygon(bodyCoM(1,end),bodyCoM(2,end),xOrdered(1,:),xOrdered(2,:));
 % find the centroid of the support polygon
projectedCoM = [bodyCoM(1:2); z0];
 xCentroid = mean(xOrdered(:,1:end-1),2);
 
thFull = kin.getIK(xyzNew);
plt.plot(thFull);
  set(supportLines, 'xdata', xOrdered(1,:), 'ydata',xOrdered(2,:), ...
     'zdata', xOrdered(3,:));
  set(projectedCOM, 'xdata', bodyCoM(1), 'ydata',bodyCoM(2), 'zdata', z0,...
     'markerFaceColor', [1 0 0]*~inSupport);
  set(scatterCentroid, 'xdata', xCentroid(1), 'ydata',xCentroid(2), 'zdata',z0);

 end

end