% plotOptResultsStatic
global plt
close all;
     plt = SnakeMonsterPlotter(); 
     kin = SnakeMonsterKinematics();


 %% plot the results
xyStep = reshape(stateOpt(1:nLegs*2), [2 nLegs]);
planexyc = stateOpt(end-2:end);

xyz = zeros(3,6);
xyz(1:2,:) = xyStep; 
% set the feet z positions to be the projection on to the plane
% planex*x + planey*y + z +planec = 0 -->% z = -planex*x - planey*y -planec
xyz(3,stanceLegs) = ...
    - xyz(1,stanceLegs)*planexyc(1) - xyz(2,stanceLegs)*planexyc(2) - planexyc(3);
% leave z of extralegs = 0

  thIK = kin.getIK(xyz);
  xyzFK = kin.getLegPositions(thIK);
   
% get the COM:
 bodyCoM = kin.getSnakeMonsterCoM(thIK);
 % project it onto the plane:
% difference between origins of plane and point
% dp = bsxfun(@minus, origins, point);
dp = [0; 0; -planexyc(3)] - bodyCoM;
% relative position of point on normal's line
% t = bsxfun(@rdivide, sum(bsxfun(@times,normals,dp),2), sum(normals.^2,2));
normals = [planexyc(1); planexyc(2); 1];
temp = sum(normals.*dp)/sum(normals.^2);
% add relative difference to project point back to plane
% projBodyCoM = bsxfun(@plus, point, bsxfun(@times, t, normals));
projBodyCoM = bodyCoM + temp*normals;

% %  the support polygon
 xyzContact = xyz(:,stanceLegs);
 K = convhull(xyzContact(1:2,:).');
 xOrdered = xyzContact(:,K.');
 xCentroid = mean(xOrdered(:,1:end-1),2);
  
%   plt = SnakeMonsterPlotter(); 
  plt.plot(thIK); hold on;
    set(gcf, 'position', [10 100 500 500]);
projectedCOM = scatter3(0,0,0,'k', 'filled');
supportLines = plot3(0,0,0,'k');
% scatterCentroid = scatter3(0,0,0,'b');
  set(supportLines, 'xdata', xOrdered(1,:), 'ydata',xOrdered(2,:), ...
     'zdata', xOrdered(3,:));
  inSupport = inpolygon(projBodyCoM(1,end),projBodyCoM(2,end),xOrdered(1,:),xOrdered(2,:));
  set(projectedCOM, 'xdata', projBodyCoM(1), 'ydata',projBodyCoM(2), 'zdata', projBodyCoM(3),...
     'markerFaceColor', [1 0 0]*~inSupport);
%   set(scatterCentroid, 'xdata', xCentroid(1), 'ydata',xCentroid(2), 'zdata',xCentroid(3));
