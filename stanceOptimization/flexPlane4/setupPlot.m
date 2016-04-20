     plt = SnakeMonsterPlotter(); 
     set(gcf, 'position', [10 100 700 700]);  
plt.plot(zeros(1,18)); hold on;
projectedCOM = scatter3(0,0,0,'k', 'filled'); 
supportLines = plot3(0,0,0,'k');
% planeArrow = quiver3(0,0,0,0,0,0,'b');