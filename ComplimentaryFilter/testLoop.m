%setupSnakeMonster

legBases = getSMLegBases;
lastDCM = [];

dt = 1;
oldtime = 0;

while 1
    
    fbk = legGroup.getNextFeedback;
    
    [chassisDCM,chassisAngVel] = snakeMonsterCF(fbk,dt,legBases,lastDCM);

    gz = [0;0;1];
    T = chassisDCM*gz;

    X = [0,T(1)];
    Y =[0,T(2)];
    Z = [0,T(3)];

    plot3(X,Y,Z)
    
    axis equal
    xlim([-1,1])
    ylim([-1,1])
    zlim([-1,1])
    drawnow
    
    dt = fbk.time-oldtime;
    oldtime = fbk.time;
end