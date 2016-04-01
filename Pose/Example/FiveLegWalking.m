cd ../..
addpath(genpath(fullfile(pwd,'Pose')))
cd(fullfile(pwd,'Pose'))
cd(fullfile(pwd,'Example'))
log = HebiUtils.convertGroupLog('2016-03-28_15-39-41.125646.hebilog');

clear F

n = size(log.accelX,1);

videoName = 'unknown.avi';
generateVideo = false;

if ishandle(1)
    close(figure(1))
end

plt = SnakeMonsterPlotter();
pose = SnakeMonsterPose();
count = 0;
firstRun = true;

figure(1)


F(n) = struct('cdata',[],'colormap',[]);
for i=1:n
    fbk.time = log.time(i,:);

    fbk.accelX = log.accelX(i,:);
    fbk.accelY = log.accelY(i,:);
    fbk.accelZ = log.accelZ(i,:);

    fbk.gyroX = log.gyroX(i,:);
    fbk.gyroY = log.gyroY(i,:);
    fbk.gyroZ = log.gyroZ(i,:);
    
    fbk.torque = log.torque(i,:);
 
    fbk.position = log.position(i,:);

    plt.plot(fbk.position)
    pose.plotEnvironment(fbk);
    axis equal
    
    if firstRun
        pause
        firstRun = false;
    end
    if generateVideo
        az = 180;
        el = -90;
        
        view(az,el)
        axis equal
        xlim([-0.5 0.5])
        ylim([-0.4 0.6])
        F(i) = getframe;
    end
end
close(1)

if generateVideo
    text(-0.5,0,0,'WRITING VIDEO...','Color','g','FontSize',25)
    xlim([-1 1])
    ylim([-1 1])
    v = VideoWriter(videoName);
    open(v)
    for i=1:size(F,2)
        writeVideo(v,F(1,i));
    end
    close(v)
    close(1)
end

text(-0.675,0,0,'DONE','Color','g','FontSize',100)
xlim([-1 1])
ylim([-1 1])

