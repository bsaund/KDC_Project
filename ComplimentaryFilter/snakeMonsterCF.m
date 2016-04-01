function [chassisDCM, chassisAngVel] = ...
                        snakeMonsterCF( fbk, dt, legBases, lastDCM )
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
    numLegs = size(legBases,3);
    
    % Weight on accelerometer correction term
    accelWeight = .5;

    accelVecModule = nan(3,numLegs);
    accelVecBody = nan(3,numLegs);
    
    gyroVecModule = nan(3,numLegs);
    gyroVecBody = nan(3,numLegs); 
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % AVERAGE THE ACCELEROMETERS + GYROS %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Take the readings from the first module on each leg.
    for i=1:numLegs
        
        % Accelerometers (g)
        accelVecModule(1,i) = fbk.accelX(i*3-2);
        accelVecModule(2,i) = -fbk.accelY(i*3-2);
        accelVecModule(3,i) = fbk.accelZ(i*3-2);
        
        accelVecBody(:,i) = legBases(1:3,1:3,i) * ...
                                        accelVecModule(:,i);
                                    
        % Gyros (rad/s)
        gyroVecModule(1,i) = fbk.gyroX(i*3-2);
        gyroVecModule(2,i) = fbk.gyroY(i*3-2);
        gyroVecModule(3,i) = fbk.gyroZ(i*3-2);
        
        gyroVecBody(:,i) = legBases(1:3,1:3,i) * ...
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
    if isempty(lastDCM)
        
          accelAxis = cross( upVec, gravityVec );
          accelAxis = accelAxis / norm(accelAxis);
          
          accelAngle = rad2deg( acos( dot(upVec,gravityVec) ) );
          
          q = SpinCalc( 'EVtoQ', ...
                     [accelAxis', accelAngle], ...
                     1E6, 0 )';
    else
        q = SpinCalc( 'DCMtoQ', lastDCM, 1E6, 0 )';
    end
    
    % ESTIMATE NEW ORIENTATION BY FORWARD INTEGRATING GYROS
    w_x = gyroVecAvg(1);
    w_y = gyroVecAvg(2);
    w_z = gyroVecAvg(3);
    
    q_update = quat_rotate_better( w_x, w_y, w_z, q, dt );    
    orientDCM = SpinCalc('QtoDCM', q_update', 1E6, 0 )';
    
    %gravityVec
    accelGravity = orientDCM' * gravityVec;
    
    accelAxis = cross( upVec, accelGravity );
    accelAxis = accelAxis / norm(accelAxis);
    accelAngle = rad2deg( acos( dot(upVec,accelGravity) ) );
    
    
    % MESS W/ THE ACCEL WEIGHT
    
    % Scale down if gyro readings are large
    gyroMag = norm(gyroVecAvg);
    gyroScale = 1;
    
    accelWeight = accelWeight / (1 + gyroScale * gyroMag);

    % Scale down if accelerometers deviate from 1g.
    accelMag = norm(accelVecAvg);
    accelThresh = .1;
    
    accelDev = abs(accelMag - 1.01) > accelThresh;
    
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
    chassisDCM = updatedDCM;
    chassisAngVel = [ w_x; 
                      w_y;
                      w_z ];
    
end