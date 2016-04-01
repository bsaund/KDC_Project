function [ legBases ] = getSMLegBases(  )
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
    R_leftLegs = R_leftLegs* R_x(-degtorad(1.5));
    R_rightLegs = R_rightLegs * R_x(-degtorad(1.5));
      
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

