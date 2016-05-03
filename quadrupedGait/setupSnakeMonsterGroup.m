%%%%%%%%%%%%%%%%%%%%%%%
% ONE GREAT BIG GROUP %
%%%%%%%%%%%%%%%%%%%%%%%

%  Leg Numbering / Chassis Coordinate convention:
%  
%   2 ----- 1     +y
%       |          ^
%   4 ----- 3      |
%       |          o--> +x
%   6 ----- 5    +z
%
HebiLookup.clearModuleList();
names = { 'SA046', 'SA026', 'SA028', ...  %Leg 1
          'SA021', 'SA071', 'SA039', ...  %Leg 2
          'SA047', 'SA042', 'SA024', ...  %Leg 3
          'SA037', 'SA036', 'SA041', ...  %Leg 4
          'SA050', 'SA029', 'SA038', ...  %Leg 5
          'SA035', 'SA045', 'SA048' };    %Leg 6
      
% names = { 'SA041', 'SA033', 'SA050', ...  %Leg 1
%           'SA044', 'SA026', 'SA022', ...  %Leg 2
%           'SA046', 'SA035', 'SA045', ...  %Leg 3
%           'SA039', 'SA037', 'SA027', ...  %Leg 4
%           'SA047', 'SA043', 'SA029', ...  %Leg 5
%           'SA048', 'SA049', 'SA036' };    %Leg 6
      
      
groupName = '*';

snakeMonster = HebiLookup.newGroupFromNames(groupName, names);
   
setGainsSM; % set the gains