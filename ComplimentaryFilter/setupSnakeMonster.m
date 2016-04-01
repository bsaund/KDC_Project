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

HebiLookup.clearGroups();

%Ky: Changed the modules to be reflective of the current configuration
names = { 'SA041', 'SA033', 'SA050', ...  %Leg 1   
          'SA028', 'SA026', 'SA022', ...  %Leg 2
          'SA046', 'SA035', 'SA045', ...  %Leg 3
          'SA039', 'SA037', 'SA027', ...  %Leg 4  
          'SA047', 'SA043', 'SA029', ...  %Leg 5
          'SA048', 'SA049', 'SA036' };    %Leg 6


% names = { 'SA003', 'SA026', 'SA019', ...          
%           'SA001', 'SA041', 'SA022', ...           
%           'SA035', 'SA050', 'SA005', ...
%           'SA044', 'SA049', 'SA006', ...
%           'SA032', 'SA037', 'SA008', ...          
%           'SA039', 'SA047', 'SA045' };
% names = { 'SA001', 'SA041', ... 
%           'SA005', 'SA037', 'SA006', ... 
%           'SA017', 'SA050', 'SA022', ...
%           'SA013', 'SA031', 'SA004', ...
%           'SA007', 'SA043', 'SA003', ...
%           'SA020', 'SA049', 'SA002' };
      
feet = {'FOOT_1', 'FOOT_2', 'FOOT_3', 'FOOT_4','FOOT_5','FOOT_6'};

groupName = '*';

legGroup = HebiLookup.newGroupFromNames(groupName, names);
   
%footGroup = HebiApi.newGroupFromNames(groupName, feet);

% Setup matlabSnakeControl Tools
SETUP_SCRIPT;


fprintf('Success!\n\n');
