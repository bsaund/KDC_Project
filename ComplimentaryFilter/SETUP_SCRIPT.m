% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SETUP_SCRIPT               %
% % matlabSnakeControl Project %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Run this script before using any of the code in the matlabSnakeControl
% project.
%
% DETAILS:
%
% This script must be run from the directory in which it is located.  This
% allows the appropriate paths to be added relative to each person's.  The
% script makes sure that all the proper folders are added to the
% Matlab path.
%
% Dave Rollinson
% Aug 2013


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SET FIGURE RENDERER STUFF   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Currently setting to 'zbuffer', since 'opengl' has lots of issues on 
% Linux.
%rendererType = 'ZBUFFER';
%rendererType = 'PAINTERS';
rendererType = 'OPENGL';

fprintf(['Setting Renderer to ' rendererType ' ...']);

set(0,'DefaultFigureRenderer',rendererType);

% % Don't use hardware opengl.
% if strcmp(rendererType,'OPENGL')
%     opengl('software');
% end

fprintf('Done!\n'); 


%%%%%%%%%%%%%%%%%%%%%%%%%
% ADD STUFF TO THE PATH %
%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('Adding Paths...');

% This folder
addpath( pwd );

% Save this folder
matlabSC_folder = pwd;

% Common folders 
% (gaits, helper functions, log tools, etc.)
addpath( [pwd '/common'], ...
         [pwd '/common/java'], ...
         [pwd '/common/matlab/'], ...
            [pwd '/common/matlab/logTools/sc2logs'], ...
            [pwd '/common/matlab/animate'], ... 
            [pwd '/common/matlab/animate/examples'], ... 
            [pwd '/common/matlab/gaits'], ... 
            [pwd '/common/matlab/interpolate'], ... 
            [pwd '/common/matlab/interpolate/naninterp'], ... 
            [pwd '/common/matlab/kinematics'], ... 
            [pwd '/common/matlab/rotate'], ... 
            [pwd '/common/matlab/rotate/SpinCalc'], ...
            [pwd '/common/matlab/virtual_chassis'], ...
            [pwd '/common/matlab/state_estimation'], ...
         [pwd '/common/Eigen'], ...
         [pwd '/common/protocol']);
         
% Add the other folders
addpath( [pwd '/ekfPlotting'], ...
         [pwd '/snakeBehaviors'], ...
         [pwd '/snakeGUIs'], ...
         [pwd '/SEA_Snake_Testing'], ...
            [pwd '/SEA_Snake_Testing/SEA_Gains'], ...
            [pwd '/SEA_Snake_Testing/SEA_Logs'], ...
            [pwd '/SEA_Snake_Testing/SEA_SpringEstimation'], ...
            [pwd '/SEA_Snake_Testing/ThermalModelling'], ...
         [pwd '/SnakeMonster'], ...   
         [pwd '/snakeCharming'], ...
         [pwd '/snakeIK'] );
            
     
fprintf('Done!\n');   


%%%%%%%%%%%%%%%%%%%%
% SETUP JAVA STUFF %
%%%%%%%%%%%%%%%%%%%%

% Run the java setup function.  This compiles:
%   LCM messages
%   LCM tools (lcm-spy, lcm-logplayer)
%   Java ethernet process
%   Java tools (kinematics, etc)

% % Takes in a parameter, overwrite.  Which forces it to compile if true,
% % otherwise it does compile if javaSnakeTools.jar already exists.
% overwriteJava = true;    
% SETUP_JAVA(overwriteJava);

% Change directory in case SETUP_JAVA fails
cd( matlabSC_folder );

% % Shutdown any java stuff that might be running.  This was added because of
% % some weird stuff that shows up when not running on the standard
% % LCM_DEFAULT_URL address.
% fprintf('\nShutting down left-over Java processes!\n');   
% shutdown;
% fprintf('Done!\n\n');   


