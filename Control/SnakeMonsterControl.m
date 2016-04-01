classdef SnakeMonsterControl < handle
    % SNAKEMONSTERCONTROL Control the Snake Monster
    %
    %   SnakeMonsterControl Methods (constructor):
    %      SnakeMonsterControl  - constructor
    %
    %   SnakeMonsterControl Methods:
    %      getGravityCF - gets a vector in the direction of gravity
    %
    %   Examples:
    %      control = SnakeMonsterControl();
%     %      %plt.plot([.1,.1]);
%     %
%     %      %plt = HebiPlotter(16, 'resolution', 'high');
%     %      %plt.plot(zeros(16,1));
    
    methods (Access = public)
        function this = SnakeMonsterControl(varargin)
        %SNAKEMONSTERCONTROL
        %Arguments:
        %
        %Optional Parameters:
        %  'controlType'       - 'low' (default), 'high' 
        %
        %Examples:
%         %  plt = HebiPlotter(16)
%         %  plt = HebiPlotter(4, 'resolution', 'high')
%         %
%         %  links = {{'FieldableElbowJoint'},
%         %           {'FieldableStraightLink', 'ext1', .1, 'twist', 0},
%         %           {'Foot', 'ext1', .1, 'twist', 0}};
%         %  plt = HebiPlotter('JointTypes', links)

            p = inputParser;
            
            expectedGravity = {'on','off'};
            expectedKinematics = {'on','off'};
            
            addParameter(p,'gravity','off', ...
                         @(x) any(validatestring(x,expectedGravity)));
            addParameter(p,'kinematics','off', ...
                         @(x) any(validatestring(x,expectedKinematics)));
                     
            parse(p,varargin{:});
            
            this.gravity = strcmpi(p.Results.gravity,'on');
            this.kinematics = strcmpi(p.Results.kinematics,'on');

            this.firstRun = true;
            this.oldTime = 0;
            this.lastDCM = [];
            
            
        end
    end
 
    methods(Access = private, Hidden = true)
        function initializeSnakeMonster(this)
            if exist(this.SM)
                return;
            else
                this.SM = HebiLookup
            
            end
        end
        
        function Gravity = getGravityCF(this)
            %Assumes the body center stays at the orign
            UpVector = [0;0;1];
            legBases = getSMLegBases;
            
            fbk = this.getNextFeedback;
            [DCM,~] = snakeMonsterCF(fbk,dt,legBases,lastDCM);
            
            %Gravity vector points in the direction of gravity not in the
            %direction of the opposing acceleration as usually transmitted
            %from the accelerometers
            Gravity = DCM*UpVector; 
        end
    end
    
    
    properties (Access = private, Hidden = true)
        SM
        firstRun
        oldTime
        lastDCM
        gravity
        kinematics
    end
end

