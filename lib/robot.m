classdef robot
    %ROBOT represents robot that will construct the shape.
    %   The robots:
    %   (1) are identical and labelled;
    %   (2）can locate themselves in the global map;
    %   (3) can sense the neighbouring environment (location of 
    %   objects/modules/robots);
    %   (4) run planning algorithms and move to goal positions.
    
    properties (Access = public)
        % Can be directly modified by the user.
        ID            (1, 1) int32 {mustBeNonnegative} % ID of the robot
        Status        (1, 1) uint8 % (1) initial; (2) fixed; (3) unfixed; (4) free
        isCarrying    (1, 1) logical % Carrying (1) / Not carrying (0) module
        Location           (1, 3) double  % Current robot location [x, y, z]
        localMap                    % Sensing Map
        CognMap                     % Cognitive map of individual robot
        Goal          (1, 3) double  % Current robot goal [x, y, z]
        Path % Current planned path 
        
    end
    
    methods
        function rob = robot(id,initLocation)
            %ROBOT 构造此类的实例
            %   此处显示详细说明
            rob.ID = id;
            rob.Location = initLocation;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
        
        function path = move()
            % 
        end
        
        function path = Astar(startPose, goalPose, distance)
            % default value
            if (nargin<3)
                distance = 0.01;
            end
            % Create a state space object
            stateSpace = stateSpaceSE2;
            % Create a state validator object
            validator = validatorOccupancyMap(stateSpace);
            % Create a binary occupancy map and assign the map to the state validator object.
            validator.Map = binaryOccupancyMap(map);
            % Set the validation distance for the validator
            validator.ValidationDistance = distance;
            % Assign the state validator object to the plannerHybridAStar object
            planner = plannerHybridAStar(validator);
            % Compute a path for the given start and goal poses
            pathObj = plan(planner,startPose,goalPose);
            % Extract the path poses from the path object
            path = pathObj.States;
        end
    end
end

