classdef robot
    %ROBOT represents robot that will construct the shape.
    %   The robots:
    %   (1) are identical and labelled;
    %   (2��can locate themselves in the global map;
    %   (3) can sense the neighbouring environment (location of 
    %   objects/modules/robots);
    %   (4) run planning algorithms and move to goal positions.
    
    properties (Access = public)
        % Can be directly modified by the user.
        ID            (1, 1) int32 {mustBeNonnegative} % ID of the robot
        Status        (1, 1) uint8 % (1) initial; (2) fixed; (3) unfixed; (4) free
        isCarrying    (1, 1) logical % Carrying (1) / Not carrying (0) module
        Loc           (1, 3) double  % Current robot location [x, y, z]
        localMap
        Goal          (1, 3) double  % Current robot goal [x, y, z]
        Path
        
    end
    
    methods
        function obj = robot(inputArg1,inputArg2)
            %ROBOT ��������ʵ��
            %   �˴���ʾ��ϸ˵��
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 �˴���ʾ�йش˷�����ժҪ
            %   �˴���ʾ��ϸ˵��
            outputArg = obj.Property1 + inputArg;
        end
    end
end

