classdef trajectoryFollower < handle
    %pose A layer over a column vector that provides access methods and
    % associated homogeneous transforms. For the purpose of naming the
    % homogeneous transforms, the pose is considered to be that of frame b
    % relative to frame a.
    
    properties(Constant)

    end
    
    
    properties(Access = public)
    trajGenerator;
    fbkController;
    robotMdl;
    robot;
    tf;
    end
            
    methods(Access = public)
        
        function obj = trajectoryFollower(robot, robotMdl, trajGenerator, fbkController)
            obj.trajGenerator = trajGenerator;
            obj.fbkController = fbkController;
            obj.robotMdl = robotMdl;
            obj.robot = robot;
            obj.tf = trajGenerator.refControl.getTrajectoryDuration(); 
        end
        function error = update(obj, timeNow, actualPose)
            [V, w, error] = obj.fbkController.pidCorrect(timeNow, actualPose); 
            if abs(w)>1.2
                fprintf('I found one: %.2f \n', w);
                w = w/abs(w);
            end
            [vl , vr] = obj.robotMdl.My_VwTovlvr(V, w);
            
            if timeNow > obj.tf && timeNow < obj.tf + 1
                obj.robot.stop();
            else
                obj.robot.sendVelocity(vl, vr);       
            end
        end    
    end
end