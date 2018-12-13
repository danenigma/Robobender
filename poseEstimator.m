classdef poseEstimator < handle
    %pose A layer over a column vector that provides access methods and
    % associated homogeneous transforms. For the purpose of naming the
    % homogeneous transforms, the pose is considered to be that of frame b
    % relative to frame a.
    
    properties(Constant)

    end
    
    
    properties(Access = public)

        initPose;
        robot;
        robotMdl;
        sl;
        sr;
        sl_bias;
        sr_bias;
        x;y;th;
        xArray;
        yArray;
        thArray;
        plotEst;
        posePlot;
    end
            
    methods(Access = public)
        
        function obj = poseEstimator(robot, initPoseVec, plotEst)
            obj.robot = robot;
            obj.sl = 0;
            obj.sr = 0;
            obj.sl_bias = obj.robot.encoders.LatestMessage.Vector.X;
            obj.sr_bias = obj.robot.encoders.LatestMessage.Vector.Y;
            obj.x = initPoseVec(1);
            obj.y = initPoseVec(2);
            obj.th = initPoseVec(3);
            obj.xArray(1) = obj.x;
            obj.yArray(1) = obj.y;
            obj.thArray(1) = obj.th;
            obj.plotEst = plotEst;
            if obj.plotEst==true
                figure;
                obj.posePlot = plot(obj.xArray, obj.yArray, 'b-');
                title('Pose estimation')
                xlabel('x');
                ylabel('y');
                xlim([-.7 .7]);
                ylim([-.7 .7]);
                
                
            end
        end
        function currentPose = update(obj)
            dsl = obj.robot.encoders.LatestMessage.Vector.X - obj.sl - obj.sl_bias;
            dsr = obj.robot.encoders.LatestMessage.Vector.Y - obj.sr - obj.sr_bias;
            obj.sl  = obj.sl + dsl;
            obj.sr  = obj.sr + dsr;
            ds  = (dsl  + dsr)/2;
            dth = (dsr  - dsl)/robotModel.my_W;
            obj.th =  obj.th + dth;   
            obj.th = atan2(sin(obj.th), cos(obj.th));
            obj.x  =  obj.x  + cos(obj.th)*ds;
            obj.y  =  obj.y  + sin(obj.th)*ds;
            currentPose =  pose(obj.x, obj.y, obj.th);
            if obj.plotEst==true
                set(obj.posePlot, 'xdata', [get(obj.posePlot,'xdata') obj.x],...
                'ydata', [get(obj.posePlot,'ydata') obj.y]);
            end
        end    
    end
end