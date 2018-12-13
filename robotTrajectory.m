classdef robotTrajectory < handle
    %pose A layer over a column vector that provides access methods and
    % associated homogeneous transforms. For the purpose of naming the
    % homogeneous transforms, the pose is considered to be that of frame b
    % relative to frame a.
    
    properties(Constant)

    end
    
    
    properties(Access = public)
    numSamples;
    init_dist;
    init_pose;
    dt;
    refControl;
    velArray  = [];
    wArray    = [];
    poseArray = [];
    timeArray = [];
    distArray = [];
    xArray = [];
    yArray = [];
    thArray = [];
    
    end
            
    methods(Access = public)
        
        function obj = robotTrajectory(numSamples, init_dist, init_pose, dt, refControl)
        obj.numSamples = numSamples;
        obj.distArray(1)= init_dist;
        
        obj.xArray(1) = init_pose.x();
        obj.yArray(1) = init_pose.y();
        obj.thArray(1) = init_pose.th();
        
        %obj.poseArray(1)= init_pose;
        obj.dt = dt;
        obj.refControl = refControl;
        
        for i=1:numSamples-1
            t = double(i-1)*obj.dt;
            [V, w]  = refControl.computeControl(t);
            obj.velArray(i)  = V;
            obj.wArray(i)    = w;
            obj.timeArray(i) = t;
            obj.distArray(i+1) = obj.distArray(i) + obj.velArray(i)*dt;
            obj.thArray(i+1) = obj.thArray(i) + obj.wArray(i)*dt;
            obj.thArray(i+1) = atan2(sin(obj.thArray(i+1)), cos(obj.thArray(i+1)));
            obj.xArray(i+1)  = obj.xArray(i) + obj.velArray(i)*cos(obj.thArray(i+1))*dt;
            obj.yArray(i+1)  = obj.yArray(i) + obj.velArray(i)*sin(obj.thArray(i+1))*dt;       
        end  

        t = (numSamples-1)*dt;
        [V, w]  = refControl.computeControl(t);
        obj.velArray(numSamples)  = V;
        obj.wArray(numSamples)    = w;
        obj.timeArray(numSamples) = t;
                
        end
        function pos = getPoseAtTime(obj,t)

            t_dt = t/obj.dt;
            if t_dt < 1.
                pos = [0; 0; 0]; 
            else
            x  = interp1(obj.xArray,t_dt);
            y  = interp1(obj.yArray,t_dt);
            th = interp1(obj.thArray,t_dt);            
            pos = [x; y; th]; 
            end
        end
        function V = getVAtTime(obj,t)
            t_dt = t/obj.dt;
            if t_dt < 1.
                V = 0;
            else
            V  = interp1(obj.velArray,t_dt);
            end
        end
        function w = getwAtTime(obj,t)
            t_dt = t/obj.dt;
            if t_dt < 1.
                w = 0;
            else
            w  = interp1(obj.wArray,t_dt); 
            end
        end
        
        function duration = getTrajectoryDuration(obj)
        % Return the total time required for motion and for the
        % initial and terminal pauses.
        duration = obj.refControl.getTrajectoryDuration();
        end
        function V = getCmdV(obj)
            V = 0.25;
        end
    end
end