classdef controller < handle
    properties(Constant)

    end
    
    properties(Access = public)
    traj;
    p_gain;
    d_gain;
    i_gain;
    controlMat;
    tau;
    mode;
    prevError;
    integralError;
    t_prev;
    end
            
    methods(Access = public)
        
        function obj = controller(traj, kx, ky, kth, mode)
            obj.traj = traj;
            obj.mode = mode;
            
            obj.controlMat = [kx, 0, 0; 0, ky, kth];
            obj.prevError = zeros(3,1);
            obj.integralError = zeros(3,1);
            obj.t_prev = 0.0;
            
        end
        function [V, w, error] = pidCorrect(obj, t, startPose, actualPose)
            %refPoseVec = obj.traj.getPoseAtTime(t);
            currRef = obj.traj.getPoseAtTime(t);
            currentRefPose = pose(currRef(1), currRef(2), currRef(3));
            refposWorldmat   = startPose.bToA()*currentRefPose.bToA();
            refPoseVec = pose.matToPoseVec(refposWorldmat);
            actualPoseVec  = actualPose.getPoseVec();
            actTh = actualPoseVec(3);
            refth = refPoseVec(3);
            refPoseVec(3) = 1;
            error = actualPose.aToB()*refPoseVec;
            errorPos = error(1:2);
            errorTh  = refth - actTh; 
            errorTh  = atan2(sin(errorTh), cos(errorTh));
            
            error = [errorPos', errorTh]';
            dt = t-obj.t_prev;
            obj.t_prev = t;
            errorDervative = (error-obj.prevError)/dt;
            obj.prevError  = error;
            
            obj.integralError = obj.integralError + error;
            
            feedBackControl = obj.controlMat*error;
            
            Vf  = obj.traj.getVAtTime(t);
            wf  = obj.traj.getwAtTime(t);
            
            finalControl = [Vf; wf] + obj.mode*feedBackControl;
            
            V = finalControl(1);
            w = finalControl(2);
            
        end

    end
end