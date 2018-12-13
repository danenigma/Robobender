classdef trapezoidalStepReferenceControl < handle
    %robotModel A convenience class for storing robot physical 
    % and performing related kinematic transforms. You can reference the
    % defined constants via the class name with robotModel.W2 for
    % example because they are constant properties and therefore associated
    % with the class rather than any instance. Similiarly, the kinematics
    % routines are referenced from the class name as well.
    properties(Constant)
    end
    properties(Access = private)
    tramp;
    amax;
    vmax;
    sgn;
    tPause;
    tf;
    dist;

    end
    
    methods(Access = public)
        
        function obj = trapezoidalStepReferenceControl(dist, amax, vmax, sgn, tPause)
            obj.tramp = vmax/amax;
            obj.amax = amax;
            obj.vmax = vmax;
            obj.sgn = sgn;
            obj.tPause = tPause;
            obj.dist = abs(dist);
            obj.tf  = (abs(dist) + (vmax^2)/amax)/vmax;
        end
        function [V, w] = computeControl(obj,t)
            if t < obj.tPause
                uref = 0;
            elseif t < obj.tramp + obj.tPause
                uref = obj.amax * (t-obj.tPause);
            elseif (t> obj.tramp+obj.tPause && t < obj.tPause+(obj.tf-obj.tramp))
                uref = obj.vmax;
            elseif (obj.tf + obj.tPause - t) < obj.tramp
                uref = obj.amax*(obj.tf + obj.tPause-t);
            else
                uref = 0;
            end
            if uref<0
                uref = 0;
            end
            uref = obj.sgn*uref;
            V = uref;
            w = 0;
        end
        
        function duration = getTrajectoryDuration(obj)
        % Return the total time required for motion and for the
        % initial and terminal pauses.
        duration = (obj.dist + (obj.vmax^2)/obj.amax)/obj.vmax + obj.tPause + 1;
        end
    end
end