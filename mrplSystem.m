classdef mrplSystem < handle
     
    properties(Constant)
    NUM_SAMPLES = 4001; 
    end
    properties(Access = public)
    robot;
    estRobot;
    refRobot;
    xArray;
    yArray;
    thArray;
    plotEst;
    xRefArray;
    yRefArray;
    thRefArray;
    refRobotPlot
    actPosePlot;
    finalActX;
    finalActY;
    finalActTh;
    finalRefX;
    finalRefY;
    finalRefTh;
    tickCount;
    posEst;
    range_obj;
    end
    methods(Access = public)
        
        function obj = mrplSystem(robot, plotEst)
            obj.robot   = robot;
            obj.range_obj =  rangeImage();
            global mapPose; global MAP_LOCALIZATION
            MAP_LOCALIZATION = true;
            pause(2.);
            obj.refRobot = mapPose.getPoseVec();            
            obj.estRobot = poseEstimator(obj.robot, mapPose.getPoseVec(), false);
            obj.posEst   = pose(mapPose.getPoseVec());
            MAP_LOCALIZATION = false;
            fprintf('init map pose x: %2.2f, y: %2.2f, th: %2.2f \n',mapPose.x, mapPose.y, mapPose.th);
            obj.xArray(1) = obj.refRobot(1);
            obj.yArray(1) = obj.refRobot(2);
            obj.thArray(1) = obj.refRobot(3);
            obj.xRefArray(1) = obj.refRobot(1);
            obj.yRefArray(1) = obj.refRobot(2);
            obj.thRefArray(1) = obj.refRobot(3);
            obj.tickCount = 1;
            obj.plotEst = plotEst;
            %disp('Initializing the mrpl system')
            if obj.plotEst==true
                figure;
                obj.actPosePlot = plot(obj.xArray, obj.yArray, 'b-', 'DisplayName', 'actual Traj');
                hold on;
                obj.refRobotPlot = plot(obj.xRefArray, obj.yRefArray, 'r-', 'DisplayName', 'referance Traj');
                title('Pose estimation');
                xlabel('x');
                ylabel('y');
                %xlim([-.7 .7]);
                %ylim([-.7 .7]);
                legend('show')
            end
        end
        function executeTrajectoryToRelativePose(obj, x, y, th, vmax, sgn, useMap)
            global mapPose; global MAP_LOCALIZATION
            obj.posEst = obj.estRobot.update();
            mapPose = pose(obj.posEst.getPoseVec());
            MAP_LOCALIZATION = true;
            pause(2.)
            
            %while norm([mapPose.x-obj.posEst.x, mapPose.y-obj.posEst.y])>1.
                  %disp('map jump detected!!!')
            %end
            posEst_x = obj.posEst.x + 0.25*(mapPose.x-obj.posEst.x);
            posEst_y = obj.posEst.y + 0.25*(mapPose.y-obj.posEst.y);
            thDiff = mapPose.th-obj.posEst.th;
            thDiff = atan2(sin(thDiff), cos(thDiff));
            thAdd  = obj.posEst.th + 0.25*thDiff;
            thAdd  = atan2(sin(thAdd), cos(thAdd));
            posEst_th = thAdd;
            obj.posEst = pose(posEst_x, posEst_y, posEst_th);
            obj.estRobot = poseEstimator(obj.robot, obj.posEst.getPoseVec(), false);
            obj.refRobot = obj.posEst.getPoseVec();
            
            fprintf('init map pose x: %2.2f, y: %2.2f, th: %2.2f \n',mapPose.x, mapPose.y, mapPose.th);
            fprintf('init est pose x: %2.2f, y: %2.2f, th: %2.2f \n',obj.posEst.x, obj.posEst.y, obj.posEst.th);
            fprintf('init ref pose x: %2.2f, y: %2.2f, th: %2.2f \n',obj.refRobot(1), obj.refRobot(2), obj.refRobot(3));

            MAP_LOCALIZATION = false;
            
            curve = cubicSpiralTrajectory.planTrajectory(x, y, th,sgn);
            curve.planVelocities(vmax); 
            tau = 0.75;
            kx = 1/tau;
            ky = 2/(tau^2*vmax);
            kth = 1/tau;            
            obj.executeTrajectory(curve, kx, ky, kth);      
            
            mapPose = pose(obj.posEst.getPoseVec());            
            MAP_LOCALIZATION = true;
            fprintf('arrived at  map pose x: %2.2f, y: %2.2f, th: %2.2f \n',mapPose.x, mapPose.y, mapPose.th);
            pause(2.)                        
            posEst_x = obj.posEst.x + 0.25*(mapPose.x-obj.posEst.x);
            posEst_y = obj.posEst.y + 0.25*(mapPose.y-obj.posEst.y);
            thDiff = mapPose.th-obj.posEst.th;
            thDiff = atan2(sin(thDiff), cos(thDiff));
            thAdd  = obj.posEst.th + 0.25*thDiff;
            thAdd  = atan2(sin(thAdd), cos(thAdd));
            posEst_th = thAdd;
            obj.posEst = pose(posEst_x, posEst_y, posEst_th);
            obj.estRobot = poseEstimator(obj.robot, obj.posEst.getPoseVec(), false);
            obj.refRobot = obj.posEst.getPoseVec();
            
            fprintf('arrived at  map pose x: %2.2f, y: %2.2f, th: %2.2f \n',mapPose.x, mapPose.y, mapPose.th);
            fprintf('arrived at  est pose x: %2.2f, y: %2.2f, th: %2.2f \n',obj.posEst.x, obj.posEst.y, obj.posEst.th);                        
            MAP_LOCALIZATION = false;
        end
        
        function executeTrajectory(obj, curve, kx, ky, kth)
            
            firstIteration = false;
            tf  = curve.getTrajectoryDuration();
            
            myController = controller(curve, kx, ky, kth, 1);
            %startPose = obj.estRobot.update();
            startPose = pose(obj.refRobot(1), obj.refRobot(2), obj.refRobot(3)); 
            while true
                if(firstIteration== false)
                    startTic = tic();
                    timePrev = toc(startTic);
                    firstIteration= true;
                end
                timeNow = toc(startTic);
                dt = timeNow - timePrev;
                timePrev = timeNow;
                if timeNow > tf 
                    obj.robot.stop()
                    break;
                end
                currentPose = obj.estRobot.update();
                obj.posEst = pose(currentPose.getPoseVec());
                %fprintf('map pose x: %2.2f, y: %2.2f, th: %2.2f \n',mapPose.x, mapPose.y, mapPose.th)
                obj.refRobot  = startPose.bToA()*curve.getPoseAtTime(timeNow);
                %MAP_LOCALIZATION = true;
                currRef = curve.getPoseAtTime(timeNow);
                currentRefPose = pose(currRef(1), currRef(2), currRef(3));
                refposWorldmat   = startPose.bToA()*currentRefPose.bToA();
                obj.refRobot = pose.matToPoseVec(refposWorldmat);

                [V, w, error] = myController.pidCorrect(timeNow, startPose, currentPose);
                
                [vl , vr] = robotModel.My_VwTovlvr(V, w);
                obj.robot.sendVelocity(robotModel.velScale*vl, vr);
                
                if obj.plotEst==true
                    set(obj.actPosePlot, 'xdata', [get(obj.actPosePlot,'xdata') currentPose.x],...
                    'ydata', [get(obj.actPosePlot,'ydata') currentPose.y]);
                    set(obj.refRobotPlot, 'xdata', [get(obj.refRobotPlot,'xdata') obj.refRobot(1)],...
                    'ydata', [get(obj.refRobotPlot,'ydata') obj.refRobot(2)]);              
                end
                obj.xRefArray(obj.tickCount) = obj.refRobot(1);
                obj.yRefArray(obj.tickCount) = obj.refRobot(2);
                obj.thRefArray(obj.tickCount) = obj.refRobot(3);
                obj.xArray(obj.tickCount) = currentPose.x;
                obj.yArray(obj.tickCount) = currentPose.y;
                obj.thArray(obj.tickCount) = currentPose.th;
                
                obj.tickCount = obj.tickCount + 1;
                pause(0.005);
        
            end
            obj.finalActX = currentPose.x;
            obj.finalActY = currentPose.y;
            obj.finalActTh = currentPose.th;
            obj.finalRefX = obj.refRobot(1);
            obj.finalRefY = obj.refRobot(2);
            obj.finalRefTh = obj.refRobot(3);    
        end
        function moveRelDist(obj, dist, doControlPlotting)
        % move forward or backward a specified distance and
        tPause = 1.;
        sgn = 1;
        vmax = 0.15;%0.25
        amax = 3*0.15;%0.25
        init_dist  = 0;
        init_pose  = pose(0,0,0);
        dt = 0.001;
        if dist < 0
            sgn = -1;
        end
        trapizod = trapezoidalStepReferenceControl(dist, amax, vmax, sgn, tPause);
        tf = trapizod.getTrajectoryDuration();       
        numSamples = int64((tf + tPause + 1)/dt)+1;
        curve = robotTrajectory(numSamples, init_dist, init_pose, dt, trapizod);
        if doControlPlotting
            figure;
            plot(curve.timeArray, curve.velArray, 'x')
        end
        tau = 0.65;
        kx = 1/tau;
        ky = 2/(tau^2*vmax);
        kth = 1/tau;
        obj.executeTrajectory(curve, kx, ky, kth);

        end
        function turnRelAngle(obj,angle, doControlPlotting)
        % make sure the velocity is such that the distance will take at
        % least a second
        % move a distance forward or backward    
        % move forward or backward a specified distance and
        tPause = 1.;
        sgn = 1;
        wmax = 1.;%0.25;
        amax = 3*wmax;
        init_dist  = 0;
        init_pose  = pose(0,0,0);
        dt = 0.001;
        if angle < 0
            sgn = -1;
        end
        trapizod = trapezoidalTurnReferenceControl(angle, amax, wmax, sgn, tPause);
        tf = trapizod.getTrajectoryDuration() ;       
        numSamples = int64((tf + tPause + 1)/dt)+1;
        curve = robotTrajectory(numSamples, init_dist, init_pose, dt, trapizod);
        if doControlPlotting
            figure;
            plot(curve.timeArray, curve.wArray, 'x')
        end
        
        tau = 0.65;%0.65;
        kx = 0;
        ky = 0;
        kth = 1/tau;

        obj.executeTrajectory(curve, kx, ky, kth);

        end
        function executeTrajectoryToAbsolutePose(obj, x, y, th, vmax, sgn, useMap)
            goal_in_w = pose(x, y, th);
            goal_in_r = obj.absToRel(goal_in_w);
            obj.executeTrajectoryToRelativePose(goal_in_r.x, goal_in_r.y,...
                                                                   goal_in_r.th, vmax, sgn, useMap)        
        end
        function goal_in_r = absToRel(obj, goal_in_w)

            goal_in_r = pose(pose.matToPoseVec(obj.posEst.aToB()*goal_in_w.bToA()));
        
        end
        function goal_in_w = relToAbs(obj, goal_in_r)
                       
            goal_in_w = pose(pose.matToPoseVec(obj.posEst.bToA()*goal_in_r.bToA()));            
        end
        function pickDropObject(obj, obj_pos, drop_pos, vmax, sgn, useMap)
            global range_image;
            r = 0.3;
            c = 0.01;
            %approach_angle = 0*pi/180;
            th = -obj_pos(3);
            %x  =  obj_pos(1) - r*sin(approach_angle);
            %y  =  obj_pos(2) - r*cos(approach_angle);
            
            x  = obj_pos(1)-(robotModel.forkOffset+r)*cos(th)-c*sin(th)-0.0;
            y  = obj_pos(2)-(robotModel.forkOffset+r)*sin(th)+c*cos(th);            
            
            acquisition_in_w = pose(x,y,th);
            fprintf('acquisition pose x:%.2f y:%.2f th:%.2f\n', acquisition_in_w.x, acquisition_in_w.y, acquisition_in_w.th);
            acquisition_in_r = obj.absToRel(acquisition_in_w);
            acquisition_bearing = atan2(acquisition_in_r.y, acquisition_in_r.x);
            
            obj.turnRelAngle(acquisition_bearing, 0);
            %obj.turnRelAngle(-pi, 0);
            
            pause(0.05);
            obj.executeTrajectoryToAbsolutePose(x, y, th, vmax, sgn, useMap);
            %obj.turnRelAngle(15*pi/180, 0);
            %pause(0.05);
            %obj.posEst.th
            %th
            %obj.turnRelAngle(5*pi/180, 0);
            pause(0.05);
            found_obj = false;
            finding_clock = tic;
            
            while found_obj == false
                 %obj_pickup_pose = obj.range_obj.findLineCandidate(range_image, 3);
                 obj_pickup_pose = obj.range_obj.findLineCandidateROI(range_image, 5, 20);
                 %obj_pickup_pose = obj.range_obj.findLineCandidate(range_image, 3, 45, 45);
                 
                 %obj_pickup_pose = obj.range_obj.findLineCandidateInROI(range_image, 5, 20);
                 
                 if obj_pickup_pose ~= [0;0;0]
                      found_obj = true;
                      disp('found pallet!!!!!')                   
                 end   
                 disp('looking for pallet first time');
                 find_time = toc(finding_clock);
                 if find_time > 2.
                     finding_clock = tic;                     
                     obj.moveRelDist(-0.02, 0);
                     disp('moving because i couldnt find pallet!!!');
                 end
            end
            
             obj_pickup_pose_w = obj.relToAbs(pose(obj_pickup_pose));
             fprintf('obj pick_up pose %.2f %.2f %.2f\n', obj_pickup_pose_w.x, obj_pickup_pose_w.y, obj_pickup_pose_w.th);
             r = 0.05;
             th = obj_pickup_pose_w.th;
             x  = obj_pickup_pose_w.x-(robotModel.forkOffset+r)*cos(th)-c*sin(th);
             y  = obj_pickup_pose_w.y-(robotModel.forkOffset+r)*sin(th)+c*cos(th);
             th = th - 2*pi/180;
             %approach_angle = 0*pi/180;
             
             %th =  obj_pickup_pose_w.th + approach_angle ;
             %x  =  obj_pickup_pose_w.x + r*sin(approach_angle);
             %y  =  obj_pickup_pose_w.y - r*cos(approach_angle);
             
             obj.executeTrajectoryToAbsolutePose(x, y,th, vmax, sgn, useMap);
             %found_obj = false;
             
             %f = figure;
             %i = 1:360;
             %pause(1.)
             
             while found_obj == false
                
                %valid_r    = range_image <1 & range_image > 0.06; %less than one meters and > 2cm
                %valid_i    = i<20 & i> 340;
                %valid_r = valid_r & valid_i;

                %[x_disp, y_disp, th_disp] = obj.range_obj.irToXy(i(valid_r), range_image(valid_r));
                %obj_pickup_pose = obj.range_obj.findLineCandidate(range_image, 3);
                obj_pickup_pose = obj.range_obj.findLineCandidateROI(range_image, 5, 20);
                %obj_pickup_pose = obj.range_obj.findLineCandidate(range_image, 3, 45, 45);                
                %obj_pickup_pose = obj.range_obj.findLineCandidateInROI(range_image, 5, 10);
                %clf(f);
                %plot(x_disp, y_disp, 'rx');    
                if obj_pickup_pose ~= [0;0;0]
                      found_obj = true;
                      disp('second found pallet!!!!!')
                 end             
                 disp('looking for pallet second time');
                 pause(0.005);
             end
             fprintf('second look pick_up pose %.2f %.2f %.2f\n', obj_pickup_pose(1), obj_pickup_pose(2), obj_pickup_pose(3));
             
             final_pick_bearing = atan2(obj_pickup_pose(2), obj_pickup_pose(1));
             pause(1.)
             fprintf('final bearing %.2f\n', final_pick_bearing)
             
             if abs(final_pick_bearing)> 0.1
             %obj.turnRelAngle(double(final_pick_bearing/4), 0);
             pause(1.)
             end
             r = 0.06;
             obj.robot.forksDown()
             pause(1.)
             obj.moveRelDist(r, 0);
             pause(1.)
             obj.robot.forksUp()
             pause(1.)
             obj.moveRelDist(-r, 0);
             pause(1.)
            
             drop_pos_in_r = obj.absToRel(pose(drop_pos));
             drop_pos_bearing = atan2(drop_pos_in_r.y, drop_pos_in_r.x);
             obj.turnRelAngle(drop_pos_bearing, 0);
             obj.executeTrajectoryToAbsolutePose(drop_pos(1), drop_pos(2),drop_pos(3), vmax, sgn, useMap);
 
             %drop_final_r = obj.absToRel(pose(drop_pos));
             %drop_final_bearing = atan2(drop_final_r.y, drop_final_r.x);           
             %obj.turnRelAngle(drop_final_bearing, 0);
             obj.robot.forksDown()
             pause(1.)
             obj.moveRelDist(-r, 0);
                    
        end
    end
end