 function laserEventListener(handle,event)
        global robot;
        global mapPose;
        global lineMapLoc;
        global MAP_LOCALIZATION;
        global range_image;
        range_im =  robot.laser.LatestMessage.Ranges;
        range_image = range_im;

        if MAP_LOCALIZATION
            index = 1:360;
            maxIters = 150;
            goodOnes = range_im > 0.06 & range_im < 4.0;
            range_im = range_im(goodOnes);
            ths = index(goodOnes);
            [x, y, ~]   = irToXy(ths, range_im);
            ptsInRobotFrame = [x y ones(length(x), 1)]';    
            [success, mapPose, ~] = lineMapLoc.refinePose(mapPose, ptsInRobotFrame, maxIters);
            
            %fprintf('old map pose x: %2.2f, y: %2.2f, th: %2.2f \n',mapPose.x, mapPose.y, mapPose.th);
        end
        
  end