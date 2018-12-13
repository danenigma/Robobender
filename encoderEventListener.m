 function encoderEventListener(handle,event)
        global robot;
        global encoderDataTimeStamp;
        encoderDataTimeStamp = double(event.Header.Stamp.Sec) + ...
		double(event.Header.Stamp.Nsec)/1000000000.0;
        robot.encoders.LatestMessage.Vector.X
        

  end