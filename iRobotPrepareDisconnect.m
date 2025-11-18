function iRobotPrepareDisconnect(u)
%   iRobotDisconnect(u) deletes the inactivity timer of the robot as 
%   preamble to disconnect. To finally disconnect use the clear command.
%   
%   Example: 
%        iRobotPreDisconnect(u)
%        clear u
%
%   Author: Prof. E. Rodriguez-Seda
%   Revised: November 5, 2025

    try
        u.inactivityTimer.delete
        disp('iRobot ' + u.robot.Name + ' is ready to disconnect.')
    catch ME
        warning("Failed to prepare iRobot: %s", ME.message);
    end

end