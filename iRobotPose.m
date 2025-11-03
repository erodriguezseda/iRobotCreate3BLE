function pose = iRobotPose(u)
%iRobotPose(u) Request the position (x,y) and orientation of iRobot Create
% Inputs:
%   u    = BLE object
% Outputs:
%   pose = A 1x3 vector containing the position and orientation of the
%   robot (x,y,yaw). Position is in meters while orientation is in radians.
%
%                           Author: Prof. E. Rodriguez-Seda
%                           Date:   November 30, 2022

decMessage = zeros(1,20);
decMessage(1) = 1;      %Motor device
decMessage(2) = 16;       %Command
decMessage(3) = u.packetID;

tempString = dec2bin(decMessage(1:19),8);
inputMsg = zeros(19*8,1);
for i = 1:19
    for j = 1:8
        inputMsg(8*(i-1)+j) = str2double(tempString(i,j));
    end
end

codeword = u.crc8(inputMsg);
checksum = codeword(end-7:end)';
decMessage(20) = bin2dec(num2str(checksum));

getPose = 1;
tStart = tic; 
while getPose
    write(u.dataTx,decMessage)
    dataRead = read(u.dataRx);
    if dataRead(2) == 16
        getPose = 0;
        x90 = double(typecast(fliplr(uint8(dataRead(8:11))), 'int32'));  %in mm
        y90 = double(typecast(fliplr(uint8(dataRead(12:15))), 'int32')); %in mm
        x = cos(-pi/2)*x90 - sin(-pi/2)*y90;
        y = sin(-pi/2)*x90 + cos(-pi/2)*y90;
        yaw = typecast(fliplr(uint8(dataRead(16:17))), 'int16'); %in deci-degrees
        yaw = double(yaw) - 900;
        yaw_rad = pi/180*yaw/10;
        pose_x = double(x/1000) + u.x0;
        pose_y = double(y/1000) + u.y0;
        pose_yaw = wrapToPi(double(yaw_rad) + u.yaw0);
        %pose = double([x/1000,y/1000,yaw_rad]) + [u.x0, u.y0, u.yaw0];
        pose = [pose_x, pose_y, pose_yaw];
    end
    if toc(tStart) > 1
        warning('Timeout, took longer than 1 seconds to get pose.')
        break;
    end

end

packetID = u.packetID + 1; 
if packetID > 255
    packetID = 0;
end
u.packetID = packetID;


end