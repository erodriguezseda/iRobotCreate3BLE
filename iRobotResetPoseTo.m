function u = iRobotResetPoseTo(u,new_pose)
%iRobotResetPoseTo(u, new_pose) Resets the position (x,y) and orientation 
% of iRobot Create to the new_pose in meters and radians. 
%
% Inputs:
%   u        = BLE object
%   new_pose = 1x3 
%
%                           Author: Prof. E. Rodriguez-Seda
%                           Date:   November 30, 2022

decMessage = zeros(1,20);
decMessage(1) = 1;      %Motor device
decMessage(2) = 15;       %Command
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

write(u.dataTx,decMessage);
u.x0 = new_pose(1);
u.y0 = new_pose(2);
u.yaw0 = new_pose(3);
end