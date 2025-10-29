function iRobotNavigateTo(u,final_pose)
%iRobotNavigateTo(u,final_pose) sends command to navigate to final_pose.
% Inputs:
%   u           = BLE object
%   final_pose  = 1x3 vector [desired x in m, desired y in m, desired
%                             heading in rad]
%
%                           Author: Prof. E. Rodriguez-Seda
%                           Date:   October 28, 2025

x = (final_pose(1)-u.x0)*1000;  
y = (final_pose(2)-u.y0)*1000;
yaw = wrapTo360((final_pose(3)-u.yaw0 + pi/2)*180/pi)*10; 

decMessage = zeros(1,20);
decMessage(1) = 1;      %Motor device
decMessage(2) = 17;       %Command
decMessage(3) = u.packetID;
decMessage(4:7) = fliplr(double(typecast(int32(x), 'uint8')));
decMessage(8:11) = fliplr(double(typecast(int32(y), 'uint8')));
decMessage(12:13) = fliplr(double(typecast(int16(yaw), 'uint8')));

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
write(u.dataTx,decMessage)
packetID = u.packetID + 1; 
if packetID > 255
    packetID = 0;
end
u.packetID = packetID;

end