function iRobotUndock(u)
%iRobotUndock(u) sends command to undock
% Inputs:
%   u  = BLE object
%
%                           Author: Prof. E. Rodriguez-Seda
%                           Date:   October 28, 2025
decMessage = zeros(1,20);
decMessage(1) = 1;      %Motor device
decMessage(2) = 20;       %Command
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


end