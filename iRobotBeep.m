function iRobotBeep(u,freq,duration)
%iRobotBeep(u,freq,duration) Sends a sound beep command to iRobot Create
% Inputs:
%   u        = BLE object
%   freq     = Frequency of sound in Hz
%   duration = Duration of sound in seconds
%
%                           Author: Prof. E. Rodriguez-Seda
%                           Date:   October 28, 2025
if freq > 2000
    freq = 2000;
elseif freq < 0
    freq = 0;
end

if duration > 10
    duration = 10;
elseif duration < 0
    duration = 0;
end
duration_ms = duration*1000;

decMessage = zeros(1,20);
decMessage(1) = 5;       %Device
decMessage(2) = 0;       %Command
decMessage(3) = u.packetID;
decMessage(4:7) = fliplr(double(typecast(int32(freq), 'uint8')));
decMessage(8:9) = fliplr(double(typecast(int16(duration_ms), 'uint8')));

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