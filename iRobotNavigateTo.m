function iRobotNavigateTo(u, final_pose)
% iRobotNavigateTo(u, final_pose)
% Sends a command to the iRobot to navigate to a specified pose.
%
% Inputs:
%   u          - iRobot BLE object
%   final_pose - 1x3 vector [x (m), y (m), heading (rad)]
%
% Example:
%   iRobotNavigateTo(robot, [1.2, 0.5, pi/4])
%
% Author: Prof. E. Rodriguez-Seda
% Date:   October 28, 2025

    %-------------------------------%
    % Validate inputs
    %-------------------------------%
    if nargin < 2
        error('Not enough input arguments. Usage: iRobotNavigateTo(u, final_pose)');
    end
    if ~isvector(final_pose) || numel(final_pose) ~= 3
        error('final_pose must be a 1x3 vector: [x, y, heading].');
    end

    %-------------------------------%
    % Compute relative position and orientation
    %-------------------------------%
    dx = (final_pose(1) - u.x0) * 1000;  % convert to mm
    dy = (final_pose(2) - u.y0) * 1000;
    
    % Rotate by -pi/2 to align with robot frame
    x = cos(-pi/2) * dx + sin(-pi/2) * dy;
    y = -sin(-pi/2) * dx + cos(-pi/2) * dy;
    
    % Convert heading to tenths of degrees (0–3600)
    yaw = wrapTo360((final_pose(3) - u.yaw0 + pi/2) * 180/pi) * 10;

    %-------------------------------%
    % Build command message
    %-------------------------------%
    decMessage = zeros(1, 20);
    decMessage(1)  = 1;                 % Motor device ID
    decMessage(2)  = 17;                % Navigation command
    decMessage(3)  = u.packetID;        % Packet counter

    % Encode x, y, and yaw
    decMessage(4:7)   = fliplr(double(typecast(int32(x), 'uint8')));
    decMessage(8:11)  = fliplr(double(typecast(int32(y), 'uint8')));
    decMessage(12:13) = fliplr(double(typecast(int16(yaw), 'uint8')));

    %-------------------------------%
    % Compute CRC8 checksum
    %-------------------------------%
    tempString = dec2bin(decMessage(1:19), 8);
    inputMsg = zeros(19*8,1);
    for i = 1:19
        for j = 1:8
            inputMsg(8*(i-1)+j) = str2double(tempString(i,j));
        end
    end
    %inputMsg = reshape(str2double(tempString.'), [], 1); % Flatten column-wise
    codeword = u.crc8(inputMsg);
    checksum = codeword(end-7:end)';
    decMessage(20) = bin2dec(num2str(checksum));

    %-------------------------------%
    % Send message and update packet ID
    %-------------------------------%
    write(u.dataTx, decMessage);

    u.packetID = mod(u.packetID + 1, 256); % Wrap around at 255

end