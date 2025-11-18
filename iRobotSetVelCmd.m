function iRobotSetVelCmd(u, v, w)
%IROBOTSETVELCMD Sends linear and angular velocity commands to iRobot Create.
%
%   IROBOTSETVELCMD(U, V, W) transmits a velocity command via BLE to the
%   iRobot Create platform, setting the desired forward and angular
%   velocities. The function automatically constrains motor commands within
%   the maximum allowable range.
%
%   Inputs:
%       u - Structure returned by iRobotConnect (contains BLE and CRC info)
%       v - Forward velocity (m/s), range: approximately [-0.306, 0.306]
%       w - Angular velocity (rad/s)
%
%   Example:
%       iRobotSetVelCmd(u, 0.1, 0.5);
%
%   Author: Prof. E. Rodriguez-Seda
%   Revised: November 5, 2025

    try
        % --- Validate BLE object ---
        if nargin < 3 || ~isstruct(u) || ~isfield(u, "dataTx")
            error("Invalid robot object. Please connect using iRobotConnect first.");
        end

        % --- Verify velocity limits ---
        if u.velMax <= 0 || u.velMax > 306
            u.velMax = 306;
            warning("Overriding maximum velocity to 0.306 m/s.");
        end

        % --- Compute wheel velocities (m/s -> mm/s) ---
        wheelBase = 0.235;             % Wheel separation (m)
        vL = v - (wheelBase / 2) * w;  % Left wheel linear velocity (m/s)
        vR = v + (wheelBase / 2) * w;  % Right wheel linear velocity (m/s)

        % Convert to mm/s and round
        vL = round(1000 * vL);
        vR = round(1000 * vR);

        % --- Saturate velocities to limits ---
        vL = max(-u.velMax, min(vL, u.velMax));
        vR = max(-u.velMax, min(vR, u.velMax));

        % --- Build BLE message ---
        decMessage = zeros(1, 20);
        decMessage(1) = 1;      % Motor device ID
        decMessage(2) = 4;      % Command ID (velocity control)
        decMessage(3) = u.packetID;

        % Left and right velocities (int32, little-endian reversed)
        decMessage(4:7)  = fliplr(double(typecast(int32(vL), 'uint8')));
        decMessage(8:11) = fliplr(double(typecast(int32(vR), 'uint8')));

        % --- Compute CRC8 checksum (vectorized) ---
        bits = reshape(double(dec2bin(decMessage(1:19), 8).') - '0', [], 1);
        codeword = u.crc8(bits);
        checksumBits = codeword(end-7:end).';
        decMessage(20) = bin2dec(num2str(checksumBits));

        % --- Send command to robot ---
        write(u.dataTx, decMessage);

        % --- Update packet ID (wrap-around at 255) ---
        u.packetID = mod(u.packetID + 1, 256);

        stop(u.inactivityTimer);
        start(u.inactivityTimer);

        % --- Optional feedback ---
        % fprintf("Velocity command sent | v = %.3f m/s | w = %.3f rad/s | Time: %s\n", ...
        %         v, w, datestr(now, 'HH:MM:SS'));

    catch ME
        % --- Handle transmission or logic errors gracefully ---
        warning("Failed to send velocity command: %s", ME.message);
    end
end