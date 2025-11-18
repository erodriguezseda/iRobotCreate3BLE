function iRobotBeep(u, freq, duration)
%IROBOTBEEP Sends a beep command to an iRobot Create via BLE.
%
%   IROBOTBEEP(U, FREQ, DURATION) sends a sound command to the connected
%   iRobot Create using BLE communication. The frequency and duration are
%   automatically constrained to valid limits.
%
%   Inputs:
%       u        - Structure returned by iRobotConnect (contains BLE and CRC info)
%       freq     - Frequency of the beep in Hz (0–2000)
%       duration - Duration of the beep in seconds (0–10)
%
%   Example:
%       iRobotBeep(u, 880, 0.5)
%
%   Author: Prof. E. Rodriguez-Seda
%   Revised: November 5, 2025

    % --- Validate BLE connection ---
    if ~isfield(u, "dataTx") || isempty(u.dataTx)
        error("Invalid BLE object. Please connect using iRobotConnect first.");
    end

    % --- Sanitize input parameters ---
    freq = max(0, min(freq, 2000));           % Limit to 0–2000 Hz
    duration = max(0, min(duration, 10));     % Limit to 0–10 s
    duration_ms =  duration * 1000;           % Convert to milliseconds

    % --- Build command packet ---
    decMessage = zeros(1, 20);
    decMessage(1) = 5;                        % Device ID
    decMessage(2) = 0;                        % Command ID
    decMessage(3) = u.packetID;               % Packet ID

    % Frequency (4 bytes, int32, little-endian reversed)
    decMessage(4:7) = fliplr(double(typecast(int32(freq), 'uint8')));
    % Duration (2 bytes, int16, little-endian reversed)
    decMessage(8:9) = fliplr(double(typecast(int16(duration_ms), 'uint8')));
    % --- Compute CRC8 checksum ---
    bits = reshape(double(dec2bin(decMessage(1:19), 8).') - '0', [], 1);
    codeword = u.crc8(bits);
    checksumBits = codeword(end-7:end).';
    decMessage(20) = bin2dec(num2str(checksumBits));

    % --- Transmit command ---
    try
        write(u.dataTx, decMessage);
        %fprintf("Beep command sent: %.0f Hz for %.2f s\n", freq, duration);
    catch ME
        warning("Failed to send beep command: %s", ME.message);
        return;
    end

    % --- Update packet ID (wraps around at 255) ---
    u.packetID = mod(u.packetID + 1, 256);
end