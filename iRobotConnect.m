function u = iRobotConnect(name, velCmdDuration)
%IROBOTCONNECT Establishes a BLE connection with an iRobot Create device.
%
%   U = IROBOTCONNECT(NAME) connects to the iRobot Create via Bluetooth
%   Low Energy (BLE) using the given device name (e.g., "BillTheBot01").
%   The function initializes communication characteristics, subscribes to
%   the receive channel, and prepares CRC configuration and default robot
%   parameters.
%
%   Inputs:
%       name - String specifying the BLE name of the robot.
%       velCmdDuration  - Optional argument, set duration of velocity
%                         commands in seconds (default = 2) 
%
%   Outputs:
%       u    - Struct containing the BLE object, communication
%              characteristics, and robot configuration parameters.
%   Example:
%       u = iRobotConnect("BillTheBot01");
%
%   Author: Prof. E. Rodriguez-Seda
%   Revised: November 5, 2025

    % Initialize output structure
    u = struct();

    if nargin < 2
        velCmdDuration = 2;
    end

    function stopDueToInactivity(~, ~, u)
        try
            iRobotStop(u);
            %fprintf("Robot stopped due to 1s inactivity at %s\n", datestr(now,'HH:MM:SS'));
        catch ME
            warning("Failed to stop robot: %s", ME.message);
        end
    end
   
    try
        % --- Establish BLE connection ---
        fprintf("Connecting to iRobot '%s'...\n", name);
        u.robot = ble(name);
        fprintf("Connected successfully to %s.\n", u.robot.Name);

        % --- Define UART service and characteristics (Nordic UART UUIDs) ---
        serviceUUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
        rxUUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; % RX (read/notify)
        txUUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; % TX (write)

        % --- Access BLE characteristics ---
        u.dataRx = characteristic(u.robot, serviceUUID, rxUUID);
        u.dataTx = characteristic(u.robot, serviceUUID, txUUID);

        % --- Subscribe to the receive channel ---
        subscribe(u.dataRx);
        %fprintf("Subscribed to data receive characteristic.\n");

        % --- Initialize robot communication parameters ---
        u.packetID = 0;

        % CRC configuration (for data integrity)
        u.crc8 = comm.CRCGenerator( ...
            'Polynomial', 'z^8 + z^2 + z + 1', ...
            'InitialConditions', 0, ...
            'DirectMethod', true, ...
            'FinalXOR', 0);

        % --- Robot default configuration parameters ---
        u.velMax = 306;  % Maximum velocity (example units)
        u.x0 = 0;
        u.y0 = 0;
        u.yaw0 = 0;

        u.inactivityTimer = timer('StartDelay', velCmdDuration, ...
                                'ExecutionMode', 'singleShot', ...
                                'TimerFcn', {@stopDueToInactivity, u});

        start(u.inactivityTimer);

        fprintf("iRobot connection setup complete.\n");

    catch ME
        % --- Handle connection or setup errors gracefully ---
        warning("Failed to connect to BLE device '%s'.\nError: %s", name, ME.message);
        u = struct(); % Return empty struct on failure
    end

    % function stopCallback(~, ~, u)
    %     try
    %         iRobotStop(u);
    %     catch ME
    %         warning("iRobot stopCallBack function failed: %s", ME.message);
    %     end
    % end
    % 
    % % --- Create and configure timer ---
    % t = timer;
    % t.ExecutionMode = 'fixedRate';    % Executes at fixed intervals
    % t.Period = 3;                     % Every 2 seconds
    % t.TimerFcn = {@stopCallback, u};  % Pass robot struct to callback
    % % --- Start the timer ---
    % start(t);
    % disp("Beeping every 2 seconds... Press Ctrl+C or run stop(t) to stop.");

end