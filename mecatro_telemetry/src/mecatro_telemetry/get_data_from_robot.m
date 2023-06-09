function [log_time, data_values] = get_data_from_robot(port_name, Tmax, ...
    dt_arduino)
% This code is a MATLAB example to handle the fixed-size telemetry provided by the Arduino.
% See format details in MecatroUtils.cpp

% Inputs:
%  port_name:   Name of the serial port used to communicate with the
%               Arduino. Find the list using the serialportlist Matlab command. 
%  Tmax:        Duration of the experiment.
%  dt_arduino:  Value of the CONTROL_LOOP_PERIOD constant in the Arduino 
%               sketch. WARNING: if inconsistent, the duration of the data
%               logging will be off.
% Outputs:
%  log_time:    Vector containing the instants when the data is logged (in
%               ms)
%  data_values: Map (similar to Python dictionary) containing the data
%               values

   
% For efficiency, pre-allocate data holders
MAX_DATA_LENGTH = 1000000;
log_time = zeros(1, MAX_DATA_LENGTH);
data_values = containers.Map();

% Store the index of where we are at in the time vector
current_idx = 0;

% Open serial port at 1Mbps
s = serialport(port_name, 1000000, 'Timeout', 1.0);

% The Arduino will reboot when the port is open, but we might still get some data from the previous run.
% To avoid this, we wait for the Arduino to start its reboot sequence, then clear the input buffer.
pause(0.1);
flush(s);
tic
% myTime = toc;
while true
    % Wait for the start of a frame, aka @ 
    while true
%         myTime = toc;
%         matlabTime(current_idx+1) = myTime;
        b = read(s, 1, 'uint8');
        if char(b) == '@'
            break;
        end
    end
    
    % We have read the @, now we need to decode the message
    % The next 6 bytes are the variable's name, to be interpreted as ascii characters.
    name = char(read(s, 6, 'char'));
    % Remove trailing spaces
    name = strtrim(name);
    
    % Next we read the next 4 bytes, the variable value.
    data = read(s, 4, 'uint8');
    % Now we have two cases: are we dealing with an unsigned int (timestamp) or a float?
    if strcmp(name, 'TS')
        % Read unsigned int32, little endian (i.e. LSB first).
        current_time = typecast(uint8(data), 'uint32');
        current_time = cast(current_time,'double');
        % Increment index and store time
        current_idx = current_idx + 1;
        if current_idx >= MAX_DATA_LENGTH
            error('Experiment too long for buffer - you are very patient!');
        end
        log_time(current_idx) = current_time / 1.0e6;
    else
        % We received a float variable, little endian
        value = typecast(uint8(data), 'single');
        
        % If the variable does not exist, create it
        if ~isKey(data_values, name)
            data_values(name) = zeros(1, MAX_DATA_LENGTH);
        end
        
        % Update the value at the current index
        values = data_values(name);
        values(current_idx) = value;
        data_values(name) = values;
    end
    % For testing: stop after 1000 points i.e. 5s
%     if current_idx > 20000 || myTime > Tmax
    if current_idx > Tmax / dt_arduino%myTime > Tmax
        break;
    end
end
%Remove last time stamp
current_idx = current_idx - 1;
% Crop to remove unused points
log_time = log_time(1:current_idx);
data_values = containers.Map(data_values.keys, cellfun(@(v) v(1:current_idx), data_values.values, 'UniformOutput', false));
    