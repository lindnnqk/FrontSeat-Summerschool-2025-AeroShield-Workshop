classdef SerialReader < handle
    properties
        port            % COM port or device name (e.g. 'COM3', '/dev/ttyUSB0')
        baudRate = 9600 % Default baud rate (adjust if needed)
        serialObj       % Serialport object
        data            % Matrix with collected data
    end
    
    methods
        % Constructor
        function obj = SerialReader(port, baudRate)
            if nargin > 0
                obj.port = port;
            end
            if nargin > 1
                obj.baudRate = baudRate;
            end
            
            % Create serialport object
            obj.serialObj = serialport(obj.port, obj.baudRate);
            
            % Configure callbacks
            configureCallback(obj.serialObj, "terminator", @(src, evt)obj.readLine());
            
            % Initialize storage
            obj.data = [];
        end
        
        % Callback function for each new line
        function readLine(obj)
            try
                line = readline(obj.serialObj);         % Read one line
                nums = str2double(split(line, ','));    % Convert comma-separated string to numbers
                if all(~isnan(nums))                    % Check valid numbers
                    obj.data = [obj.data; nums'];       % Append row
                end
            catch ME
                warning("Error reading serial data: %s", ME.message);
            end
        end
        
        % Stop reading & clear callback
        function stop(obj)
            configureCallback(obj.serialObj, "off");
        end
        
        % Destructor (cleanup automatically)
        function delete(obj)
            if ~isempty(obj.serialObj)
                configureCallback(obj.serialObj, "off");
                clear obj.serialObj
            end
        end
    end
end
