classdef Survivor < handle
    properties
        ID              % Unique identifier
        Position        % [x, y, z] coordinates
        Priority        % 1 (High), 2 (Medium), 3 (Low)
        Status          % 'UNDETECTED', 'IN_PROGRESS', 'DETECTED', 'RESCUED'
        Color           % RGB color based on priority
        AssignedVehicle % ID of vehicle currently assigned to this survivor
        Visited  % NEW: Boolean indicating if this survivor has been visited

        %% ADDED CODE
        TimeTagged      % Timestamp of when survivor was tagged (double)
        TaggedBy        % Vehicle name/ID that tagged the survivor (string or numeric)
    end
    
    methods
        function obj = Survivor(id, position, priority)
            obj.ID = id;
            obj.Position = position;
            obj.Priority = priority;
            obj.Status = 'UNDETECTED';
            obj.AssignedVehicle = [];
            
            % Set color based on priority
            switch priority
                case 1 % High priority
                    obj.Color = [1 0 0];     % Red
                case 2 % Medium priority
                    obj.Color = [1 0.5 0];   % Orange
                case 3 % Low priority
                    obj.Color = [1 1 0];     % Yellow
            end

            %% ADDED CODE: Initialize new properties
            obj.TimeTagged = NaN;
            obj.TaggedBy   = '';
            obj.Visited = false; % NEW
        end
        
        %% ADDED CODE: (Optional) Helper method if you want a direct “tag” function
        function tag(obj, vehicleID, currentTime)
            % Mark this survivor as tagged/detected by the given vehicle at the specified time
            if ~strcmp(obj.Status, 'RESCUED') && ~strcmp(obj.Status, 'DETECTED')
                obj.Status = 'DETECTED'; 
            end
            obj.TimeTagged = currentTime;
            obj.TaggedBy   = vehicleID;
        end
    end
end