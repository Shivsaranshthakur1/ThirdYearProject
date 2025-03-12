classdef VehicleFactory
    methods(Static)
        function platform = createAerialVehicle(scenario, name, position, color, speed)
            try
                % Input validation
                if nargin < 5
                    error('VehicleFactory:InvalidInput', ...
                        'Not enough input arguments for aerial vehicle creation');
                end
                
                % Position validation
                initialPos = reshape(position, 1, 3);
                if any(isnan(initialPos)) || all(initialPos == 0)
                    error('VehicleFactory:InvalidPosition', ...
                        'Invalid initial position for aerial vehicle: [%.1f, %.1f, %.1f]', ...
                        initialPos(1), initialPos(2), initialPos(3));
                end
                
                % Environment bounds validation (assuming standard dimensions)
                if initialPos(1) < 0 || initialPos(1) > 300 || ...
                   initialPos(2) < 0 || initialPos(2) > 300 || ...
                   initialPos(3) < 10 || initialPos(3) > 80
                    error('VehicleFactory:OutOfBounds', ...
                        'Initial position out of valid bounds: [%.1f, %.1f, %.1f]', ...
                        initialPos(1), initialPos(2), initialPos(3));
                end
                
                fprintf('Creating aerial vehicle %s at position [%.1f, %.1f, %.1f]\n', ...
                    name, initialPos(1), initialPos(2), initialPos(3));
                
                fullName = sprintf('%s_aerial', name);
                initialMotion = [
                    initialPos(1:3), ...  % Position (3)
                    speed, 0, 0, ...      % Velocity - speed in x direction (3)
                    0, 0, 0, ...          % Acceleration (3)
                    1, 0, 0, 0, ...       % Quaternion (4)
                    0, 0, 0               % Angular velocity (3)
                ];
                
                platform = uavPlatform(fullName, scenario, 'ReferenceFrame', 'ENU');
                move(platform, initialMotion);
                
                %%% DEBUG CODE %%%
                currentMotionCheck = read(platform);
                fprintf('DEBUG: After move(), read() for %s => [%.2f, %.2f, %.2f]\n',...
                    fullName, currentMotionCheck(1), currentMotionCheck(2), currentMotionCheck(3));
                fprintf('DEBUG: Velocity => [%.2f, %.2f, %.2f]\n',...
                    currentMotionCheck(4), currentMotionCheck(5), currentMotionCheck(6));
                
                % Add quadrotor mesh
                meshSize = 8; % Make UAV larger and more visible
                updateMesh(platform, 'quadrotor', {meshSize}, color, ...
                    [0 0 0], ...
                    eul2quat([0 0 pi]));
                
                fprintf('Successfully created aerial vehicle %s\n', name);
                
            catch e
                fprintf('Error creating aerial vehicle %s: %s\n', name, e.message);
                fprintf('Stack trace:\n');
                for i = 1:length(e.stack)
                    fprintf(' %s: Line %d\n', e.stack(i).name, e.stack(i).line);
                end
                rethrow(e);
            end
        end
        
        function platform = createGroundVehicle(scenario, name, position, color, speed)
            try
                % Input validation
                if nargin < 5
                    error('VehicleFactory:InvalidInput', ...
                        'Not enough input arguments for ground vehicle creation');
                end
                
                % Position validation
                initialPos = reshape(position, 1, 3);
                initialPos(3) = 0; % Ensure ground vehicle is at z=0
                
                if any(isnan(initialPos)) || any(abs(initialPos(1:2)) > 1e3)
                    error('VehicleFactory:InvalidPosition', ...
                        'Invalid initial position for ground vehicle: [%.1f, %.1f, %.1f]', ...
                        initialPos(1), initialPos(2), initialPos(3));
                end
                
                % Environment bounds validation (assuming standard dimensions)
                if initialPos(1) < 0 || initialPos(1) > 300 || ...
                   initialPos(2) < 0 || initialPos(2) > 300
                    error('VehicleFactory:OutOfBounds', ...
                        'Initial position out of valid bounds: [%.1f, %.1f, %.1f]', ...
                        initialPos(1), initialPos(2), initialPos(3));
                end
                
                fullName = sprintf('%s_ground', name);
                initialMotion = [
                    initialPos(1:3), ...  % Position (3)
                    speed, 0, 0, ...      % Velocity (3)
                    0, 0, 0, ...          % Acceleration (3)
                    1, 0, 0, 0, ...       % Quaternion (4)
                    0, 0, 0               % Angular velocity (3)
                ];
                
                platform = uavPlatform(fullName, scenario, 'ReferenceFrame', 'ENU');
                move(platform, initialMotion);
                
                %%% DEBUG CODE %%%
                currentMotionCheck = read(platform);
                fprintf('DEBUG: After move(), read() for %s => [%.2f, %.2f, %.2f]\n',...
                    fullName, currentMotionCheck(1), currentMotionCheck(2), currentMotionCheck(3));
                fprintf('DEBUG: Velocity => [%.2f, %.2f, %.2f]\n',...
                    currentMotionCheck(4), currentMotionCheck(5), currentMotionCheck(6));
                
                % Add larger cuboid mesh for better visibility
                vehicleSize = [4 3 2]; % [length width height]
                updateMesh(platform, 'cuboid', {vehicleSize}, color, ...
                    [0 0 vehicleSize(3)/2], ...
                    eul2quat([0 0 0]));
                
                fprintf('Successfully created ground vehicle %s\n', name);
                
            catch e
                fprintf('Error creating ground vehicle %s: %s\n', name, e.message);
                fprintf('Stack trace:\n');
                for i = 1:length(e.stack)
                    fprintf(' %s: Line %d\n', e.stack(i).name, e.stack(i).line);
                end
                rethrow(e);
            end
        end
        
        function updateVehiclePosition(platform, newPosition)
            try
                if isempty(platform) || ~isa(platform, 'uavPlatform')
                    error('VehicleFactory:InvalidPlatform', 'Invalid platform object');
                end
                
                newPosition = reshape(newPosition, 1, []);
                if any(isnan(newPosition)) || length(newPosition) ~= 3
                    error('VehicleFactory:InvalidPosition', ...
                        'Invalid new position: %s', mat2str(newPosition));
                end
                
                % For ground vehicles, ensure z=0
                if contains(platform.Name, '_ground')
                    newPosition(3) = 0;
                end
                
                currentMotion = read(platform);
                if isempty(currentMotion)
                    error('VehicleFactory:NoCurrentState', ...
                        'Could not read current vehicle state');
                end
                
                motionVector = currentMotion;
                motionVector(1:3) = newPosition(1:3);
                
                move(platform, motionVector);
                
                fprintf('Updated position for vehicle %s to [%.1f, %.1f, %.1f]\n', ...
                    platform.Name, newPosition(1), newPosition(2), newPosition(3));
                
                %%% DEBUG CODE %%%
                newCheck = read(platform);
                fprintf('DEBUG: After move(), read() => [%.2f, %.2f, %.2f]\n',...
                    newCheck(1), newCheck(2), newCheck(3));
                
            catch e
                fprintf('Error updating position for vehicle %s: %s\n', ...
                    platform.Name, e.message);
                rethrow(e);
            end
        end
        
        function type = getVehicleType(platform)
            if contains(platform.Name, '_aerial')
                type = 'aerial';
            else
                type = 'ground';
            end
        end
        
        function speed = getVehicleSpeed(platform)
            % Get speed from initial velocity x-component
            speed = platform.InitialVelocity(1);
        end
        
        function currentPos = getCurrentPosition(platform)
            try
                if isempty(platform) || ~isa(platform, 'uavPlatform')
                    error('VehicleFactory:InvalidPlatform', 'Invalid platform object');
                end
                
                currentMotion = read(platform);
                if isempty(currentMotion)
                    error('VehicleFactory:NoCurrentState', ...
                        'Could not read current vehicle state');
                end
                
                currentPos = currentMotion(1:3);
                
                % Validate position
                if any(isnan(currentPos))
                    error('VehicleFactory:InvalidPosition', ...
                        'Invalid current position: %s', mat2str(currentPos));
                end
                
                % For ground vehicles, ensure z=0
                if contains(platform.Name, '_ground')
                    currentPos(3) = 0;
                end
                
            catch e
                fprintf('Error getting current position for vehicle %s: %s\n', ...
                    platform.Name, e.message);
                currentPos = [];
            end
        end
    end
end