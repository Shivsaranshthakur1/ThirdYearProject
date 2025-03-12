classdef searchRescueEnvironment < handle
    properties
        scenario        % UAV scenario object
        dimensions      % Search area dimensions [length width height]
        buildingData    % Structure containing building information
        obstacles       % Structure containing obstacle information
        buildingList    % Array of building structures with positions and dimensions
        occupancyMap    % 3D occupancy map for path planning
    end
    
    methods
        function obj = searchRescueEnvironment(dimensions)
            % Initialize the environment with dimensions and 3D occupancy map
            
            %%% DEBUG CODE %%%
            fprintf('DEBUG: Creating searchRescueEnvironment\n');
            fprintf('      Using environment dims = [%.2f, %.2f, %.2f]\n', ...
                    dimensions(1), dimensions(2), dimensions(3));

            obj.dimensions = dimensions;
            
            % Create a UAV scenario with a reference location at [0, 0, 0]
            obj.scenario = uavScenario('UpdateRate', 100, ...
                                       'StopTime', Inf, ...
                                       'ReferenceLocation', [0 0 0]);

            %%% DEBUG CODE %%%
            refLoc = obj.scenario.ReferenceLocation;
            fprintf('DEBUG: uavScenario referenceLocation = [%.2f, %.2f, %.2f]\n', ...
                refLoc(1), refLoc(2), refLoc(3));
            fprintf('DEBUG: => All environment coordinates are in meters, origin at [0,0,0]\n');
            
            % Initialize 3D occupancy map (1 cell per meter)
            resolution = 1; % 1 cell per meter
            obj.occupancyMap = occupancyMap3D(resolution);
            
            % Initialize all space as free
            obj.initializeFreeSpace();
            
            % Create environment components
            obj.createGround();
            obj.createBuildings();
            obj.createObstacles();
        end
        
        function initializeFreeSpace(obj)
            fprintf('Initializing free space in environment...\n');
            
            % We fill [0..dimensions(1), 0..dimensions(2), 0..dimensions(3)] with FREE
            [X, Y, Z] = meshgrid(0:obj.occupancyMap.Resolution:obj.dimensions(1), ...
                                 0:obj.occupancyMap.Resolution:obj.dimensions(2), ...
                                 0:obj.occupancyMap.Resolution:obj.dimensions(3));
            points = [X(:) Y(:) Z(:)];
            setOccupancy(obj.occupancyMap, points, zeros(size(points,1), 1));
        end
        
        function createGround(obj)
            groundVertices = [
                0                 0;
                obj.dimensions(1) 0;
                obj.dimensions(1) obj.dimensions(2);
                0                 obj.dimensions(2)
            ];
            
            % Add ground mesh
            addMesh(obj.scenario, 'Polygon', ...
                    {groundVertices, [-1 0]}, ...
                    [0.7 0.7 0.7]); % grey color
        end
        
        function createBuildings(obj)
            % Define building configurations [x, y, width, length, height]
            buildingConfigs = [
                [50,  50,  20, 30, 40];
                [100, 80,  25, 25, 35];
                [150, 60,  15, 45, 25];
                [200, 120, 30, 30, 45];
                
                [75,  150, 20, 20, 20];
                [120, 180, 15, 25, 15];
                
                [180, 40,  10, 10, 10];
                [220, 90,  12, 12, 12]
            ];
            
            obj.buildingList = struct('position', cell(1, size(buildingConfigs, 1)), ...
                                      'dimensions', cell(1, size(buildingConfigs, 1)));
            
            for i = 1:size(buildingConfigs, 1)
                config = buildingConfigs(i, :);
                vertices = obj.createBuildingVertices(config);
                
                obj.buildingList(i).position = [config(1), config(2), 0];
                obj.buildingList(i).dimensions = [config(3), config(4), config(5)];
                
                % Add building polygon mesh
                addMesh(obj.scenario, 'Polygon', ...
                        {vertices, [0 config(5)]}, ...
                        [0.8 0.8 0.8]);
                
                obj.addBuildingToOccupancyMap(obj.buildingList(i));
            end
        end
        
        function vertices = createBuildingVertices(~, config)
            x = config(1);
            y = config(2);
            width = config(3);
            lengthVal = config(4);
            
            vertices = [
                x          y;
                x + width  y;
                x + width  y + lengthVal;
                x          y + lengthVal
            ];
        end
        
        function addBuildingToOccupancyMap(obj, building)
            xRange = (building.position(1)):obj.occupancyMap.Resolution:(building.position(1) + building.dimensions(1));
            yRange = (building.position(2)):obj.occupancyMap.Resolution:(building.position(2) + building.dimensions(2));
            zRange = (building.position(3)):obj.occupancyMap.Resolution:(building.position(3) + building.dimensions(3));
            
            [X, Y, Z] = meshgrid(xRange, yRange, zRange);
            points = [X(:) Y(:) Z(:)];
            
            setOccupancy(obj.occupancyMap, points, ones(size(points,1), 1));
        end
        
        function createObstacles(obj)
            % [x, y, radius, height]
            obstacleConfigs = [
                [180, 40,  3, 50];
                [90,  120, 2, 30];
                [220, 180, 4, 40];
                
                [140, 70,  2, 20];
                [160, 150, 2, 20];
                [60,  200, 2, 20];
            ];
            
            obj.obstacles = struct('config', obstacleConfigs);
            
            for i = 1:size(obstacleConfigs, 1)
                config = obstacleConfigs(i, :);
                
                addMesh(obj.scenario, 'Cylinder', ...
                    {[config(1) config(2) config(3)], [0 config(4)]}, ...
                    [0.6 0.6 0.6]);
                
                [X, Y, Z] = cylinder(config(3), 20);
                X = X * config(3) + config(1);
                Y = Y * config(3) + config(2);
                Z = Z * config(4);
                points = [X(:) Y(:) Z(:)];
                setOccupancy(obj.occupancyMap, points, ones(size(points,1), 1));
            end
        end
        
        function buildings = getBuildings(obj)
            buildings = obj.buildingList;
        end
        
        function map = getOccupancyMap(obj)
            map = obj.occupancyMap;
        end
        
        function show(obj)
            try
                ax = gca;
                hold(ax, 'on');
                
                show(obj.occupancyMap, 'Parent', ax);
                
                for i = 1:length(obj.buildingList)
                    building = obj.buildingList(i);
                    pos = building.position;
                    dims = building.dimensions;
                    
                    [X,Y,Z] = obj.createBuildingBox(pos, dims);
                    
                    h1 = fill3(ax, X(:,[1 2 3 4 1])', Y(:,[1 2 3 4 1])', Z(:,[1 2 3 4 1])', ...
                        [0.8 0.8 0.8], 'EdgeColor', [0.5 0.5 0.5]);
                    h2 = fill3(ax, X(:,[5 6 7 8 5])', Y(:,[5 6 7 8 5])', Z(:,[5 6 7 8 5])', ...
                        [0.8 0.8 0.8], 'EdgeColor', [0.5 0.5 0.5]);
                    h3 = fill3(ax, X(:,[1 5 8 4 1])', Y(:,[1 5 8 4 1])', Z(:,[1 5 8 4 1])', ...
                        [0.7 0.7 0.7], 'EdgeColor', [0.5 0.5 0.5]);
                    h4 = fill3(ax, X(:,[2 6 7 3 2])', Y(:,[2 6 7 3 2])', Z(:,[2 6 7 3 2])', ...
                        [0.7 0.7 0.7], 'EdgeColor', [0.5 0.5 0.5]);
                    h5 = fill3(ax, X(:,[4 8 7 3 4])', Y(:,[4 8 7 3 4 1])', Z(:,[4 8 7 3 4])', ...
                        [0.9 0.9 0.9], 'EdgeColor', [0.5 0.5 0.5]);
                    h6 = fill3(ax, X(:,[1 5 6 2 1])', Y(:,[1 5 6 2 1])', Z(:,[1 5 6 2 1])', ...
                        [0.9 0.9 0.9], 'EdgeColor', [0.5 0.5 0.5]);
                    
                    set([h1 h2 h3 h4 h5 h6], 'Tag', 'building');
                end
                
                for i = 1:size(obj.obstacles.config, 1)
                    config = obj.obstacles.config(i,:);
                    [X,Y,Z] = cylinder(config(3), 20);
                    X = X * config(3) + config(1);
                    Y = Y * config(3) + config(2);
                    Z = Z * config(4);
                    surf(ax, X, Y, Z, 'FaceColor', [0.6 0.6 0.6], ...
                        'EdgeColor', 'none', 'Tag', 'building');
                end
                
            catch e
                fprintf('Error in environment show: %s\n', getReport(e));
            end
        end

        function [X,Y,Z] = createBuildingBox(obj, pos, dims)
            x = pos(1) + [0 dims(1) dims(1) 0 0 dims(1) dims(1) 0];
            y = pos(2) + [0 0 dims(2) dims(2) 0 0 dims(2) dims(2)];
            z = pos(3) + [0 0 0 0 dims(3) dims(3) dims(3) dims(3)];
            
            X = reshape(x, 1, 8);
            Y = reshape(y, 1, 8);
            Z = reshape(z, 1, 8);
        end
    end
end