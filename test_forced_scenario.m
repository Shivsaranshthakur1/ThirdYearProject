function test_forced_scenario
    % Minimal forced-scenario test for UAV detection

    clear; clc; close all;

    % 1) Create smaller environment
    dims = [100, 100, 50];
    env = searchRescueEnvironment(dims);

    % 2) Create RRT controller with 1 aerial, 0 ground
    controller = centralController_RRT(env, 1, 0);

    % 3) Force the first survivor to [50,50,0] 
    if ~isempty(controller.survivorManager.Survivors)
        controller.survivorManager.Survivors(1).Position = [50,50,0];
        controller.survivorManager.Survivors(1).Status   = 'UNDETECTED';
        controller.survivorManager.Survivors(1).Priority = 1;
        fprintf('DEBUG: Survivor1 forced to [50,50,0]\n');
    end

    % 4) Force the UAV to [50,50,10]
    if ~isempty(controller.aerialPlatforms)
        currentMotion = controller.aerialPlatforms{1}.read();
        newMotion = currentMotion;
        newMotion(1:3) = [50, 50, 10];
        move(controller.aerialPlatforms{1}, newMotion);
        fprintf('DEBUG: UAV1 forced to [50,50,10]\n');
    end

    % Optional: Try a direct plan from [50,50,10] to [50,50,0]
    startPos = [50,50,10];
    goalPos  = [50,50,0];
    [success, path] = controller.planPath(startPos, goalPos, true);
    if success
        fprintf('DEBUG: Hard-coded path success. Path length: %.2f\n', ...
                sum(sqrt(sum(diff(path(:,1:3)).^2,2))));
    else
        fprintf('DEBUG: Hard-coded planPath failed\n');
    end

    % 5) Optionally start the mission to see if detection triggers
    %     If you want a minimal "mission" logic:
    fprintf('\nStarting minimal mission...\n');
    controller.startMission();  % Or just skip if you only wanted the forced planPath test
    
    fprintf('\nDone with forced scenario test.\n');
end