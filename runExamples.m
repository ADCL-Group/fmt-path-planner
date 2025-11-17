% This script is a toplevel script that executes the users desired example case:
setupPath();

u_choice = input(sprintf([ ...
    'Select the test case to run:\n', ...
    '\t(1) Custom Map  –  Single + Multi Waypoints (3D + 4D)\n', ...
    '\t(2) Real Map    –  Multi Waypoints (3D)\n', ...
    '\t(3) Real Map    –  Multi Waypoints (4D)\n', ...
    'User Input: ']));

switch u_choice
    case 1
        fprintf('Running: Custom Map (3D + 4D waypoints)...\n');
        test_customMap_waypoints;

    case 2
        fprintf('Running: Real Map (3D waypoints)...\n');
        test_realMap_waypoints_3D;

    case 3
        fprintf('Running: Real Map (4D waypoints)...\n');
        test_realMap_waypoints_4D;

    otherwise
        fprintf('Invalid selection. Please choose 1, 2, or 3.\n');
        return
end
