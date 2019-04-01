% map = [
%     0 0 0 0 0
%     0 0 0 0 0
%     0 1 1 1 0
%     0 1 1 1 0
%     0 0 0 1 0
%     0 0 0 1 0
%     0 0 0 0 0
%     ];
% goal = [4 2];
% 
% % Run learner solution.
% % check dimensions
% % check first point is start, last point is goal
% 
% dx = myFunction('dxform', map, goal)
map = [
    0 0 0 0 0
    0 0 0 0 0
    0 1 1 1 0
    0 1 1 1 0
    0 0 0 1 0
    0 0 0 1 0
    0 0 0 0 0
    ];
goal = [4 2]; start = [3 6];

% Run learner solution.

path = myFunction('path', map, start, goal)

assert(size(path,2)==2, 'Path should have 2 columns');
assert(all(path(1,:)== start), 'Path should begin with the start point');
assert(all(path(end,:)== goal), 'Path should end with the goal point');