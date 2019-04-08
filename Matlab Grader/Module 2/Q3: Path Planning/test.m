map = [
    0 0 0 0 0 0 0 0 0 0 1
    0 0 0 0 0 0 0 0 0 1 1
    0 1 1 1 1 1 0 0 1 1 1
    0 1 1 1 1 1 0 0 1 0 1
    0 1 0 1 0 0 0 1 1 0 0
    0 1 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0 0
    ];
start = [4 1]
goal = [4 6]

% test the distance transform
dtransform = myFunction('dxform', map, goal)

% test the path generation
path = myFunction('path', map, start, goal)


idisp(dtransform)
hold on
plot(path(:,1),path(:,2))

assert(size(path,2)==2, 'Path should have 2 columns');
assert(all(path(1,:)== start), 'Path should begin with the start point');
assert(all(path(end,:)== goal), 'Path should end with the goal point');