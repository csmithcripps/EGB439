% Below is the full source code of a breadth-first search, that makes use of the functions that you debugged in the previous
% question.
% Modify this to use the A* search algorith.

function [path,frontierList,exploredList] = graph_planner(distanceMatrix, placeCoords, placeNames, startNode, goalNode)
    
    parentNode = [];  % parentNode(i) is the number of the node which is the parent of node i
    
    frontierList = startNode;  % put the starting node in the frontier
    exploredList = [];  % nothing has been explored yet
    g(startNode) = 0; % distance travelled to reach each node
    f(startNode) = 0;
    % do the breadth first search
    while ~isempty(frontierList) % while the frontier is not empty
        
        % get next node from frontier list, this is the node we are expanding
        [frontierList, expandingNode] = qpop(frontierList);
        
        fprintf('expanding node %d (%s)\n', expandingNode, placeNames{expandingNode});
        
        % check if we are there yet?
        if expandingNode == goalNode
            break;  % yes
        end
        
        % visit all the neighbours of the node being expanded
        for n = neighbours(distanceMatrix, expandingNode)  % for each neighbour
            if ~qcontains(frontierList, n) && ~qcontains(exploredList, n)
                % this neighbour is not on the frontier and is not explored
                g(n) = g(expandingNode) + distanceMatrix(expandingNode, n); % record the distance travelled
                f(n) = g(n) + heuristic(placeCoords,n,goalNode);
                fprintf('  adding node %d (%s) to frontier, with a cost of %.0f \n', n, placeNames{n}, f(n));
                frontierList = qinsert(frontierList, n,f); % add it to the frontier
                parentNode(n) = expandingNode;  % record how we got to this node
            elseif qcontains(frontierList,n)
                gnew = g(expandingNode) + distanceMatrix(expandingNode, n);
                if gnew <= g(n)
                    % Reparent
                    g(n) = gnew;
                    f(n) = g(n) + heuristic(placeCoords,n,goalNode);
                    parentNode(n) = expandingNode;
                    frontierList = reSort(frontierList, f);
                    fprintf('  reparenting node %d (%s) , with a cost of %.0f \n', n, placeNames{n}, f(n));
                end
            end
        end
        
        % we are done with this node now, add it to the explored list
        exploredList = qappend(exploredList, expandingNode);
        fprintf('  adding node %d %s to explored list\n', expandingNode, placeNames{expandingNode});
    end
    
    % Now we need to reconstruct the path.  For each node we record it's parent
    % node, the node we arrived from.  Starting at the goal node, we find its
    % parent, then the parent of that node, until we get to the start node
    
    path = [];
    n = goalNode;  % set current node to the start
    while true
        path = [n path];    % prepend it to the path
        if n == startNode   % quit now if we have reached the starting node
            return
        end
        n = parentNode(n);  % find the parent of this node
    end
end

function h = heuristic(placeCoords, n, goal)
    % Check the distance (time) from n to goal.
    % Inputs:
    %   placeCoords = list of coordinates (indexed with the nodeids in n and goal)
    %   n           = current node
    %   goal        = goal node
    
    nx = placeCoords(1,n); ny = placeCoords(2,n);
    goalx = placeCoords(1,goal); goaly = placeCoords(2,goal);
    
    distance = sqrt((nx - goalx)^2 + (ny - goaly)^2);
    
    h = distance/(100*1000);
    
end

function newList = reSort(list, cost)
    newList = [];
    for n = list
        newList = qinsert(newList, n, cost);
    end
end

function newlist = qappend(list, nodeid)
    % add the node id to the end of the list and return an updated list 
    %
    % Input:
    %   nodeid  (scalar)
    %   list    (vector)
    %
    % Output:
    %    newlist (vector)


    newlist = [list nodeid];
end

function in = qcontains(list, nodeid)
    % return true if the node id is in the list, otherwise false
    % 
    % Input:
    %   list    (vector)
    %   nodeid  (scalar)
    %
    % Output:
    %   in (logical)

    in = ~isempty(find(list==nodeid,1));
end

function [newlist,nodeid] = qpop(list)
    % get a node id from the front of list and return an updated list. If the list is
    % empty return newlist as and nodeid as the empty value [].
    % 
    % Input:
    %   list    (vector)
    %
    % Output:
    %   newlist (vector)
    %   nodeid  (scalar)
    newlist = list(2:end);
    if isempty(list)
        nodeid = [];
    else
        nodeid = list(1);
    end
end

function newlist = qinsert(list, nodeid, cost)
    % insert the node id into the list and return an updated list.  The node is inserted
    % at a location such that the cost of the nodes is non-decreasing in magnitude.
    % cost is a vector such that cost(i) is the cost of node i. It is guaranteed that 
    % cost will be a vector with length at least equal to the maximum nodeid.
    % If multiple nodes have the same cost, their relative ordering does not matter.
    % 
    % 
    % Input:
    %   list    (vector)
    %   nodeid  (scalar)
    %   cost    (vector)
    %
    % Output:
    %   newlist (vector)

    nodeCost = cost(nodeid);

    for i = 1:length(list)
        if nodeCost < cost(list(i))
            if i == 1
                newlist = [nodeid list];
            else
                newlist = [list(1:i-1) nodeid list(i:end)];
            end
            return;
        end
    end
    newlist = [list nodeid];
end

function nodeid = neighbours(distanceMatrix, nodeid)
    % Return a list of node id's for the neighbours of the given node id
    %
    % Input:
    %   distanceMatrix  (square symmetric matrix)
    %   nodeid          (scalar)
    %   
    % Output:
    %   nodeid   (scalar)
    nodeid = find(distanceMatrix(nodeid,:) > 0);
end