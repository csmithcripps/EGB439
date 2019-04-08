classdef Astar1
    
    methods (Static)       
        % this wrapper function allows the assessment code to access your functions

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

        function [newlist,nodeid] = qinsert(list, nodeid, cost)
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
                    newlist = [list(1:i-1) nodeid list(i:end)];
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
    end % static methods
end % classdef