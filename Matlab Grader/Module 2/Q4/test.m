clc;clear all;
%% test qappend function
list = [];
list = myFunction('qappend', list, 1)
list = myFunction('qappend', list, 2)
list = myFunction('qappend', list, 3)

%% test qcontains function
myFunction('qcontains', list, 2)
myFunction('qcontains', list, 7)

%% test qpop function
[list,out] = myFunction('qpop', list)
[list,out] = myFunction('qpop', list)
[list,out] = myFunction('qpop', list)
[list,out] = myFunction('qpop', list)

%% test popmin function
list = [5 4 1 3 4];  % create a list of nodes
% create a list of costs
% cost of node 1 is 8
% cost of node 2 is 7
% cost of node 3 is 6 etc.
% the lowest code node is node 3 with a cost of 1
cost = [8 7 4 7 5]
list = [];
list = myFunction('qinsert', list, 1, cost)
list = myFunction('qinsert', list, 2, cost)
list = myFunction('qinsert', list, 3, cost)
list = myFunction('qinsert', list, 4, cost)
list = myFunction('qinsert', list, 5, cost)

% this should return node 3 and remove it from the list
[list,out] = myFunction('qpop', list)

%% test neighbours function
D = [0  5  1  0  0
     5  0  2  0  3
     1  2  0  1  0
     0  0  1  0  0
     0  3  0  0  0]

nbours = myFunction('neighbours', D, 3)
