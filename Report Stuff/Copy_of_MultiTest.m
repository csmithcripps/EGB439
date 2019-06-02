%% Test Multiple R and Q values
clear variables
close all
addpath('../Robot_Functions')
error = [0 0];


% R dN
RdN = 0.01:0.1:0.5;

% R thetaN value
RthetaN = 1:1:10;

% Q dN
QdN = 0.5:0.15:2;

% Q thetaN value
QthetaN = 5:2:30;

error = inf;

for rd = RdN
    for rt = RthetaN
        for qd = QdN
            for qt = QthetaN
                R1 = diag([rd rt*pi/180]).^2;
                Q1 = diag([qd qt*pi/180]).^2;
                newError = SimTest(R1,Q1,10,0);
                if newError < error
                    bestVals = [rd, rt, qd, qt];
                    error = newError;
                end
                disp([rd,rt,qd,qt])
            end
        end
    end
end
%%
rd = bestVals(1);
rt = bestVals(2);
qd = bestVals(3);
qt = bestVals(4);


R1 = diag([rd rt*pi/180]).^2;
Q1 = diag([qd qt*pi/180]).^2;
newError = SimTest(R1,Q1,10);












