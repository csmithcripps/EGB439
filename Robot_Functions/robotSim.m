classdef robotSim < handle
    %ROBOTSIM A simple simulated version of the 
    
    properties
        q
        dt
    end
    
    methods
        function self = robotSim(StartingPose,TimeStep)
            %ROBOTSIM Constructs a virtual model of the robot
            self.q = StartingPose;
            self.dt = TimeStep;
        end

        function update(self,vel)
            qd = piBotHelpers.qdot(self.q, vel);
            qnew = self.q + qd * self.dt;
            self.q = qnew;
        end
        
        
        function plot(self)
            hold on
            piBotHelpers.qplot(self.q)
        end


    end
end

