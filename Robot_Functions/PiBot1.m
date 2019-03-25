classdef PiBot1 < handle
    %ROBOTSIM A simple simulated version of the 
    
    properties
        q
        dt
        sim
        pb
    end
    
    methods
        function self = robotSim(StartingPose,TimeStep, sim, IP, localiserIP,group)
            %ROBOTSIM Constructs a virtual model of the robot
            self.q = StartingPose;
            self.dt = TimeStep;
            self.sim = sim;
            if ~sim
                self.pb = PiBot(IP,localiserIP,group);
            end
        end

        function update(self,vel)
            if self.sim
                self.q = piBotHelpers.qupdate(self.q, vel, self.dt);
            else
                self.pb.setVelocity(vel);
                self.q = piBotHelpers.getPoseVector(self.pb);
            end
        end
        
        
        function plot(self)
            piBotHelpers.qplot(self.q)
        end


    end
end

