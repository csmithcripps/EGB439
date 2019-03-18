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

        function wheelVel = vw2wheels(self, vw)
            % Inputs:
            % vw is the velocity vector (v, omega) in units of metres/s and radians/s
            % Return:
            % wheelVel is the wheel velocity vector (vleft, vright) each in the range -100 to +100 to achieve
            % this velocity   

            W = 0.16; % Lateral Wheel Spacing
            d = 0.065; % Wheel Diameter

            vdelta = vw(2) * W;
            vlinear = [-1,1;1,1] \ [vdelta ; 2 * vw(1)]; %[vl;vr]


            wheelVel = vlinear* ((60*2)/(pi*d));
            wheelVel(1) = min(max(wheelVel(1),-100),100);
            wheelVel(2) = min(max(wheelVel(2),-100),100);

            wheelVel = transpose(wheelVel);
        end

        function vw = wheels2vw(self, wheelVel)
            % Inputs:
            % wheelVel is the wheel velocity vector (vleft, vright) each in the range -100 to +100
            % Return:
            % vw is the resulting velocity vector (v, omega) in units of metres/s and radians/s
            d = 0.065;
            W = 0.16; % Lateral Wheel Spacing
            vlinear = ((wheelVel /2) * pi*d)/60; % Convert from wheelVel to linear Velocities.

            vdelta = vlinear(2) - vlinear(1);

            v = 0.5 * (vlinear(1) + vlinear(2));
            omega = vdelta/W;

            vw = [v , omega];
        end

        function qd = qdot(self, q, vel)
            % Inputs:
            % q is the configuration vector (x, y, theta) in units of metres and radians
            % vel is the velocity vector (vleft, vright) each in the range -100 to +100
            % Return:
            % qd is the vector (xdot, ydot, thetadot) in units of metres/s and radians/s

            vw = self.wheels2vw(vel);
            xdot = vw(1)*cos(q(3));
            ydot = vw(1)*sin(q(3));

            qd = [xdot,ydot,vw(2)];

        end

        function update(self,vel)
            % Inputs:
            % q is the configuration vector (x, y, theta) in units of metres and radians
            % vel is the velocity vector (vleft, vright) each in the range -100 to +100
            % dt is the length of the integration timestep in units of seconds
            % Return:
            % qnew is the new configuration vector vector (x, y, theta) in units of metres and radians at the
            % end of the time interval..
            qd = self.qdot(self.q, vel);
            qnew = self.q + qd * self.dt;
            self.q = qnew;
        end
        
        
        function plot(self,lineVar)
            x = self.q(1);
            y = self.q(2);
            theta = self.q(3);


            hold on

            piBotHelpers.drawTriangle(x,y,rad2deg(theta));
            set(gca,'Color','g')
            axis([0,2,0,2]);
            if nargin > 1
                
                x = 0:0.1:2;
                y = ((-lineVar(1)*x)/lineVar(2))- lineVar(3)/lineVar(2);
                plot(x,y,'r')
            end

            hold off
        end


    end
end

