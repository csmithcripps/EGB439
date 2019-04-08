classdef piBotHelpers
    
    methods (Static)
        function wheelVel = vw2wheels(vw)
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
        
        function vw = wheels2vw(wheelVel)
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
        
        function qd = qdot(q, vel)
            % Inputs:
            % q is the configuration vector (x, y, theta) in units of metres and radians
            % vel is the velocity vector (vleft, vright) each in the range -100 to +100
            % Return:
            % qd is the vector (xdot, ydot, thetadot) in units of metres/s and radians/s

            vw = piBotHelpers.wheels2vw(vel);
            xdot = vw(1)*cos(q(3));
            ydot = vw(1)*sin(q(3));
            thetadot = vw(2);


            qd = [xdot,ydot,thetadot];

        end
        
        function qnew = qupdate(q, vel, dt)
            % Inputs:
            % q is the configuration vector (x, y, theta) in units of metres and radians
            % vel is the velocity vector (vleft, vright) each in the range -100 to +100
            % dt is the length of the integration timestep in units of seconds
            % Return:
            % qnew is the new configuration vector vector (x, y, theta) in units of metres and radians at the
            % end of the time interval.
            qd = piBotHelpers.qdot(q,vel);
            qnew = q + qd * dt;
        end
        
        function [PoseTriangle] = poseToTriangle(x,y,theta)
            %POSETOTRIANGLE Converts an input pose to a triangle plotted
            l = 0.1;
            w = 0.04;
            theta = -theta;
            R = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
            triangle = [0 0; -l +w; -l -w];

            PoseTriangle = triangle*R + [x,y; x,y; x,y];



        end
        
        function q = poseToVector(pose)
            q = [pose.x,pose.y,pose.theta];
        end
        
        function drawTriangle(x,y,theta)
            %DRAWTRIANGLE 

            triangle = piBotHelpers.poseToTriangle(x,y,theta);
            fill(triangle(:,1),triangle(:,2),'b')

        end
        
        function qplot(q,point)
            if isa(q,'struct')
                x = q.x;
                y = q.y;
                theta = q.theta;
            else
                x = q(1);
                y = q(2);
                theta = q(3);
            end
                    
            hold on
        
            piBotHelpers.drawTriangle(x,y,rad2deg(theta));
            set(gca,'Color','g')
            axis([0,2,0,2]);
            
            if nargin >1
                scatter(point(1),point(2),'b')
            end
            hold off
        end
% etc
    end % static methods
end % classdef