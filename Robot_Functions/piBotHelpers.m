classdef piBotHelpers
    
    methods (Static)       
        
        function q = getPoseVector(pb)
            % Takes a Pibot object and returns the object's pose as
            % a vector rather than a struct
            pose = pb.getLocalizerPose.pose;
            q = [pose.x,pose.y,deg2rad(pose.theta)];
        end        
        
        function wheelVel = vw2wheels(vw, noLim)
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
            if nargin < 2
                wheelVel(1) = min(max(wheelVel(1),-100),100);
                wheelVel(2) = min(max(wheelVel(2),-100),100);
            else
                if ~noLim
                    wheelVel(1) = min(max(wheelVel(1),-100),100);
                    wheelVel(2) = min(max(wheelVel(2),-100),100);
                end
            end
                    

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
        
        function drawTriangle(x,y,theta)
            %DRAWTRIANGLE 
            l = 0.1; %Length of Triangle
            w = 0.04; %Width of Triangle
            
            theta = -theta; %Theta needs to be inverted
            %
            R = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
            triangle = [0 0; -l +w; -l -w];
            
            triangle = triangle*R + [x,y; x,y; x,y];
            
            fill(triangle(:,1),triangle(:,2),'b')

        end
        
        function qplot(q)
            % Plot Given a pose q. Optional goal point can also be input,
            % which will be drawn as a blue pentagram
            
            if isa(q,'struct')
                x = q.x;
                y = q.y;
                theta = q.theta;
            else
                x = q(1);
                y = q(2);
                theta = q(3);
            end
            
            
            persistent previous;
            if isempty(previous)
                previous = [x y];
            end
            if ~(x == 0 && y == 0 && theta == 0)
                previous = [previous; x y];
            end
                    
            hold on
           
            piBotHelpers.drawTriangle(x,y,rad2deg(theta));
            set(gca,'Color','g')
            axis([0,2,0,2]);
            axis 'square'
            grid on
            plot(previous(:,1),previous(:,2))
        end
% etc
    end % static methods
end % classdef