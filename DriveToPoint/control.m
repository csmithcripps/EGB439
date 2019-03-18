classdef control < handle
    
    methods (Static)
        function vel = driveToPoint(q, point, Kh,Kv)
            % Inputs:
            % q is the initial configuration vector (x, y, theta) in units of metres and radians
            % point is the vector (x, y) specifying the goal point of the robot
            if nargin < 3
                Kh = 4;
                Kv = 0.5;
            end
            
            r = sqrt((point(1) - q(1))^2 + (point(2) - q(2))^2);

            if r < 0.1
                vel = [0,0];
            else
                v = Kv * r;

                desiredTheta = atan2((point(2) - q(2)),(point(1)-q(1)));

                omega = Kh * (mod((desiredTheta - q(3))+pi, 2*pi) - pi);

                vel = piBotHelpers.vw2wheels([v,omega]);
            end
        end
            
        function vw = purePursuit(goal, q, d, dt, first)
                % Inputs:
                %  goal is a 2x1 vector giving the current point along the path
                %  q is a 1x3 vector giving the current configuration of the robot
                %  d is the pure pursuit following distance
                %  dt is the time between calls to this function
                %  first is true (1) on the first call in a simulation, otherwise false (0)
                % Return:
                %  vw is a 1x2 vector containing the request velocity and turn rate of the robot [v, omega]

                    % clever stuff
            
            persistent eIntegral
            Kh = 0.3;
            Kv = 0.3;
            Ki = 0.05;
            

            desiredTheta = atan2((goal(2) - q(2)),(goal(1)-q(1)));

            omega = Kh * (mod((desiredTheta - q(3))+pi, 2*pi) - pi);
            
            r = sqrt((goal(1) - q(1))^2 + (goal(2) - q(2))^2);
            
            e = r - d;
            
            if first
                eIntegral = e*dt;
            else
                eIntegral = eIntegral + e*dt;
            end
            
            v = Kv*e + Ki*eIntegral;
            vw =[v,omega];
        end
        
        function vel = AlongLine(q, lineVar, Kh,Kd)
            % Inputs:
            % q is the initial configuration vector (x, y, theta) in units of metres and radians
            % lineVar = (a,b,c)
            if nargin < 3
                Kh = 4;
                Kd = 0.5;
            end
            
            d = lineVar*transpose(q)/sqrt(lineVar(1)^2 + lineVar(2)^2);

            
            v = 1;
            
            desiredTheta = atan2(-lineVar(1),lineVar(2));

            omega = -Kd*d + Kh * (mod((desiredTheta - q(3))+pi, 2*pi) - pi);
            vel = piBotHelpers.vw2wheels([v,omega]);

        end
    end
end