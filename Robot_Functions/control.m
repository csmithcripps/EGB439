classdef control < handle
    
    methods (Static)
        function vel = driveToPoint(q, point, Kh,Kv)
            % Inputs:
            % q is the initial configuration vector (x, y, theta) in units of metres and radians
            % point is the vector (x, y) specifying the goal point of the robot
            if nargin < 3
                Kh = 0.2;
                Kv = 0.15;
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
            
        function vel = purePursuit(goal, q, d, dt, first)
            % Follow at a distance (Follow Path)
            % Inputs:
            %  goal is a 2x1 vector giving the current point along the path
            %  q is a 1x3 vector giving the current configuration of the robot
            %  d is the pure pursuit following distance
            %  dt is the time between calls to this function
            %  first is true (1) on the first call in a simulation, otherwise false (0)
            % Return:
            %  vw is a 1x2 vector containing the request velocity and turn rate of the robot [v, omega]
            
            % Controller Gains
            Kh = 5;
            Kv = 0.55;
            Ki = 0.03;

            % Create persistent integral variable
            persistent eIntegral;
            if isempty(eIntegral) || first %
                eIntegral = 0;
            end

            % Convert Pose to variables (Ease of reading)
            x = q(1);
            y = q(2);
            theta = q(3);

            % Calculate Desired angle
            thetaGoal = atan2(goal(2)-y, goal(1)-x);
            h = thetaGoal - theta;
            h = (mod((h)+pi, 2*pi) - pi);

            % Calculate turning speed
            omega = Kh * h;
            omega = min(max(omega,-1), 1);

            % Calculate forward speed
            e = sqrt((goal(1)-x)^2 + (goal(2)-y)^2) - d;
            eIntegral = eIntegral + e * dt;
            v = Kv * e + Ki * eIntegral;
            v = min(v, 0.5);

            % Calculate wheel speeds
            vw = [v omega];
            vel = piBotHelpers.vw2wheels(vw,true);
        end
        
        function vel = AlongLine(q, lineVar, Kh,Kd)
            % Inputs:
            % q is the initial configuration vector (x, y, theta) in units of metres and radians
            % lineVar = (a,b,c)
            
            % Get Controller  Gains
            if nargin < 2
                Kh = 1;
                Kd = 0.5;
            end
            
            % Line Parameters (ax + by + c = 0)
            A = lineVar(1);
            B = lineVar(2);
            C = lineVar(3);
            
            % Find Linear Distance (d = (A,B,C)*(x,y,1)
            d = dot([A,B,C],[q(1),q(2),1])/sqrt(A^2 + B^2);
            
            
            % Find Angular Distance
            desiredTheta = atan2(A,-B);
            h = (mod((desiredTheta - q(3))+pi, 2*pi) - pi);

            omega = Kd*d + Kh * h;
            
            %set v
            v=0.05;
            
            vel = piBotHelpers.vw2wheels([v,omega]);

        end
    end
end