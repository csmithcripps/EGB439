%this function will test your implementation of the ekf-localiser 
%complete the tow functions predict_step and update_step. Also, write any extra functions you need. For example, you can write functions to calculate the jacobians.
function [output] = myFunction(mode)    
    load_data();  
    map = get_map();
    switch mode
        % case 0 test the full simulation whereas case 1 and 2 test only one step. 
        case 0        
            % this simulator run for 50 steps.
            nsteps = 50;        
            scatter(map(:,1),map(:,2),200,'k*');
            hold on
            % pose of the robo at time step 0
            x = [0 0 0]'; 
            % The initial covariance matrix on the robot pose
            S = diag([1 1 5*pi/180]).^2;
            % The covariance matrices of the process and the measurement noise 
            R = diag([.01 3*pi/180]).^2;
            Q = diag([.1 3*pi/180]).^2;
            % run for 50 steps
            for k = 1:nsteps
                [d,dth]  = get_odom(k);
                    
                [x,S] = predict_step(x,S,d,dth,R);
                % The sensor measurements to all the landmarks at the 
                % current time step. z is a matrix of shape nx2 
                % where n is the number of landmarks in the map. 
                % The first column in z is the range (m) and
                % the second column is the bearing (rad).    
                z = sense(k);
        
                [x,S] = update_step(map,z,x,S,Q);    
                
                plot_cov(x,S,3);     
%                 plot_robot(x,15)
            end
            output = [x,S];
        case 1
            x = [0 0 0]'; 
            S = diag([1 1 5*pi/180]).^2;
            R = diag([.01 3*pi/180]).^2; 
            [d,dth]  = get_odom(1);
            [x,S] = predict_step(x,S,d,dth,R);
            output = [x,S];        
        case 2
            Q = diag([.1 3*pi/180]).^2;
            x = [0 0 0]'; 
            S = diag([1 1 5*pi/180]).^2;
            z = sense(1);
            [x,S] = update_step(map,z,x,S,Q);
            output = [x,S]
    end    
end
      
% This function takes:
%     the mean, and a covariance matrix of the robot pose 
%     as well as the odometry information (d the distance travelled from time step k-1 and k and dth, the change of heading) 
%     and the matrix R (the covariance of the odometry noise). 
% The function performs a prediction step of the EKF localiser and returns the mean and covariance of the robot.      
function [xt,S] = predict_step(xt,S,d,dth,R)
    theta = wrapToPi(xt(3));
    xt = xt + [d*cos(theta);
               d*sin(theta);
               dth];
    xt(3) = wrapToPi(xt(3));

    Jxt = [1 0 -d*sin(xt(3));
           0 1 d*cos(xt(3));
           0 0 1];
    Jut = [cos(xt(3)) 0;
           sin(xt(3)) 0;
           0          1];
    
    S = Jxt * S * Jxt' + Jut * R * Jut';
    
end
   
% This function takes:
%     The location of all the landmarks (map)
%     The sensor readings of the range and bearing to all the landmarks in the map at the current time step.
%     The mean, and a covariance matrix of the robot pose
%     and the matrix Q (the covariance of the sensor noise). 
%
% The function performs an update step of the EKF localiser and returns the mean and covariance of the robot. 
function [x,S] = update_step(map,z,x,S,Q)
    for i = 1:size(map,1)
        xl = map(i,1);
        yl = map(i,2);
        xr = x(1);
        yr = x(2);        
        x(3) = wrapToPi(x(3));
        r  = sqrt((xr - xl)^2 + (yr - yl)^2);
        beta = wrapToPi(atan2(yl - yr, xl - xr) - x(3));
        
        G = [-((xl-xr)/r), -((yl-yr)/r) 0;
             ((yl-yr)/r^2),  -((xl-xr)/r^2) -1];
        
        K = S*G' *(G*S*G' + Q)^-1;


        x = x + K*(z(i,:)' - [r; beta]);
        x(3) = wrapToPi(x(3));
        S = (eye(3) - K*G)*S;
        
        
    end
    
    
end    

% ----------------------------
% write the extra functions that you need and call them in the above two functions





function plot_cov(x,P,nSigma)
    P = P(1:2,1:2); 
    x = x(1:2);
    if(~any(diag(P)==0))
        [V,D] = eig(P);
        y = nSigma*[cos(0:0.1:2*pi);sin(0:0.1:2*pi)];
        el = V*sqrtm(D)*y;
        el = [el el(:,1)]+repmat(x,1,size(el,2)+1);
        line(el(1,:),el(2,:));
    end;
end


