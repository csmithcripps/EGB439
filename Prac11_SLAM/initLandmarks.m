function [mu, Sigma] = initLandmarks(z,Q,mu,Sigma)
    %@param:
    %       z       : The sensor measurements to all the landmarks at the current time step.
    %       Q       : The covariance of the measurements
    %       xr      : The true pose of the robot [x;y;theta].
    %@return:
    %       mu      : The Average State Vector for the Landmarks
    %       Sigma   : The Covariance matrix of all landmarks

    % Init
    xR = mu(1);     %Robot x (m)
    yR = mu(2);     %Robot y (m)
    thetaR = mu(3); %Robot angle (radians)



    % For each landmark
    for i = 1:size(z,1)
        %Sensor
        r = z(i,1);
        beta = z(i,2);

        %Calculate for Landmark_i
        theta_ = wrapToPi(thetaR + beta);
        lnew = [xR + r*cos(theta_);
                yR + r*sin(theta_)];
        L = [cos(theta_) -r*sin(theta_);
             sin(theta_)  r*cos(theta_)];

        % Append to existing values
        mu = [mu;lnew];
        Sigma = [Sigma      zeros(size(Sigma,1),2);
                 zeros(2,size(Sigma,2)) L*Q*L'];
    end
 
    
end