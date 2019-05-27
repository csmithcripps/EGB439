function [mu, Sigma] = initLandmarks(z,Q,mu,Sigma)
    %@param:
    %       z       : The sensor measurements to a landmark at the current time step.
    %       Q       : The covariance of the measurements
    %       mu      : state vector (Robot pose and all landmarks)
    %@return:
    %       mu      : state vector
    %       Sigma   : The Covariance matrix

    % Init
    xR = mu(1);     %Robot x (m)
    yR = mu(2);     %Robot y (m)
    thetaR = mu(3); %Robot angle (radians)
    
    %Sensor
    r = z(1);
    beta = z(2);

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