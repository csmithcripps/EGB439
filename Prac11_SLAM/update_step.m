function [mu, Sigma] = update_step(i,zi,mu,Sigma,Q)
    %@param:
    %           i           : the ith landmark 
    %           zi          : the range and bearing to landmark i
    %           Q           : the covariance of the measurements
    %           mu,Sigma    : the current estimate of the map.
    %@return:
    %           mu          : The Average State Vector for the Landmarks
    %           Sigma       : The Covariance matrix of all landmarks

    % Init
    xr = mu(1:3);
    xR = xr(1);     %Robot x (m)
    yR = xr(2);     %Robot y (m)
    thetaR = xr(3); %Robot angle (radians)
    xl = mu(2*(i-1) +1 +3);
    yl = mu(2*(i-1) +2 +3);
           %(2*(i-1)) = all previous landmarks
           % + 3 at the end is for the robot state

    % Calculate the intermediate Variables
    

    r  = sqrt((xR - xl)^2 + (yR - yl)^2);
    beta = wrapToPi(atan2(yl - yR, xl - xR) - thetaR);
    h = [r;beta];

    G = zeros(2,length(mu));
    G__ = [ -((xl-xR)/r),   -((yl-yR)/r);
             ((yl-yR)/r^2), -((xl-xR)/r^2)];
         
    G(1:2,1:3) = [G__,[0;-1]];
    G(:,  2*(i-1) +1 +3  :  2*(i-1) +2 +3) = -G__;
    K = Sigma*G' * (G*Sigma*G' + Q)^-1;

    b = (zi' - h);
    b(2) = wrapToPi(b(2));

    %Propogate Updates
    mu = mu + K*b;
    Sigma = (eye(length(mu)) - K*G)*Sigma;

end
