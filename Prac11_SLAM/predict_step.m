function [mu,Sigma] =predict_step(mu,Sigma,d,dth,R)
    %@param:
    %       mu      : The Average State Vector for the Landmarks
    %       Sigma   : The Covariance matrix of all landmarks
    %       d       : distance travelled from t = k-1 to t=k.
    %       Q       : change of heading from t = k-1 to t=k.
    %       xr      : The true pose of the robot [x;y;theta].
    %@return:
    %       mu      : The Average State Vector for the Landmarks
    %       Sigma   : The Covariance matrix of all landmarks

    twoN = length(mu) - 3;
    %Calculate Predicted Pose
    xt = mu(1:3,1);
    xt = xt + [d*cos(xt(3));
               d*sin(xt(3));
               dth];
    xt(3) = wrapToPi(xt(3));

    %Calculate Jacobians
    Jxrt = [1 0 -d*sin(xt(3));
            0 1 d*cos(xt(3));
            0 0 1];
    Jurt = [cos(xt(3)) 0;
            sin(xt(3)) 0;
            0          1];

    Jx = [Jxrt                  zeros(3,twoN);
          zeros(twoN,3)   eye(twoN)];
    Ju = [Jurt;
          zeros(twoN,2)];

    %Propogate Changes
    mu(1:3,1) = xt;
    Sigma = Jx * Sigma * Jx' + Ju * R * Ju';
    
end