function [xt,S] = predict_step(xt,S,dq,R)
    %@param:
    %       xt      : State Vector
    %       S       : The Covariance matrix
    %       d       : encoder distance
    %       dth     : encoder: Change in Heading
    %       R       : Encoder Noise
    %@return:
    %       xt      : State Vector
    %       S       : The Covariance matrix
%     if nargin < 4
%         R = diag([0.5 50*pi/180]).^2;
%     end
    xt = xt + dq;
    xt(3) = wrapToPi(xt(3));


    Jxt = [1 0 dq(2);
           0 1 dq(1);
           0 0 1];
    Jut = [cos(xt(3)) 0;
           sin(xt(3)) 0;
           0          1];
    
    S = Jxt * S * Jxt' + Jut * R * Jut';
    
    
    
end