function [x,y] = findCentroid(mu,S)
    mu = mu(4:end);
    S = S(:,4:end);
    
    X = mu(1:2:end);
    Y = mu(2:2:end);
    
    x = mean(X);
    y = mean(Y);
end