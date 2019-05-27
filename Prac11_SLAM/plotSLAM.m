function plotSLAM(mu,S)
    q = mu(1:3);
    Sr = S (1:3,1:3);
    mu = mu(4:end);
    S = S(:,4:end);
    
    X = mu(1:2:end);
    Y = mu(2:2:end);
    
    
    %% Plot
%   clf
    hold on
    plot_cov(q',Sr,3)
    hold on
    drawFrame(q)
    for i = 1:length(X)
        Sl = S(2*(i-1) +1: 2*(i-1) +2,2*(i-1) +1: 2*(i-1) +2);
        hold on
        plot(X(i),Y(i))
        plot_cov([X(i);Y(i)],Sl,3)
        drawnow
    end
    axis equal
    drawnow
end

