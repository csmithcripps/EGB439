function qplot(q)

    % Setup figure
    clf
    
    % Display triangle
    tri = [0 0 1; -0.15 -0.05 1; -0.15 0.05 1; 0 0 1]';
    x = q(1);
    y = q(2);
    ang = q(3);
    H = [cos(ang) -sin(ang) x; sin(ang) cos(ang) y; 0 0 1];
    Q = H*tri;
    plot(Q(1,:), Q(2,:), 'k-')
    
    % Draw
    xlim([0 2])
    ylim([0 2])
    %set(gca,'color','g');
    drawnow

end