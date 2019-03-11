function pastPoints = qplot(x,y,theta,pastPoints)

    pastPoints(1,:) = [];
    pastPoints = [pastPoints; x y];
    
    hold on

    drawTriangle(x,y,theta);
    axis([0,2,0,2]);
    
    plot(pastPoints(:,1), pastPoints(:,2), 'b');
    hold off
end

