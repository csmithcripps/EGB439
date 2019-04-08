function pastPoints = qplot(x,y,theta,pastPoints)
    
    
    
    hold on

    drawTriangle(x,y,theta);
    axis([0,2,0,2]);
    pastPoints(1,:) = [];
    pastPoints = [pastPoints; x y];
    plot(pastPoints(:,1), pastPoints(:,2), 'b');
    
    hold off
end

