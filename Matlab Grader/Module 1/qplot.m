function pastPoints = qplot(q)
    x = q(1);
    y = q(2);
    theta = q(3);
    
    
    hold on

    drawTriangle(x,y,rad2deg(theta));
    set(gca,'Color','g')
    axis([0,2,0,2]);
    
    hold off
end

