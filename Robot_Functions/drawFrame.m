function drawFrame(q)
    x = q(1);
    y = q(2);
    theta= q(3);
    r = 0.1;
    
    cx_x = x + r*cos(theta);
    cx_y = y + r*sin(theta);
    cy_x = x + r*cos(theta+pi/2);
    cy_y = y + r*sin(theta+pi/2);
    
    hold on 
    plot(x,y,'r*')
    hold on
    plot([x cx_x], [y cx_y],'r-')
    hold on
    plot([x cy_x], [y cy_y],'b-')
    
    drawnow
end
    