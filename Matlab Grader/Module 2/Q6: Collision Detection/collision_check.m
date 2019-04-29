function collide = collision_check(q)
    % q is a 1x3 vector containing the robot configuration with elements x, y and theta (in degrees)
    % collide is true if the robot intersects any of the obstacles, otherwise it is false
    theta = q(3);
    x = q(1);
    y = q(2);
    
    T = [cosd(theta) -sind(theta) x;
         sind(theta)  cosd(theta) y;
         0            0           1];
     
    robCoor = [T* [-0.75 -1.5 1;
              -0.75  1.5 1;
               0.75  1.5 1;
               0.75 -1.5 1]']';
    rob = polyshape(robCoor(:,1:2));
    
    obsX = [ 2,3,3,2;
             5,6,6,5;
             5,6,6,5];
    obsY = [ 7,7,5,5;
             10,10,6,6;
             4,4,0,0];
    figure
    obs = [polyshape(obsX(1,:),obsY(1,:));
           polyshape(obsX(2,:),obsY(2,:));
           polyshape(obsX(3,:),obsY(3,:))];
    
    inter = intersect(rob,obs);
       
    plot(obs,'FaceColor','black')
    hold on
    plot(rob,'FaceColor','blue')
    hold on
    plot(q(1),q(2),'rp')
    plot(inter,'FaceColor','red', 'FaceAlpha',1)
    xlim([0,10]);
    ylim([0,10]);
    axis 'square'
    grid 'on'
         
    collide = sum(area(inter))>0;
end