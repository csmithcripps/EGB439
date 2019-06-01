function plotSLAM(mu,S,idMap)
    clf
    grid on; axis equal;
    axis([-2 3 -2 3]);
    hold on
    drawFrame(mu(1:3))  
    hold on
    plot_cov(mu,S,3,'b')
    for i=1:length(idMap)
        lidx   = 3 + i*2 -1;
        lSigma = S(lidx:lidx+1,lidx:lidx+1);
        hold on
        plot_cov(mu(lidx:lidx+1),lSigma,3,'g');   
        hold on
        scatter(mu(lidx),mu(lidx+1),50,'b+');   
    end
    drawnow
end

