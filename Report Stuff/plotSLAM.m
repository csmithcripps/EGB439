function plotSLAM(mu,S)
    nBeacons = (length(mu)-3)/2;
    grid on; axis equal;
%     axis([0 3 -0.5 2.5]);
    hold on
    drawFrame(mu(1:3))  
    hold on
    plot_cov(mu,S,3,'b')
    for i=1:nBeacons
        lidx   = 3 + i*2 -1;
        lSigma = S(lidx:lidx+1,lidx:lidx+1);
        hold on
        plot_cov(mu(lidx:lidx+1),lSigma,3,'g');   
        hold on
        scatter(mu(lidx),mu(lidx+1),50,'b+');   
    end
end

