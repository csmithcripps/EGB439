function [error] = SimTest(R,Q, nsteps,plotOut)    
% this simulator runs for 100 steps
    load('reportData.mat')
    
    if nargin < 4
        plotOut = 1;
    end
    startStep = 4;
    idMap = [];
    qSlam = [];

    mu =   [q(startStep,1);q(startStep,2);q(startStep,3)];
    S =diag([0.1 0.1 0.1*pi/180]).^2;
    % main loop
    error = 0;
    for k = startStep:nsteps + startStep
       qSlam = [qSlam;mu(1:3)'];
       
       
       dTicks = o(k,:);
       [d,dth]  = encoderToPose(dTicks,mu(1:3));    
       % complete the prediction step in the body of the function below
       [mu,S] = predict_step(mu,S,d,dth,R);

        zStep = z(:,:,k);    
        for j = 1:5
            if zStep(j,1) == 0
                continue
            end
            if sum(idMap == zStep(j,1))
                rb = zStep(j,2:3);
                r = rb(1);
                b = rb(2);
                idNumber = find(idMap == zStep(j,1));
                [mu,S] = update_step(idNumber,[r;b],mu,S,Q);
            else
                rb = zStep(j,2:3);
                r = rb(1);
                b = rb(2);
                [mu,S] = initLandmarks([r;b],Q,mu,S);
                idMap = [idMap; zStep(j,1)];
            end
        end
    %     hold on
    %     plotSLAM(mu,S);
    %     hold on
    %     scatter(RealBeaconPos(:,1),RealBeaconPos(:,2),'r+')
    %     hold on
    %     plot(q(k,1),q(k,2),'g*')
    
        error = error + sqrt((q(startStep +length(qSlam),1) - qSlam(end,1))^2 +...
                             (q(startStep +length(qSlam),2) - qSlam(end,2))^2);
    end
    
    
    qSlam = [qSlam;mu(1:3)'];
    
    if plotOut
        hold on
        plotSLAM(mu,S);
        hold on
        GTruth = plot(q(startStep:startStep+length(qSlam),1),q(startStep:startStep+length(qSlam),2),'r--');
        hold on
        SLAMOut = plot(qSlam(:,1),qSlam(:,2),'m--');
%         legend([GTruth,SLAMOut],'Ground Truth', 'SLAM');
    end
end