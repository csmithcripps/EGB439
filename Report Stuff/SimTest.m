function [sumError] = SimTest(R,Q, nsteps)    
% this simulator runs for 100 steps
    load('reportData.mat')
    
    
    startStep = 4;
    idMap = [];
    qSlam = [];

    mu =   [q(startStep,1);q(startStep,2);q(startStep,3)];
    S =diag([0.1 0.1 0.1*pi/180]).^2;
    % main loop
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
    end

    hold on
    plotSLAM(mu,S);
    hold on
    GTruth = plot(q(startStep:nsteps+startStep,1),q(startStep:nsteps+startStep,2),'r--');
    hold on
    SLAMOut = plot(qSlam(:,1),qSlam(:,2),'m--');
    legend([GTruth,SLAMOut],'Ground Truth', 'SLAM');

    sumError = [0 0];
    for i = 2:length(qSlam)
        sumError(1) = sumError(1) + abs(q(i,1) - qSlam(i,1)/q(i,1));
        sumError(2) = sumError(2) + abs(q(i,2) - qSlam(i,1)/q(i,2));
    end
    error = (100/length(q)) * sumError;
end
