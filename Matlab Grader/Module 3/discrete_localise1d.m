function posterior_belief = discrete_localise1d(prior_belief,signs_map,measur_prob,z)
% prior_belief: The prior belief of the robot about its location before incorporating any sensor information
% signs_map : the map of where the signs are in the hallway.
% measurements probabilities = [p(z=1|sign) , p(z=1|no_sign)]
% z is the detector output. 1 sign, 0 no_sign.
    if z
        PZgivenC = signs_map;
        PZgivenC(PZgivenC==1) = measur_prob(1);
        PZgivenC(PZgivenC==0) = measur_prob(2);
        
        PZ = sum(PZgivenC .* prior_belief);
        
        posterior_belief = (PZgivenC .* prior_belief)/PZ;
        
    else
        PZgivenC = signs_map;
        PZgivenC(PZgivenC==1) = 1 - measur_prob(1);
        PZgivenC(PZgivenC==0) = 1 - measur_prob(2);
        
        PZ = sum(PZgivenC .* prior_belief);
        
        posterior_belief = (PZgivenC .* prior_belief)/PZ;
    end


end