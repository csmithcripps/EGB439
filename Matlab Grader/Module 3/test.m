measur_prob = [0.50,0.30];
signs_map = [1 , 0 , 0 , 1 , 0 , 1 , 0 , 0 , 0 , 0];
prior_belief   =  ones(1,length(signs_map) ) ./ length(signs_map);
z = 0;

%%
posterior_belief = discrete_localise1d(prior_belief,signs_map,measur_prob,z);

assert(length(posterior_belief) == length(prior_belief),"The length of the posterior distribution should be the same as the prior distribution")
assert(isalmostequal(sum(posterior_belief), 1) , "The posterior distribution should sum to one.")