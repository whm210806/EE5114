function [particles] = slam_resample(particles, init_weight)
	
	particles_count = size(particles, 2);
	weight_total = zeros(particles_count+1,1);
	
    for i = 2:particles_count+1
        % weight_total(i) denotes the sum of weights of particles with index less than i
		weight_total(i) = weight_total(i-1) + particles(i-1).weight; 
    end
    
	for i = 1:particles_count
        W=weight_total(end,1)*rand(1);
        a=find(weight_total(2:particles_count+1)>W);
        k=a(1);
        particles1(i)=particles(k);
        particles1(i).weight=init_weight;
        % Missing codes start here
        
        % Resamples particles based on their weights
        
        % Afterwards, each new partical should be given the same init_weight
        
        % Missing codes end here
    end
    particles=particles1;
end