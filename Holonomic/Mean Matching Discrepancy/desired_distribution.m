function [desired_dist,desired_dist1,desired_coeff,u,nlei] = desired_distribution(samples, samples1, cost_function, lb, ub,r,o, d,desired_samples)
no_of_samples=desired_samples;
%[big_sample_size_row,big_sample_size_col, no_o]=size(samples);
%indices=randi(big_sample_size_row, 1 ,no_of_samples);
%reduced_samples=samples(indices,:,:);
%reduced_samples1=samples1(indices,:,:);
%r_coeff=ones(no_of_samples,1)/no_of_samples;
[reduced_samples,reduced_samples1, r_coeff]=reduced_set_method(samples, samples1, no_of_samples, d);
nlei= @(u)nlconMMD(u,reduced_samples, reduced_samples1, r, o);
options = optimoptions('fmincon', 'Display', 'off');
u=fmincon(cost_function, [0 0], [], [], [], [], lb, ub, nlei, options);
desired_dist=reduced_samples;
desired_dist1=reduced_samples1;
desired_coeff=r_coeff;
end
