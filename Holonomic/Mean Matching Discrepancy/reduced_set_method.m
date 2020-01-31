function [reduced_sample, reduced_sample1, reduced_set_coeff]=reduced_set_method(sample, sample1, target_size, d)
 [big_sample_size_row,big_sample_size_col, no_o]=size(sample);
indices=randi(big_sample_size_row, 1 ,target_size);
reduced_sample=sample(indices,:,:);
reduced_sample1=sample1(indices,:,:);
%indices=reduced_set_samples(sample,no_o,-100,target_size, d);
for x=1:no_o
%reduced_sample(:,:,x)=sample(indices(x,:),:,x);
%reduced_sample1(:,:,x)=sample1(indices(x,:),:,x);
kz=generate_kernel_matrix(reduced_sample(:,:,x),reduced_sample(:,:,x),d);
kzx=generate_kernel_matrix(reduced_sample(:,:,x), sample(:,:,x),d);
alpha=1*ones(big_sample_size_row,1);
reduced_set_coeff(:,x)=(pinv(kz)*kzx*alpha)./big_sample_size_row;
end

end
