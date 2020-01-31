function [kernel_matrix]=generate_kernel_matrix(sample, sample1,d)
length1=size(sample,1);
length2=size(sample1,1);
[x,y]=ndgrid(1:length1, 1:length2);
kernel_matrix=reshape(kernel(sample(x,:), sample1(y,:), d),length1, length2);
end