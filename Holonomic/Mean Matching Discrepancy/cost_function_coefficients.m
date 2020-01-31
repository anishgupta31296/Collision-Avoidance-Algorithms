function [coeff_obs,hh,hf,ff]=cost_function_coefficients(h_list, reduced_sample, reduced_sample1, alpha, desired_dist,desired_dist1,beta,u1, d)
no_o=size(reduced_sample,3);
for x=1:no_o
 for i=1:6     
     for j=1:6
        hh(i, j)= alpha(:,x)'*generate_kernel_matrix(h_list{i}(reduced_sample(:,1,x),reduced_sample(:,2,x), reduced_sample1(:,1,x), reduced_sample1(:,2,x),0.4, 0, 0),h_list{j}(reduced_sample(:,1,x),reduced_sample(:,2,x), reduced_sample1(:,1,x), reduced_sample1(:,2,x),0.4, 0, 0),d)*alpha(:,x);

     end
 end
 for i=1:6
     hf(i)= alpha(:,x)'*generate_kernel_matrix(h_list{i}(reduced_sample(:,1,x),reduced_sample(:,2,x), reduced_sample1(:,1,x), reduced_sample1(:,2,x),0.4, 0, 0), h_list{7}(desired_dist(:,1,x),desired_dist(:,2,x), desired_dist1(:,1,x), desired_dist1(:,2,x),0.4, 0, 0, u1(1), u1(2)), d)*beta(:,x);
 end
 ff=beta(:,x)'*generate_kernel_matrix(h_list{7}(desired_dist(:,1,x),desired_dist(:,2,x), desired_dist1(:,1,x), desired_dist1(:,2,x),0.4, 0, 0, u1(1), u1(2)), h_list{7}(desired_dist(:,1,x),desired_dist(:,2,x), desired_dist1(:,1,x), desired_dist1(:,2,x),0.4, 0, 0, u1(1), u1(2)),d)*beta(:,x);
 
 ff=0;
 hf=zeros(6,1);
 
 coeff(1)=hh(5,5);
 coeff(2)=hh(4,4);
 coeff(3)=hh(5,3) + hh(3,5);
 coeff(4)=hh(4,2) + hh(2,4);
 coeff(5)=hh(3,3) + hh(6,5) + hh(5,6) - 2*hf(5);
 coeff(6)=hh(2,2) + hh(6,4) + hh(4,6) - 2*hf(4);
 coeff(7)=hh(6,3) + hh(3,6) - 2*hf(3);
 coeff(8)=hh(6,2) + hh(2,6) - 2*hf(2);
 coeff(9)=hh(6,6) - 2*hf(6) + ff; 
 coeff(10)=hh(5,4) + hh(4,5) + hh(1,1);
 coeff(11)=hh(5,1) + hh(1,5);
 coeff(12)=hh(4,1) + hh(1,4);
 coeff(13)=hh(5,2) + hh(2,5) + hh(3,1) + hh(1,3);
 coeff(14)=hh(4,3) + hh(3,4) + hh(2,1) + hh(1,2);
 coeff(15)=hh(3,2) + hh(2,3) + hh(6,1) + hh(1,6) - 2*hf(1);
 coeff_obs(x,:)=coeff;
end

end