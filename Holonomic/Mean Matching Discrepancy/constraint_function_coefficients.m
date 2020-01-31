function [h, f]= constraint_function_coefficients(o_r_x, o_r_v, r, o)
h5=@(i,j,u)((r+o(i))^2 - sum(o_r_x(j,:,i).*o_r_x(j,:,i)) + (o_r_x(j,1,i)^2));
h4=@(i,j,u)((r+o(i))^2 - sum(o_r_x(j,:,i).*o_r_x(j,:,i)) + (o_r_x(j,2,i)^2));
h3=@(i,j,u)(-2*o_r_v(j,1,i)*((r+o(i))^2 - sum(o_r_x(j,:,i).*o_r_x(j,:,i))) -2*(o_r_x(j,:,i)*o_r_v(j,:,i)'* o_r_x(j,1,i)) );
h2=@(i,j,u)(-2*o_r_v(j,2,i)*((r+o(i))^2 - sum(o_r_x(j,:,i).*o_r_x(j,:,i))) -2*(o_r_x(j,:,i)*o_r_v(j,:,i)'* o_r_x(j,2,i)) );
h1=@(i,j,u)(2*o_r_x(j,1,i)*o_r_x(j,2,i));
h0=@(i,j,u)((r+o(i))^2)*(norm(o_r_v(j,:,i))^2) - (norm(o_r_v(j,:,i))^2)*(norm(o_r_x(j,:,i))^2) + (o_r_x(j,:,i)*(o_r_v(j,:,i))')^2;
f=@(i,j,u)(h5(i,j)*u(1)^2 + h4(i,j)*u(2)^2 + h3(i,j)*u(1) + h2(i,j)*u(2) + h1(i,j)*u(1)*u(2) + h0(i,j));
h={h1, h2, h3, h4, h5, h0};
end