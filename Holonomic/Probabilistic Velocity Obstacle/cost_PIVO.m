function [c,ceq]=cost_PIVO(u,mean_terms,variance_terms, detected_objects,k)
u1=[u(1)^4, u(2)^4, u(1)^3, u(2)^3, u(1)^2, u(2)^2, u(1), u(2), 1, (u(1)^2)*(u(2)^2), (u(1)^3)*(u(2)), (u(1))*(u(2)^3), (u(1)^2)*(u(2)), (u(1))*(u(2)^2), u(1)*u(2)];
u2=[u(1)*u(2) u(2) u(1) u(2)^2 u(1)^2 1];
m=reshape(mean_terms, 6,detected_objects);
v=reshape(variance_terms,15,detected_objects);
c= (u2*m + k*sqrt(abs(u1*v)));
ceq=[];
end