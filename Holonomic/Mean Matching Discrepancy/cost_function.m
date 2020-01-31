function cost_func=cost_function(coeff, objective)
x=@(u)[u(1)^4, u(2)^4, u(1)^3, u(2)^3, u(1)^2, u(2)^2, u(1), u(2), 1, (u(1)^2)*(u(2)^2), (u(1)^3)*(u(2)), (u(1))*(u(2)^3), (u(1)^2)*(u(2)), (u(1))*(u(2)^2), u(1)*u(2)];
cost_func=@(u) sum(coeff*(x(u)')) ;
end