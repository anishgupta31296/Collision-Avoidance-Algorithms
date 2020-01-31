function [c, ceq]=nlcon(u,t0,t1,t2)
 ceq=[];
 c= t0 + u(1)*t1 + u(2)*t2;
end