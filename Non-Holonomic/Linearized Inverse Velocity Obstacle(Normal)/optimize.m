function opt=optimize(head, vr, wr,  xr, yr, xob, yob, xobdot, yobdot,R,dt, lb, ub, objective)
relpx=xr-xob;
relpy=yr-yob;
relvx=vr*cos(head) - xobdot;
relvy=vr*sin(head) - yobdot;
robovx=vr*cos(head);
robovy=vr*sin(head);
t0=NonLinearConstraint([0,0], head, vr, wr,relpx,relpy,relvx,relvy,robovx,robovy ,R,dt)
t1=term1([0,0], head, vr, wr,relpx,relpy,relvx,relvy,robovx,robovy ,R,dt);
t2=term2([0,0], head, vr, wr,relpx,relpy,relvx,relvy,robovx,robovy ,R,dt);
nonlin=@(u) nlcon(u,t0,t1,t2);

%     function [c ceq]=ineq(u)
%      c=NonLinearConstraint(u, head, vr, wr,relpx,relpy,relvx,relvy,robovx,robovy ,R,dt);
%      ceq=[];
%     end
% nonlin=@(u) ineq(u);
opt=fmincon(objective, [0,0], [], [], [], [], lb, ub,nonlin);
end