clear;
clc;
close all


r=0.5; r_x=[0 0]; r_v=[0.1,0.1]; vmax=1.5; vmin=0.7;
g=0.5; g_x=[15,15];


o(1)=0.4; o_x(1,:)=[10,10]; o_v(1,:)=[-0.6,-0.6];
o_r_x(1,:)=o_x(1,:)-r_x;
o_r_v(1,:)=o_v(1,:)-r_v;

samples=50;
v=norm(r_v);
theta_dot=0;
v_desired=vmax*(g_x-r_x)/norm(g_x-r_x);
Aeq= [];
beq= [];
g_r_x=g_x-r_x;
A=  [];
b=  [];
detected_obs_rel_x=[];
detected_obs_rel_v=[];
detected_obs_size=[];
lambda=0.1;
toggle=1;
time=0.1;
ub_theta=0.5*pi/8;
lb_theta=-0.5*pi/8;
a1=axes('Units','inches');
axes(a1);
circ(0,0,1);
no_o= length(o);
range=7.0;
detected_obj=0;
theta=atan2(r_v(2),r_v(1));
theta_goal=atan2(v_desired(2),v_desired(1));
v=norm(r_v);
lb_s=0;
ub_s=vmax/v;
s0=1;
x_traj=[];
y_traj=[];
%objective = @(u) norm(v_desired-(u+r_v))^2 + lambda*norm(u)^2;
%nlei= @(u)StaticFunction(u,o_r_x, o_r_v, r, o);
%x=fmincon(objective, u0, A,b,Aeq, beq,lb,ub,nlei)
%m= @(u, i) (r+o(i))^2-sum(o_r_x(i,:).*o_r_x(i,:)) + ((o_r_x(i,:)*(o_r_v(i,:)-u)')^2)/norm((o_r_v(i,:)-u))^2;
%pearsrnd(mu_x,std_x,skew_x,krt_x)

while toggle==1
    options = optimoptions('fmincon');
    
    objective = @(theta_dot) norm(theta_goal-(theta + theta_dot*time)) + lambda*(theta_dot^2);
    nlei= @(theta_dot)MPC_objective_1(theta_dot, time, detected_obs_rel_x, detected_obs_rel_v, r, detected_obs_size, detected_obj,v, theta);
    theta_dot=fmincon(objective, theta_dot, A, b, Aeq, beq, lb_theta, ub_theta, nlei, options);
    
    objective1=@(s)norm(s*v*[cos(theta + theta_dot*time) sin(theta + theta_dot*time)]-v_desired);
    nlei1=@(s)MPC_objective_2(s,detected_obs_rel_x, detected_obs_rel_v, r, detected_obs_size, detected_obj, v, theta, theta + theta_dot*time);
    s=fmincon(objective1, s0, A, b, Aeq, beq, lb_s, ub_s, nlei1, options);
    
    theta=theta + theta_dot*time
    v=s*v
    detected_obj=0;
    detected_obs_rel_x=[];
    detected_obs_rel_v=[];
    detected_obs_size=[];
    ub_s=vmax/v;
    lb_s=vmin/v;
    r_v=v*[cos(theta) sin(theta)];
    r_x=r_x+(r_v*time);
    v_desired=vmax*(g_x-r_x)/norm(g_x-r_x);
    theta_goal=atan2(v_desired(2),v_desired(1));
    x_traj=[x_traj r_x(1)];
    y_traj=[y_traj r_x(2)];
    
    circ(r_x(1), r_x(2),r);
    hold on
    plot(x_traj, y_traj, 'color', [0,0,0]);
    hold on
    circ(g_x(1), g_x(2),g);
    hold on
    quiver(r_x(1),r_x(2),r_v(1), r_v(2));
    hold on
 for i=1:no_o   
    old(i,:)=o_r_x(i,:);
    o_x(i,:)=o_x(i,:)+(o_v(i,:)*time);
    o_r_x(i,:)=o_x(i,:)-r_x;
    o_r_v(i,:)=(o_r_x(i,:)-old(i,:))/time;
    if(sensor_range(r_x,o_x(i,:),range) && dot(o_r_x(i,:),o_r_v(i,:))<0 )
         detected_obj=detected_obj+1;
         detected_obs_rel_x(detected_obj,:)= o_r_x(i,:);
         detected_obs_rel_v(detected_obj,:)= o_r_v(i,:);
         detected_obs_size(detected_obj)= o(i);         
    end
    quiver(o_x(i,1),o_x(i, 2),o_v(i, 1), o_v(i, 2));
    hold on

    circ(o_x(i,1), o_x(i,2),o(i));
    hold on
 end    
    detected_obj

    xlim([0 15])
    ylim([0 15])    
    drawnow;
    hold off
    if norm(g_x-r_x)<1
        toggle=0;
    end
    %pause
  
end