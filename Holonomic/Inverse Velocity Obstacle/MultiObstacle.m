clear;
clc;
close all

r=0.5; r_x=[0,0]; r_v=[1.5,0.0]; vmax=  1.5;
g=0.5; g_x=[15,0];

o(1)=0.5; o_x(1,:)=[12,0]; o_v(1,:)=[-0.5, 0.0];
o_r_x(1,:)=o_x(1,:)-r_x;
o_r_v(1,:)=o_v(1,:)-r_v;
% o(2)=0.5; o_x(2,:)=[12,-1]; o_v(2,:)=[-0.5, 0.0];
% o_r_x(2,:)=o_x(2,:)-r_x;
% o_r_v(2,:)=o_v(2,:)-r_v;

samples=50;

u0=[0,0];
v_desired=vmax*(g_x-r_x)/norm(g_x-r_x);
Aeq= [];
beq= [];
g_r_x=g_x-r_x;
A=  [];
b=  [];
detected_obs_rel_x=[];
detected_obs_rel_v=[];
detected_obs_size=[];
lambda=0.2;
toggle=1;
time=0.1;
ub=vmax*ones([1,2]);
lb=-vmax*ones([1,2]);
a1=axes('Units','inches');
axes(a1);
circ(0,0,1);
no_o= length(o);
range=8.0;
detected_obj=0;
theta=atan2(r_v(2),r_v(1));
theta_goal=atan2(v_desired(2),v_desired(1));
theta_dot=0.1;
v=norm(r_v);
f=@(v,w,dt,r1, r2,r,R) (v.^2).*(r1.^2) + 2.*dt.*(v.^2).*w.*r1.*r2 - (v.^2).*(-(r+R)^2 + (r1.^2) + (r2.^2));
%objective = @(u) norm(v_desired-(u+r_v))^2 + lambda*norm(u)^2;
%nlei= @(u)StaticFunction(u,o_r_x, o_r_v, r, o);
%x=fmincon(objective, u0, A,b,Aeq, beq,lb,ub,nlei)
%m= @(u, i) (r+o(i))^2-sum(o_r_x(i,:).*o_r_x(i,:)) + ((o_r_x(i,:)*(o_r_v(i,:)-u)')^2)/norm((o_r_v(i,:)-u))^2;
%pearsrnd(mu_x,std_x,skew_x,krt_x)

while toggle==1
    
    objective = @(u1) norm(v_desired-(u1+r_v)) + 0*norm(u1);
    nlei= @(u1)multiobs(u1,detected_obs_rel_x, detected_obs_rel_v, r, detected_obs_size, detected_obj);
       obj=[];
       count_i=0;
       count_j=0;
       for del=lb(2):0.01:ub(2)
        count_i=count_i+1;
        for del_v=lb(1):0.01:ub(1)
         count_j=count_j+1;
         obj(count_i,count_j)=objective([del_v,del]);
        end
        count_j=0;
       end
       mesh1=lb(2):0.01:ub(2);
       mesh2=lb(1):0.01:ub(1);
       [grid1,grid2]=ndgrid(1:length(mesh1),1:length(mesh2));
       mesh1=mesh1(grid1);
       mesh2=mesh2(grid2);
       surf(mesh2, mesh1, obj)

    options = optimoptions('fmincon');
    u=fmincon(objective, u0, A, b, Aeq, beq, lb, ub, nlei, options)
    if(detected_obj)
     pause
    end    
    detected_obj=0;
    detected_obs_rel_x=[];
    detected_obs_rel_v=[];
    detected_obs_size=[];
       
    r_v=r_v+u;
    r_x=r_x+(r_v*time);
    v_desired=vmax*(g_x-r_x)/norm(g_x-r_x);
    %u0=[0.00,0.00];
    ub=vmax*ones([1,2]) - r_v;
    lb=-vmax*ones([1,2])-r_v;
    
    circ(r_x(1), r_x(2),r);
    hold on
    circ(g_x(1), g_x(2),g);
    hold on

 for i=1:no_o   
    old(i,:)=o_r_x(i,:);
    o_x(i,:)=o_x(i,:)+(o_v(i,:)*time);
    o_r_x(i,:)=o_x(i,:)-r_x;
    o_r_v(i,:)=(o_r_x(i,:)-old(i,:))/time;
    if(sensor_range(r_x,o_x(i,:),range) && (dot(o_r_x(i,:),o_r_v(i,:))<0 || norm(r_x-o_x(i,:))<(r+o(i)+0.25)))
         detected_obj=detected_obj+1;
         detected_obs_rel_x(detected_obj,:)= o_r_x(i,:);
         detected_obs_rel_v(detected_obj,:)= o_r_v(i,:);
         detected_obs_size(detected_obj)= o(i);  
         tangent(r_x, o_x(i,:),r+o(i), r);
         hold on;
    end
    quiver(o_x(i,1),o_x(i, 2),o_v(i, 1), o_v(i, 2));
    hold on
    circ(o_x(i,1), o_x(i,2),o(i));
    hold on    
 end    
    quiver(r_x(1),r_x(2),r_v(1), r_v(2));
    hold on   
    xlim([-1 16])
    ylim([-8.5 8.5])    
    drawnow;
    hold off
    
    if norm(g_x-r_x)<1
        toggle=0;
    end

end

