clear;
clc;
close all

r=0.5; r_x=[0,0]; r_v=[1.5,0.0]; vmax=  1.5;
g=0.5; g_x=[12,12];

o(1)=0.5; o_x(1,:)=[11,11]; o_v(1,:)=[-0.8, -0.8];
o(2)=0.5; o_x(2,:)=[15,11]; o_v(2,:)=[-0.95, -0.65];
o(3)=0.5; o_x(3,:)=[11,15]; o_v(3,:)=[-0.65, -0.95];


x_min=min([o_x(:,1); r_x(1); g_x(1)]);
x_max=max([o_x(:,1); r_x(1); g_x(1)]);
y_min=min([o_x(:,2); r_x(2); g_x(2)]);
y_max=max([o_x(:,2); r_x(2); g_x(2)]);
range=max([(x_max-x_min) (y_max-y_min)]);
x_axis_min=(x_min + x_max)/2  - range/2 -1;
x_axis_max=(x_min + x_max)/2  + range/2 +1;
y_axis_min=(y_min + y_max)/2  - range/2 -1;
y_axis_max=(y_min + y_max)/2  + range/2 +1;


samples=50;

u0=[0,0];
v_desired=vmax*(g_x-r_x)/norm(g_x-r_x);
r_v=v_desired;
theta=atan2(r_v(2),r_v(1));
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
no_o= length(o);
range=6.0;
detected_obj=0;
v=norm(r_v);
vel_cap_max=1.5;
vel_cap_min=-1.5;
for i=1:size(o_x,1)
 o_r_x(i,:)=o_x(i,:)-r_x;
 o_r_v(i,:)=o_v(i,:)-r_v;
end

while toggle==1
    theta=atan2(r_v(2),r_v(1));
    Transform=[cos(theta) sin(theta); -sin(theta) cos(theta)];
    for t=1:size(detected_obs_rel_x,1)
        detected_obs_rel_x(t,:)=(Transform*detected_obs_rel_x(t,:)')';
        detected_obs_rel_v(t,:)=(Transform*detected_obs_rel_v(t,:)')';        
    end
    v_desired=(Transform*v_desired')';
    vel=(Transform*r_v')';
    objective = @(u1) norm(v_desired-(u1+vel)) + 0*norm(u1);
    nlei= @(u1)multiobs(u1,detected_obs_rel_x, detected_obs_rel_v, r, detected_obs_size, detected_obj);
    options = optimoptions('fmincon');
    u=fmincon(objective, u0, A, b, Aeq, beq, lb, ub, nlei, options);
    Transform_back=[cos(theta) -sin(theta); sin(theta) cos(theta)];
    u=(Transform_back*u')';

    if(detected_obj)
        (Transform*(r_v)')'
        pause
    end
    detected_obj=0;
    detected_obs_rel_x=[];
    detected_obs_rel_v=[];
    detected_obs_size=[];
       
    r_v=r_v+u;
%     if r_v(1)>vel_cap_max
%         r_v(1) = vel_cap_max;
%     end
%     if r_v(2)>vel_cap_max
%         r_v(2) = vel_cap_max;
%     end
%     if r_v(1)<vel_cap_min
%         r_v(1) = vel_cap_min;
%     end
%     if r_v(2)<-vel_cap_min
%         r_v(2) = -vel_cap_min;
%     end      
    thata=atan2(r_v(2),r_v(1));
    v_desired=(vmax*(g_x-r_x)/norm(g_x-r_x));    
    r_x=r_x+(r_v*time);
    %u0=[0.00,0.00];
%     ub=vmax*ones([1,2]) - r_v;
%     lb=-vmax*ones([1,2])-r_v;
    
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
   axis([x_axis_min x_axis_max y_axis_min y_axis_max])

    drawnow;
    hold off
    
    if norm(g_x-r_x)<1
        toggle=0;
    end

end

