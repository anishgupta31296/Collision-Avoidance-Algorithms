clear;
clc;
close all

k=1;

mu_x = 0;
std_x = 0.1;
skew_x = 0.0;
krt_x = 3.0;
    
mu_y = 0;
std_y = 0.1;
skew_y = 0.0;
krt_y = 3.0;

mu_ux = 0;
std_ux = 0.1;
skew_ux = 0.0;
krt_ux = 3.0;

mu_uy = 0;
std_uy = 0.1;
skew_uy = 0.0;
krt_uy = 3.0;

std_vx=0.15;
std_vy=0.15;

r=0.5; r_x=[0,0]; r_v=[1.0,1.0]; vmax=  1.0;
g=0.5; g_x=[15,15];

o(1)=0.4; o_x(1,:)=[7,7]; o_v(1,:)=[-0.7, -0.7];
o_r_x(1,:)=o_x(1,:)-r_x;
o_r_v(1,:)=o_v(1,:)-r_v;

% o(2)=0.4; o_x(2,:)=[6,8]; o_v(2,:)=[-0.7, -0.9];
% o_r_x(2,:)=o_x(2,:)-r_x;
% o_r_v(2,:)=o_v(2,:)-r_v;
% 
% o(3)=0.4; o_x(3,:)=[8,4]; o_v(3,:)=[-0.8, -0.6];
% o_r_x(3,:)=o_x(3,:)-r_x;
% o_r_v(3,:)=o_v(3,:)-r_v;


% o(1)=0.4; o_x(1,:)=[10,10]; o_v(1,:)=[-0.707, -0.707];
% o_r_x(1,:)=o_x(1,:)-r_x;
% o_r_v(1,:)=o_v(1,:)-r_v;

samples=1000;

u0=[0,0];
v_desired=vmax*(g_x-r_x)/norm(g_x-r_x);
Aeq= [];
beq= [];
g_r_x=g_x-r_x;
A= [];
b= [];
detected_obs_rel_x=[];
detected_obs_rel_v=[];
detected_obs_size=[];
lambda=0;
toggle=1;
time=0.1;
ub=vmax*ones([1,2]);
lb=-vmax*ones([1,2]);
a1=axes('Units','inches');
axes(a1);
circ(0,0,1);
no_o= length(o);
range=7;
detected_obj=0;
h1=function_handles(r,1);
du_x=[];
du_y=[];
f_val=[];
[iter1,iter2]=ndgrid(1:samples);


while toggle==1
    
    objective = @(u) norm(v_desired-(u+r_v)) + lambda*norm(u);
    nlei= @(u)multiobs(u,detected_obs_rel_x, detected_obs_rel_v, r, detected_obs_size, detected_obj);

    m=[];
    v=[];
    m1=[];
    v1=[];
    if(detected_obj)
     m1=mean_vector(detected_obs_rel_x(iter1,1,:), detected_obs_rel_x(iter1,2,:), detected_obs_rel_v(iter1,1,:), detected_obs_rel_v(iter1,2,:), detected_obs_size,du_x(iter2,:,:), du_y(iter2,:,:), h1);
     v1=variance_vector(m1,h1, detected_obs_rel_x(iter1,1,:), detected_obs_rel_x(iter1,2,:), detected_obs_rel_v(iter1,1,:), detected_obs_rel_v(iter1,2,:), detected_obs_size,du_x(iter2,:,:), du_y(iter2,:,:));
     %[m,h,f] = mean_terms(detected_obs_rel_x(:,1,:), detected_obs_rel_x(:,2,:), detected_obs_rel_v(:,1,:), detected_obs_rel_v(:,2,:), detected_obs_size, r);
     %v=variance_terms(m, h, detected_obs_rel_x(:,1,:), detected_obs_rel_x(:,2,:), detected_obs_rel_v(:,1,:), detected_obs_rel_v(:,2,:), detected_obs_size);    
     cost=@(u) cost_PIVO(u,m1,v1,detected_obj,k);
     u=fmincon(objective, u0, A, b, Aeq, beq, lb, ub, cost);

    else
     options = optimoptions('fmincon');
     u=fmincon(objective, u0, A, b, Aeq, beq, lb, ub, nlei, options);
     z=100;
    end
    detected_obj=0;
    detected_obs_rel_x=[];
    detected_obs_rel_v=[];
    detected_obs_size =[];
    f_val=[];
    du_x=[];
    du_y=[];

    r_v=r_v+u;
    r_x=r_x+(r_v*time);
    v_desired=vmax*(g_x-r_x)/norm(g_x-r_x);
    ub=vmax*ones([1,2]) - r_v;
    lb=-vmax*ones([1,2])-r_v;
    
    circ(r_x(1), r_x(2),r);
    hold on
    circ(g_x(1), g_x(2),g);
    hold on
    quiver(r_x(1),r_x(2),r_v(1)/norm(r_v), r_v(2)/norm(r_v));
    hold on
 for i=1:no_o   
    old(i,:)=o_r_x(i,:);
    o_x(i,:)=o_x(i,:)+(o_v(i,:)*time);
    o_r_x(i,:)=o_x(i,:)-r_x;
    o_r_v(i,:)=(o_r_x(i,:)-old(i,:))/time;
    if(1)
         detected_obj=detected_obj+1;
         
         x_dist= o_r_x(i,1) + pearsrnd(mu_x,std_x,skew_x,krt_x,samples,1);
         x_dist_prob=normpdf(x_dist,mu_x,std_x)/sum(normpdf(x_dist,mu_x,std_x));
         y_dist= o_r_x(i,2) + pearsrnd(mu_y,std_y,skew_y,krt_y,samples,1);
         y_dist_prob=normpdf(y_dist,mu_y,std_y)/sum(normpdf(y_dist,mu_y,std_y));
         du_x(:,:,detected_obj)=pearsrnd(mu_ux,std_ux,skew_ux,krt_ux,samples,1);
         du_x_prob=normpdf(du_x,mu_ux,std_ux)/sum(normpdf(du_x,mu_ux,std_ux));
         du_y(:,:,detected_obj)=pearsrnd(mu_uy,std_uy,skew_uy,krt_uy,samples,1);
         du_y_prob=normpdf(du_y,mu_uy,std_uy)/sum(normpdf(du_y,mu_uy,std_uy));
         detected_obs_size(:,:,detected_obj)=o(i);
         detected_obs_rel_x(:, :, detected_obj)= cat(2,x_dist,y_dist) ;
         vx_dist=(x_dist-o_r_x(i,1))*std_vx/(std_x + 0.0000001);
         vy_dist=(y_dist-o_r_x(i,2))*std_vy/(std_y + 0.0000001);
         detected_obs_rel_v(:, :, detected_obj)= (o_r_x(i,:)-old(i,:))/time + cat(2,vx_dist,vy_dist);
       
         for x=1:1000
          %tangent(r_x, [x_dist(x)+r_x(1), y_dist(x)+r_x(2)],r+o(i), r);
          %hold on;
          %circ(x_dist(x)+r_x(1),y_dist(x)+r_x(2),o(i));
         % hold on
        %  quiver(o_x(i,1),o_x(i, 2),detected_obs_rel_v(x, 1,detected_obj)/norm(detected_obs_rel_v(x, :,detected_obj)), detected_obs_rel_v(x, 2,detected_obj)/norm(detected_obs_rel_v(x, :,detected_obj)));
         % hold on
         end
         
    end
     quiver(o_x(i,1),o_x(i, 2),o_v(i, 1)/norm(o_v(i,:)), o_v(i, 2)/norm(o_v(i,:)));
     hold on
    %line([r_x(1), o_x(i, 1)],[r_x(2), o_x(i, 2)]);
    %hold on
    circ(o_x(i,1), o_x(i,2),o(i));
    hold on
 end    
    
    xlim([0 20])
    ylim([0 20])
    %txt=['k=' num2str(k)];
    %txt1=['% of samples avoided:' num2str(z)];
    %text(21,21,txt)
    %text(0,-2,txt1)

    drawnow;
    hold off

    if norm(g_x-r_x)<1
        toggle=0;
    end
    
  
end