clear;
clc;
close all;

mu_x = 0;
std_x = 0.1;
skew_x = 0.0;
krt_x = 3.0;
    
mu_y = 0;
std_y = 0.1;
skew_y = 0.0;
krt_y = 3.0;

mu_ux = 0;
std_ux = 0.0;
skew_ux = 0.0;
krt_ux = 3.0;

mu_uy = 0;
std_uy = 0.0;
skew_uy = 0.0;
krt_uy = 3.0;

std_vx=0.00;
std_vy=0.00;

r=0.5; r_x=[0,0]; r_v=[1,1]; vmax=  1.5;
g=0.5; g_x=[10,10];
o(1)=0.4; o_x(1,:)=[5,5]; o_v(1,:)=[-0.7, -0.7];
o_r_x(1,:)=o_x(1,:)-r_x;
o_r_v(1,:)=o_v(1,:)-r_v;



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
lambda=0;
toggle=1;
time=0.1;
ub=vmax*ones([1,2]);
lb=-vmax*ones([1,2]);
a1=axes('Units','inches');
axes(a1);
no_o= length(o);
range=4;
detected_obj=0;
detected_obs_size=[];
u1=0;
h=function_handles(r,1);
func1=@(u)[u(1)^4, u(2)^4, u(1)^3, u(2)^3, u(1)^2, u(2)^2, u(1), u(2), 1, (u(1)^2)*(u(2)^2), (u(1)^3)*(u(2)), (u(1))*(u(2)^3), (u(1)^2)*(u(2)), (u(1))*(u(2)^2), u(1)*u(2)];
samples=10000;
desired_samples=50;
reduced_samples=50;
while toggle==1

    objective = @(u) norm(v_desired-(u+r_v)) + lambda*norm(u);
    u0=[0 0];
    options = optimoptions('fmincon', 'Display', 'off');

    if(detected_obj)
        pause
        d=2;
        [desired_dist,desired_dist1,desired_coeff,u1,nlei] = desired_distribution(detected_obs_rel_x, detected_obs_rel_v, objective,lb, ub,r,detected_obs_size,d,desired_samples);
        [reduced_sample, reduced_sample1, reduced_set_coeff]=reduced_set_method(detected_obs_rel_x, detected_obs_rel_v, reduced_samples, d);
%         [coeff_obs,hh,hf,ff]=cost_function_coefficients(h, reduced_sample, reduced_sample1, reduced_set_coeff, desired_dist,desired_dist1,desired_coeff,u1, d);
%         cost_func=cost_function(coeff_obs, objective);
        coll_cone=@(u)nlconMMD(u,reduced_sample, reduced_sample1, r, o);
        coll_cone_desired=@(u)nlconMMD(u,desired_dist, desired_dist1, r, o);
        cost_func=@(u)norm((reduced_set_coeff'*kernel(coll_cone(u)',[],d))-(desired_coeff'*kernel(coll_cone_desired(u1)',[],d)));
        [u,fval]=fmincon(cost_func, u1, A, b, Aeq, beq, lb, ub,[],options);

    else
    u=fmincon(objective, u0, A, b, Aeq, beq, lb, ub, [], options);
    end
    
    detected_obj=0;
    detected_obs_rel_x=[];
    detected_obs_rel_v=[];
    detected_obs_size=[];
    
    %(r+o(i))^2-sum(o_r_x(i,:).*o_r_x(i,:)) + ((o_r_x(i,:)*(o_r_v(i,:)-u)')^2)/norm((o_r_v(i,:)-u))^2
    r_x=r_x+((u+r_v)*time);
    v_desired=vmax*(g_x-r_x)/norm(g_x-r_x);
    r_v=r_v+u;
    u0=[0,0];
    ub=vmax*ones([1,2]) - r_v;
    lb=-vmax*ones([1,2])-r_v;
    figure(1)
    subplot(1,1,1)
    circ(r_x(1), r_x(2),r);
    hold on
    circ(g_x(1), g_x(2),g);
    hold on
    %quiver(r_x(1),r_x(2),r_v(1), r_v(2));
    %hold on

 for i=1:no_o   
    old(i,:)=o_r_x(i,:);
    o_x(i,:)=o_x(i,:)+(o_v(i,:)*time);
    o_r_x(i,:)=o_x(i,:)-r_x;
    o_r_v(i,:)=(o_r_x(i,:)-old(i,:))/time;
    if(sensor_range(r_x,o_x(i,:),range) && (dot(o_r_x(i,:),o_r_v(i,:))<0 ))
         detected_obj=detected_obj+1;

         x_dist =o_r_x(i,1) + pearsrnd(mu_x,std_x,skew_x,krt_x,samples,1);
         y_dist =o_r_x(i,2) + pearsrnd(mu_y,std_y,skew_y,krt_y,samples,1);
         detected_obs_size(:,:,detected_obj)=o(i); 
         detected_obs_rel_x(:, :, detected_obj)= cat(2,x_dist,y_dist) ;
         dis=distance(detected_obs_rel_x(:, :, detected_obj));
         vx_dist=(x_dist-o_r_x(i,1))*std_vx/(std_x + 0.0000001);
         vy_dist=(y_dist-o_r_x(i,2))*std_vy/(std_y + 0.0000001);
         detected_obs_rel_v(:, :, detected_obj)= (o_r_x(i,:)-old(i,:))/time + cat(2,vx_dist,vy_dist);
         detected_obs_rel_x=detected_obs_rel_x(dis>((r+o(i))^2),:,:);
         detected_obs_rel_v=detected_obs_rel_v(dis>((r+o(i))^2),:,:);         
         %sqrt(detected_obs_rel_x(:,1,1).^2 + detected_obs_rel_x(:,2,1).^2) -(r+o(i))

 
         %detected_obs_rel_v(:, :, detected_obj)= (detected_obs_rel_x(:, :, detected_obj)-old(i,:))/time;
         %detected_obs_rel_v(:, :, detected_obj)=detected_obs_rel_v(:, :, detected_obj) ;
         
          for x=1:length(detected_obs_rel_x)
         %  tangent(r_x, [detected_obs_rel_x(x,1,detected_obj)+r_x(1), detected_obs_rel_x(x,2,detected_obj)+r_x(2)],r+o(i), r);
         %  hold on;
         %  quiver(r_x(1),r_x(2),-detected_obs_rel_v(x, 1, detected_obj), -detected_obs_rel_v(x, 2, detected_obj));
        %   hold on
           %circ(x_dist(x)+r_x(1),y_dist(x)+r_x(2),o(i));
           %hold on;
           %quiver(o_x(i,1),o_x(i, 2),detected_obs_rel_v(x, 1,detected_obj)/norm(detected_obs_rel_v(x, :,detected_obj)), detected_obs_rel_v(x, 2,detected_obj)/norm(detected_obs_rel_v(x, :,detected_obj)));
           %hold on
          end
         
         %pause;
    end
    quiver(o_x(i,1),o_x(i, 2),o_v(i, 1), o_v(i, 2));
    hold on

    %line([r_x(1), o_x(i, 1)],[r_x(2), o_x(i, 2)]);
    %hold on
    circ(o_x(i,1), o_x(i,2),o(i));
    hold on
 end
    quiver(r_x(1),r_x(2),r_v(1), r_v(2),'color', [1,0,0]);
    hold on
    xlim([0 20])
    ylim([0 20]) 

    drawnow;
    hold off
    
    if norm(g_x-r_x)<1
        toggle=0;
    end
    
  
end

