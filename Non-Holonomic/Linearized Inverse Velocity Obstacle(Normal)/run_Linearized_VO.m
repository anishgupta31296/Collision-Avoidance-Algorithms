clc
clear all;
close all

%========================================%
%Simulation details
%========================================%

agent_r = 0.5;
obs_r = 1.0;
goal_r=0.5;
vel_cap_max = 1.5;
vel_cap_min = 0.5;
w_cap = 1.0;
dt = 0.1;
sensor_range=8;
lb = [-0.1; -0.1];
ub = [0.1; 0.1];
ub_w=ub(2);
lb_w=lb(2);
ub_v=ub(1);
lb_v=lb(1);
u0=[0 0];

%Collision cone without linearization

%% Simulation parameters for the robot, obstacle and goal
Case=0;
agent_p = [0,0];
obs_p = [10,9.99;];
obs_v = [-1,-1];
goal_p = [15,15];
% % 
% Case =3
%  agent_p = [-7,7];
%  obs_p = [5,-5;-7,-2;4,6;];
%  obs_v = [-0.6,0.8;1.0,0.9;-0.5,-0.5;];
%  goal_p = [7,-7];
% %  
% %  Case =19 
% % agent_p = [-15,7];
% % obs_p = [-4,4;-2,2;0,0;-4,-4;-2,-2;-6,-6;10,0;10,2.1;10,4.2;10,-2.1;10,-4.2;10,-6.3];
% % obs_v = [-1,-0.6;-1,-0.6;-1,0;-0.8,0.8;-0.8,0.8;-0.8,0.8;-0.8,0;-0.8,0;-0.8,0;-0.8,0;-0.8,0;-0.8,0];
% % goal_p = [7,-7];
% 
% Case =20
% agent_p = [-15,7];
% obs_p = [0,0;2,0;4,0;0,3;2,3;4,3;0,6;2,6;4,6];
% obs_v = [-1,0;-1,0;-1,0;-1,0;-1,0;-1,0;-1,0;-1,0;-1,0];
% goal_p = [7,-7];


%%
%========================================%
%Simulation details continued
%========================================%

%Calculating the axis limits based on the agent's, obstacles' and goal
x_min=min([obs_p(:,1); agent_p(1); goal_p(1)]);
x_max=max([obs_p(:,1); agent_p(1); goal_p(1)]);
y_min=min([obs_p(:,2); agent_p(2); goal_p(2)]);
y_max=max([obs_p(:,2); agent_p(2); goal_p(2)]);
range=max([(x_max-x_min) (y_max-y_min)]);
x_axis_min=(x_min + x_max)/2  - range/2 -2;
x_axis_max=(x_min + x_max)/2  + range/2 +2;
y_axis_min=(y_min + y_max)/2  - range/2 -2;
y_axis_max=(y_min + y_max)/2  + range/2 +2;


current_head = atan2(goal_p(2)-agent_p(2),goal_p(1)-agent_p(1));%variable to store current velocity direction
prev_rel_p = [];
current_rel_p = [];
agent_v = [1.5,0];
num_of_obs = size(obs_p,1);
bot_path = agent_p;%Varable to store robot's traces to plot it's path
obs_path = [];%Varable to store obstacle's traces to plot it's path
sample_path={};%Varable to store obstacle samples' traces to plot it's path
prev_agent_p = agent_p;
prev_obs_p = obs_p;
prev_rel_p=obs_p-agent_p;
counter = 1;
acc=[];
prev_v=0;
options = optimoptions(@fmincon,'Display', 'Off');
%% Start of simulation
while norm(agent_p-goal_p)>1    
    %% Updating position and velocities values
    figure(1)

    % Updating positions and plotting them
    current_head = current_head + agent_v(2)*dt; %updating our current heading
    v = [(agent_v(1))*cos(current_head ),(agent_v(1))*sin(current_head)]; %velocity value in x and y direction from v and w controls
    agent_p=agent_p  + v*dt;%updating robots position
    bot_path = [bot_path; agent_p]; %storing robots path   
    obs_p = obs_p + obs_v*dt;%updating obstacles position
    obs_path(:,:,counter) = obs_p;%storing obstacles path
    current_rel_p = obs_p - agent_p;%relative position
    current_rel_v = (current_rel_p-prev_rel_p)/dt;;%relative velocity
    lb=[lb_v; lb_w];
    ub=[ub_v; ub_w];
    if(agent_v(1)>vel_cap_min)
        lb(1)=-(agent_v(1)-vel_cap_min);
    end
    
    if(agent_v(2)>ub_w)
        lb(2)=-agent_v(2);
    elseif(agent_v(2)<lb_w)
        ub(2)=-agent_v(2);
    end
  
    if(agent_v(1)>(vel_cap_max-ub(1)))
      ub(1)=vel_cap_max-agent_v(1);
    elseif(agent_v(1)<(vel_cap_min-lb(1)))    
      lb(1)= -(agent_v(1)-vel_cap_min);
    end
    if(agent_v(2)>(w_cap-ub(2)))
      ub(2)=w_cap-agent_v(2);
    elseif(agent_v(2)<(-w_cap-lb(2)))    
      lb(2)= -(agent_v(2)+w_cap);   
    end  
    %% Saving induvidual variables to be passed to optimizer
  
    vr=agent_v(1);
    wr=agent_v(2);
    xr=agent_p(1);
    yr=agent_p(2);
    xob=obs_p(:,1)';
    yob=obs_p(:,2)';
    xobdot=obs_v(:,1)';
    yobdot=obs_v(:,2)';
    R=agent_r+obs_r;

    %% Detected Obstcles
    detected=[];
    infront_condition=0;
    for l=1:num_of_obs
     infront_condition = dot(current_rel_p(l,:),current_rel_v(l,:))<0;
     if(norm(agent_p-obs_p(l,:))<sensor_range && infront_condition)
      detected=[detected l]; 
     end
    end
    xob=xob(:,detected);
    yob=yob(:,detected);
    xobdot=xobdot(:,detected);
    yobdot=yobdot(:,detected);
    v_desired=vel_cap_max*(goal_p-agent_p)/norm(goal_p-agent_p);
    objective = @(u) norm(v_desired-((agent_v(1)+u(1))*[cos(current_head + dt*(agent_v(2)+u(2))) sin(current_head + dt*(agent_v(2)+u(2)))])) ;

     %% Optimization
       if(~isempty(detected))
        opt=optimize( current_head, vr, wr,  xr, yr, xob, yob, xobdot, yobdot,R,dt, lb, ub,objective);
       else
        opt=fmincon(objective, [0,0], [], [], [], [], lb, ub);   
       end
       agent_v = agent_v+opt;%updating the controls
    %% updating the other parameters
    if(~isempty(detected))
        acc=[acc (norm(v)-norm(prev_v))/dt];
    end
    prev_rel_p=current_rel_p;
    prev_v=v;
    draw(obs_p, agent_p, goal_p, obs_r, agent_r,goal_r,   bot_path,x_axis_min, x_axis_max, y_axis_min, y_axis_max)
    counter = counter + 1; 
    pause
    % --------------------------------------------------------------------------------
end