function[]= draw(obs_p, robo_p, goal_p, obs_r, robo_r, goal_r,  bot_path,  x_axis_min, x_axis_max, y_axis_min, y_axis_max)
%This file is used to plot the current state of the agent and obstacles

%the following are the colores used for obstacle and sample
obs_sample_color = [0.79,0.79,0.79];
obs_sample_path_color = [0.79,0.79,0.79];
obs_path_color=[0.50,0.50,0.50];
color_obs= [0.50,0.50,0.50];
axes('position',[0.13 0.11 0.775 0.815])

%% Animation for Obstacles
for k = 1:size(obs_p,1)
    circ(obs_p(k,1),obs_p(k,2),obs_r,color_obs(1,:),color_obs(1,:))
    hold on; 
end


%% animation for robo1


%plot the agent
circ(robo_p(1), robo_p(2), robo_r,[0 0 1], [0 0.6 1],1.2);
hold on

%plot agent's path
plot(bot_path(:,1),bot_path(:,2),'--','color',[0 0 1],'Linewidth',1.6);
hold on;

%% plot the goal
circ(goal_p(1), goal_p(2), goal_r, [1 0.874 0], [1 1 1], 2, '--');
hold on
circ(goal_p(1), goal_p(2), goal_r-0.05, [1 0.874 0], [1 0.874 0]);
hold on
  
%% Draw
  axis([x_axis_min x_axis_max  y_axis_min y_axis_max])
  hold off
  drawnow;
  
  
%% Saving each thumbnail
% if(constant==0)
%    directory=['/home/anish/Non Holonomic/Case_',num2str(situation),'(Deterministic)'];
% else
%    directory=['/home/anish/Non Holonomic/Case_',num2str(situation),'(K=',num2str(constant),')'];    
% end
% mkdir(directory);
% file_name=num2str(size(sample_path,1),'%04.f');
% dest=fullfile(directory,file_name);
% print(gcf,[dest,'.png'],'-dpng','-r600')
end 
