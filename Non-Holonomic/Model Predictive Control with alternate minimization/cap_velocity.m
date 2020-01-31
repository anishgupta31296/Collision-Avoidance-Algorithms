function [capped_vel] = cap_velocity(bot_vel, controls, vmax)
    controls;
    capped_vel = bot_vel + controls;
    xlimit = vmax;
    ylimit = vmax;
    if capped_vel(1) < -xlimit
        capped_vel(1) = -xlimit;
    elseif capped_vel(1) > xlimit
        capped_vel(1) = xlimit;
    end
    
    if capped_vel(2) < -ylimit
        capped_vel(2) = -ylimit;
    elseif capped_vel(2) > ylimit
        capped_vel(2) = ylimit;
    end
end