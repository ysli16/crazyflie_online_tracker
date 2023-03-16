function plot_trajectory(axes, reference, curr_state, next_state, sim_t, sim_timestep)
% The function plots the commands received and the simulated states in real
% time.
    subplot(axes(1))
    hold on
    plot([sim_t-sim_timestep, sim_t], [curr_state(4), next_state(4)], 'blue')
    plot([sim_t-sim_timestep, sim_t], [reference(1), reference(1)], 'red')
    xlabel(axes(1), 'time[s]')
    ylabel(axes(1), 'x velocity[m/s]')

    subplot(axes(2))
    hold on
    plot([sim_t-sim_timestep, sim_t], [curr_state(5), next_state(5)],'blue')
    plot([sim_t-sim_timestep, sim_t], [reference(2), reference(2)],'red')
    xlabel(axes(2), 'time[s]')
    ylabel(axes(2), 'y velocity[m/s]')

    subplot(axes(3))
    hold on
    plot([sim_t-sim_timestep, sim_t], [curr_state(3), next_state(3)],'blue')
    plot([sim_t-sim_timestep, sim_t], [reference(3), reference(3)],'red')
    xlabel(axes(3), 'time[s]')
    ylabel(axes(3), 'z position[m]')

    subplot(axes(4))
    hold on
    plot([sim_t-sim_timestep, sim_t], [curr_state(9), next_state(9)],'blue')
    plot([sim_t-sim_timestep, sim_t], [reference(4), reference(4)], 'red')
    xlabel(axes(4), 'time[s]')
    ylabel(axes(4), 'yaw[rad]')

    subplot(axes(5))
    hold on
    scatter(next_state(1), next_state(2),'blue')
end

