function h = visualize_test(data)
    % Create the quadcopter object. Returns a handle to
    % the quadcopter itself as well as the thrust-display cylinders.
    t  = quadcopter;
    q = [1 0 0 0];
    ax = gca;
    ax.XLim = [-6 6]
    ax.YLim = [-6 6]
    ax.ZLim = [-5 5]
    ax.XLimMode = 'manual'
    ax.YLimMode = 'manual'
    ax.ZLimMode = 'manual'
    axang = SpinCalc('QtoEV', q, 0.001, 1);
    M = makehgtform('axisrotate', axang(1:3), axang(4)*pi/180);
    set(t,'Matrix', M);

    % Set axis scale and labels.
    %axis([-10 30 -20 20 5 15]);
    %zlabel('Height');
    %title('Quadcopter Flight Simulation');

    % Animate the quadcopter with data from the simulation.
    %animate(data, t, thrusts, plots);
end