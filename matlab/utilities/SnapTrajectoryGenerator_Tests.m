clear;
digits(32);
waypoints = [0    0   0;
             1    1   1;
             2    0   2;
             3    -1  1;
             4    0   0]';


subscripts = ['x';'y';'z'];    % What to control
m = 4;                              % Snap is fourth derivative of pos
[instance, solution] = SnapTrajectoryGenerator.GenerateTrajectorySolution(...
    waypoints, subscripts, m, @VelocityFns.constantVelocity);

t = 1.5;
path = instance.getPathAtTime(t);
des_state.pos = double(instance.getPath(path, t, 0));
des_state.vel = double(instance.getPath(path, t, 1));
des_state.acc = double(instance.getPath(path, t, 2));

