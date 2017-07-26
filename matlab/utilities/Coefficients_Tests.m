clear;
digits(32);
waypoints = [0    0   0;
             1    1   1;
             2    0   2;
             3    -1  1;
             4    0   0]';


subscripts = ['x';'y';'z'];    % What to control
m = 4;                              % Snap is fourth derivative of pos

nsubs = length(subscripts);
ncoeff = m*2;                  % Order 7 polynomial
system_order = ncoeff - 1;

coeff_inst = Coefficients(waypoints, ncoeff, subscripts, @VelocityFns.constantVelocity);
n = size(coeff_inst.m_waypoints, 2);
% Get T and S vectors
[T, S] = coeff_inst.getTVector();
 % Precompute coefficient matrices (not necessary)
 for i = 1:2
    coeff_inst.getCoeffMatrix(i);
 end

%  coeff_m_expected = [C_2_0x, C_2_1x, C_2_2x, C_2_3x, C_2_4x, C_2_5x, C_2_6x, C_2_7x;
%     C_2_0y, C_2_1y, C_2_2y, C_2_3y, C_2_4y, C_2_5y, C_2_6y, C_2_7y;
%     C_2_0z, C_2_1z, C_2_2z, C_2_3z, C_2_4z, C_2_5z, C_2_6z, C_2_7z;];
%  assert(isequal(coeff_m,coeff_m_expected))

coeff_inst.initializeEquations();
% Add Constraints on starting position for paths (special case for first path)
path = 1;
t = 0;
deriv = 0;
coeff_inst.addEquation(coeff_inst.getPath(path, t, deriv), [0;0;0]);
for path = 2:n
    t = S(path - 1);
    coeff_inst.addEquation(coeff_inst.getPath(path, t, deriv),...
        coeff_inst.getWaypoint(path - 1));
end

% Add Constraints on ending position for paths
deriv = 0;
for path = 1:n
    t = S(path);
   coeff_inst.addEquation(coeff_inst.getPath(path, t, deriv),...
    	coeff_inst.getWaypoint(path));
end

for deriv = 1:(m-1)
    path = 1;
    t = 0;
    coeff_inst.addEquation(coeff_inst.getPath(path, t, deriv),...
    	[0; 0; 0]);
    path = n;
    t = S(path);
    coeff_inst.addEquation(coeff_inst.getPath(path, t, deriv),...
    	[0; 0; 0]);
end

for deriv = 1:(system_order -1)
    for path = 1:(n-1)
        coeff_inst.addEquation(coeff_inst.getPath(path, S(path), deriv),...
        	coeff_inst.getPath(path + 1, S(path), deriv));
    end
end

precision = 10;
solution = coeff_inst.solve(precision);

numpaths = coeff_inst.getNumPaths() ;
for path = 1: numpaths
    target = double(coeff_inst.getPath(path, S(path), 0));
    expected = coeff_inst.getWaypoint(path);
    assert(isalmost(target, expected, .0001, 1));
end

for path = 2: numpaths 
    target = double(coeff_inst.getPath(path, S(path-1), 0));
    expected = coeff_inst.getWaypoint(path - 1);
    assert(isalmost(target, expected, .0001, 1));
end
tic
XYZ = [];
for t = 0:.01:coeff_inst.m_S(end)  % About 1000 iterations.
    path = coeff_inst.getPathAtTime(t);
    XYZ= [XYZ, double(coeff_inst.getPath(path, t, 0))];
    des_state.vel = double(coeff_inst.getPath(path, t, 1));
    des_state.acc = double(coeff_inst.getPath(path, t, 2));    
end
figure
plot3(XYZ(1,:), XYZ(2,:), XYZ(3,:));
hold on
xlabel('X');
ylabel('Y');
zlabel('Z');
text(waypoints(1,:), waypoints(2,:), waypoints(3,:), strcat('wp-', int2str([1:size(waypoints,2)]')))
toc