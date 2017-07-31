%% Zs : Stationary accelerations Mx3 matrix
%% K: Kalman Gain Matrix
%% P: Plant Covariance matrix
%% X: State matrix
function [kf_sys, P, X] = EstimateKalmanFilter(Zs, fs)
%Estimate the Kalman Filter and related parameters given measured
%accelerations in stationary frame.
%   x => state in body frame, 
%       x = [xp, xv, xa]', where xp, xv = zeros(3, 1), xa = [0; 0; gestimated]
%   z => measured state, we only measure acc, 
%       z = accelerometer measurements.
%       zi = ith accelerometer measurement
%       Assuming xi = [0; 0; mod(zi)]
%       Compute Rotation Matrix Ri s.t. zi = Ri * xi;
%           To compute this, find the shortest quaternion qi, then convert
%           it to Ri.
%       C = mean(Ri);
%       Measurement Noise model is estimated as a 3d guassian, using the
%       noise measurements Ci * xi - zi;

%   Assume P = eye(3), assume the plant noise is large, and uncorrelated
%   gaussian white noise

    %Constants
    M = size(Zs, 1);

    %Utilities
    rotation = Rotation();
    
    %Basic Setup
    P = eye(9);         % Constant plant dynamics
    Q = 1000 * eye(9);  % Large plant noise
    Anom = [...
        1,  1/fs    0       ;...
        0,  1,      1/fs    ;...
        0,  0,      1       ;...
    ];
    A = [...
        Anom,       zeros(3,3),     zeros(3,3)  ;...
        zeros(3,3), Anom,           zeros(3,3)  ;...
        zeros(3,3), zeros(3,3),     Anom        ;...
    ];
    B = eye(9);
    U = zeros(M, 9);
    
%     temp = (cumsum(repmat(1/fs, [1,M])) - 1/fs);
%     xvadjust = 0.2 * temp;
%     yvadjust = 1 * temp;
%     zvadjust = 8 * temp;
%     U(:,2) = xvadjust';U(:,5) = yvadjust';U(:,8) = zvadjust';
    
    %Znorms = sqrt(sum(Z' .* Z')); % M x 1 vector
    %Zsnormed = Zs ./ repmat(Znorms', [1, 3]);    
    Qalign = quatinv(IMUQuatUtils.GAlignment(Zs));

    [yaw, pitch, roll] = quat2angle(Qalign);
    Ralign = [yaw, pitch, roll];
    Rmeans = mean(Ralign);
    Cnom = double(rotation.getVectorRotation('ZYX', Rmeans(1), Rmeans(2), Rmeans(3)));
    
    %% Create a C representing rotation of acceleration; we don't measure position or vel.
    C = [ zeros(3,3),     zeros(3,3),     Cnom];    % 3x9
    
%     C = [...
%         zeros(3,3),     zeros(3,3),     zeros(3,3)  ;...
%         zeros(3,3),     zeros(3,3),     zeros(3,3)  ;...
%         zeros(3,3),     zeros(3,3),     Cnom        ;...
%     ];
    % The C calculated in previous step works for a state vector in form [px, py, pz, .., ax, ay, az]
    % but our state vector has the form [px, vx, ax,..., pz, vz, az], so
    % convert appropriately.
    C = C * [... 
        zeros(1,0),     1       , zeros(1,8)    ;...
        zeros(1,3),     1       , zeros(1,5)    ;...
        zeros(1,6),     1       , zeros(1,2)    ;...
        zeros(1,1),     1       , zeros(1,7)    ;...
        zeros(1,4),     1       , zeros(1,4)    ;...
        zeros(1,7),     1       , zeros(1,1)    ;...
        zeros(1,2),     1       , zeros(1,6)    ;...
        zeros(1,5),     1       , zeros(1,3)    ;...
        zeros(1,8),     1       , zeros(1,0)    ;...        
    ];

    %R = .1*eye(9);
    R = cov(Ralign);
    
    kf_sys = struct;
    kf_sys.A = A;
    kf_sys.B = B;
    kf_sys.F = 1;
    kf_sys.Q = Q;

    kf_sys.C = C;
    kf_sys.D = [zeros(3,3),     zeros(3,3), eye(3)];
    kf_sys.G = 1;   
    kf_sys.R = R;
    
    state = [0,0,0, 0,0,0,  0,0,9.8]';
    X = zeros(M, 9);
    for i = 1:M
        [state, P]    = KF(kf_sys, Zs(i,:)', U(i,:)', state, P);
        X(i,:) = state;
    end
    DEAnalysis.plotState(X);
end

