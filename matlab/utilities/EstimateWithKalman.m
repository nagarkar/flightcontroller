classdef EstimateWithKalman   
    properties
    end
    
    methods (Static)        
        %% Zs: Altitude and vertical acceleration (down is positive) measurements from barometer and 
        function [kf_sys, P, X, K] = AltBiasFromAccBar(Zs, fs, initialState)
        %{
            System Model:
                T = 1/fs
                x = [pos, vel, acc, acc_bias]'
                    A = [...
                        1   T   .5*T^2     0   ;...
                        0   1   T              0   ;...
                        0   0   1              0   ;...
                        0   0   0              1   ;...
                    ];
                    Q = eye(4);
                z = [pos_barometer, acc_gyro]'
                    pos_barometer = pos + noise;
                    acc_gyro = acc + acc_bias + noise
                    
                    C = [...
                        1   0   0   0;
                        0   0   1   1;
                    ];            
                    
        %}
            T = 1/fs;
            A = [...
                1   T   0.5*T^2    0   ;...
                0   1   T               0   ;...
                0   0   1               0   ;...
                0   0   0               1   ;...
            ];

            P = [...
                0        0    0           0           ;...
                0        0    0           0           ;...
                0        0    0           0           ;...
                0        0    0           0   ;...
            ];
        
%             Q = [...
%                 0.00001    0              0             0           ;...
%                 0              0.00001    0             0           ;...
%                 0              0              0.00001   0           ;...
%                 0              0              0             .00001   ;...
%             ];            
            % Setting Q to zero makes all outputs go to zero.
            Q = 0.000000001 * eye(4);
            Q(4,4) = .000000000000000000000001;
            %Q = 0.01 * eye(4);

            C = [...
                1   0   0   0;
                0   0   1   1;
            ];
            %R = eye(2);
            R = [0.20     0.00003;...
                    0.00003     0.000005 ];          % Measurement Noise
             %R = [0.06, -0.0003; -0.0003, 0.00005];
%         
%              R = [.2625       -0.0003;...
%                       -0.0003             0 ];          % Measurement Noise

            kf_sys = EstimateWithKalman.KFSYS_A_B_F_Q_C_D_G_R(A, 0, 1, Q, C, 0, 1, R);

            AltitudeSettlingTime = .05;
            AltitudeSettlingWindow = ceil(fs*AltitudeSettlingTime);
            SamplesForAveraging = AltitudeSettlingWindow;
            %SamplesForAveraging = 1;
            M = size(Zs, 1);                                    
            
            %p0 =   mean(Zs(1:SamplesForAveraging, 1));            
            %g =     mean(Zs(1:SamplesForAveraging, 2));
            
            p0 =   Zs(SamplesForAveraging, 1);            
            g =     Zs(SamplesForAveraging, 2);
            
            if isempty(initialState)                
                state = [p0, 0,  0, g]';
            else
                state = reshape(initialState, [4,1]);
            end

            X = zeros(M, 4); 
            U =0;
            
            %AltitudeMovMeanWindow = AltitudeSettlingWindow;
            %Z = [movingmean(Zs(:,1), AltitudeMovMeanWindow), Zs(:,2)];
            Z = Zs;
            for i = 1:M
                [state, P, K]    = KF(kf_sys, Z(i,:)', U, state, P);
%                 if (i > 10000)
%                     K
%                 end
                X(i,:) = state;
            end
            figure(220),
            set(gcf,'name','Z Axis position, velocity, acc, acc-bias','numbertitle','off');
            t2p = 8;    % thingstoplot
            cols = 2;
            %validInterval = 500:size(X, 1);
            validInterval = 1:size(X,1);
            subplot(t2p/cols,cols,1), plot(X(validInterval,1)), title('Position');
            subplot(t2p/cols,cols,2), plot(X(validInterval,2)), title('Vel');
            subplot(t2p/cols,cols,3), plot(X(validInterval,3)), title('Acc');
            subplot(t2p/cols,cols,4), plot(X(validInterval,4)), title('Bias');
            subplot(t2p/cols,cols,5), plot(Zs(validInterval,1)), title('Measured Alt');
            subplot(t2p/cols,cols,6), plot(Zs(validInterval,2)), title('Measured Acc');
            %subplot(t2p/cols,cols,7), plot(Z(validInterval,1)), title(sprintf('Measured Alt (moving mean), %d', AltitudeMovMeanWindow));
        end         
        function [kf_sys] = KFSYS_A_B_F_Q_C_D_G_R(A, B, F, Q, C, D, G, R)
            kf_sys = struct;
            kf_sys.A = A;
            kf_sys.B = B;
            kf_sys.F = F;
            kf_sys.Q = Q;

            kf_sys.C = C;
            kf_sys.D = D;
            kf_sys.G = G;   
            kf_sys.R = R;
        end
    %{
            Zs : Stationary accelerations Mx3 matrix
            K: Kalman Gain Matrix
            P: Plant Covariance matrix
            X: State matrix
        %}
        function [kf_sys, P, X] = PositionVelocityFromAcc(Zs, fs)
            %{
            Estimate the position, velocity given measured accelerations in body frame.
              x => state in body frame, 
                  x = [xp, xv, xa]', where xp, xv = zeros(3, 1), xa = [0; 0; gestimated]
              z => measured state, we only measure acc, 
                  z = accelerometer measurements.
                  zi = ith accelerometer measurement
                  Assuming xi = [0; 0; mod(zi)]
                  Compute Rotation Matrix Ri s.t. zi = Ri * xi;
                      To compute this, find the shortest quaternion qi, then convert
                      it to Ri.
                  C = mean(Ri);
                  Measurement Noise model is estimated as a 3d guassian, using the
                   noise measurements Ci * xi - zi;
            %}

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
                [state, P, K]    = KF(kf_sys, Zs(i,:)', U(i,:)', state, P);
                X(i,:) = state;
            end
            DEAnalysis.plotState(X);
        end
    end
    
end

