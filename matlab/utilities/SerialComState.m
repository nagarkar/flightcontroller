%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Talks to Serial Communication port COM4, and retrieves attitude quaternion, linear
% acc, angular rates, magnetic field, and altitude data.
% Usage:
%    SerialComState(50, 115200 * 3, 'imusample.mat', false, false).run();
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef SerialComState < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Data in
        m_fs, m_maxDataPointsToStore, m_nFields, m_filename;

        % Display settings, profiling
        m_show, m_profiler;
        
        % Derived display variables
        m_effectiveFramesamplingInterval, m_quadGraphic, m_altitudePlot, m_filteredAltitudeResults;

        % Resources/handles
        m_serial, m_timer;
        
        %State management
        m_dataCounter;
        m_done; % false = not done, 1/true = done
        
        % Kalman Altitude Filter
        m_kf_sys, m_P, m_state;
        
        %Data out
        m_results;
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%
        % INPUTS
        % recordForSeconds: How much real time we want to record
        % fs:               Sampling frequency (how often is data available on the COM port)
        % port, baud:       Serial Port settings
        % filename:         Name of file to put result variables into. The
        % OUTPUT variables are:
        %   Q:       Quaternion Rotation describing attitude relative to earth frame (Mx4)
        %   A:       Accelerations measured (Mx3), in m/s*s
        %   M:       Magnetic Fields measured (Mx3)
        %   G:       Gyro Measurements (Mx3), angular velocity in rad/sec
        %   Altitude:Altitude in meters
        %   TimeStamps: These indicate the value of the internal counter at the
        %       time of measurement. The counter may not have consecutive values,
        %       but the interval between coutner values should stay the same if
        %       the data is sent with a constant sampling period.
        %   
        % Debugging: If data is malformed, try increasing the baud rate,
        % reducing the sampling frequency. Both of these require changes in
        % the data-source, not this file. To reduce sampling frequency, you
        % can simply send data on the serial line once every 'k' samples,
        % thereby maintaining your internal sampling (for any processing
        % you are doing in the source).
        %
        % USAGE: 
        % >> SerialComState(1000, 'COM4', 921600, 'imusample.mat', false, false).run();
        % Wait for completion message "Job Complete"
        % >> load imusample.mat;
        %%%%%%%%%%%%%%%%%%%%%%%%
        function obj = SerialComState(recordForSecs, nFields, fs, port, baud, filename, doprofiling, show)            
            obj.m_fs = fs;
            obj.m_maxDataPointsToStore = ceil(recordForSecs*fs);
            obj.m_nFields = nFields;
            obj.m_filename = filename;
            obj.m_show = show;         
            obj.m_profiler = doprofiling;
            obj.m_dataCounter = 0;
            obj.m_results = cell(1, obj.m_maxDataPointsToStore);            
            obj.m_done = false;
            framerate = 40;
            obj.m_effectiveFramesamplingInterval = ceil(framerate*fs/1000);
            
            numReplots = rem(obj.m_maxDataPointsToStore, obj.m_effectiveFramesamplingInterval);

            obj.createKalmanSystem();
            
            obj.setupGraphic();
            try 
                fclose(instrfind);
                delete(instrfind);
                delete 'serial_result.txt';
            catch ME
                disp(ME.message);
                disp(ME.cause);
            end
            
            obj.m_serial.BytesAvailableFcnMode = 'terminator';
            obj.m_serial.Terminator = 'CR/LF';
            obj.m_serial = serial(port, 'BaudRate', baud,'RecordName', 'serial_result.txt');
            obj.m_serial.InputBufferSize = 100000; % 100 kB
            
            disp('serialcomstate object created');
        end
        
        function run(obj)
            obj.m_serial.BytesAvailableFcn = {@obj.dataAvailableEvt};
            obj.m_serial.BreakInterruptFcn = {@obj.breakEvt};
            obj.m_serial.ErrorFcn           = {@obj.errEvt};
            obj.m_serial.OutputEmptyFcn     = {@obj.outputEmptyEvt};
            fopen(obj.m_serial);                        
            disp('serialcomstate serial opened');
        end
        
        function status = serialIsOpen(obj) 
            status = false;
            if strncmpi(obj.m_serial.Status,'open', 4)
                status = true;
            end
        end
        
        function breakEvt(~)
            disp('break occurred');
        end
        
        function errEvt(~)
            disp('error occurred');
        end
        
        function outputEmptyEvt(~)
             disp('output empty occurred');
        end
        
        function dataAvailableEvt(obj, ~, ~)            
           
            %% Counters
            mdp = obj.m_maxDataPointsToStore;
            cnt = obj.m_dataCounter;
            
            if (mod(cnt, obj.m_fs) == 0)
                fprintf('.');
            end
            
            if (cnt == 0)
                obj.m_timer = tic;                
                if obj.m_profiler, profile on, end
            end
            
            
            %% Terminal Condition            
            if ((mdp ~= -1 && cnt == mdp) || (mdp == -1 && cnt == 1000)) && obj.m_done == 0
            
                % Profiling
                elapsed = toc(obj.m_timer);
                if obj.m_profiler, profile viewer, end;
                fprintf('Time elapsed in serial data loop: %f\n', elapsed);
                
                % Transform into variables for analysis
                dc = obj.m_dataCounter; % datacollected, for brevity.
                Q = repmat([1 0 0 0]', [1, dc]);
                A = zeros(3, dc);
                G = zeros(3, dc);
                M = zeros(3, dc);
                Altitude = zeros(1, dc);
                FAltitude = zeros(1, dc);
                TimeStamps = zeros(1, dc);
                ResMat = zeros(dc, obj.m_nFields);
                ress = char(obj.m_results);
                
                skipped = 0;
                for idx = 1:dc                    
                    res = strsplit(ress(idx,:),',');      %regexp(c, ',','split');
                    res = res(2:end)';
                    res = str2double(res);
                    len = length(res);
                    if (len ~= obj.m_nFields)
                        skipped = skipped + 1;
                        continue;
                    end
                    i = idx - skipped;
                    Q(:,i) = [res(1); res(2:4)];
                    A(:,i) = res(5:7);
                    G(:,i) = res(8:10);
                    M(:,i) = res(11:13);
                    Altitude(i) = res(14);
                    FAltitude(i) = res(15);
                    TimeStamps(i) = res(obj.m_nFields); % The last field should be the timestamp
                    ResMat(i,:) = res;  
                end
                Q = Q(:,1:end-skipped);
                A = A(:,1:end-skipped);
                G = G(:,1:end-skipped);
                M = M(:,1:end-skipped);
                Altitude = Altitude(1:end-skipped);
                TimeStamps = TimeStamps(1:end-skipped);
                ResMat = ResMat(1:end-skipped, :);

                % Save data
                save(obj.m_filename, 'Q', 'A', 'G', 'M', 'Altitude', 'FAltitude', 'TimeStamps');                                                             
                fclose(instrfind);
                delete(instrfind);
                obj.m_done = 1;                
                fprintf('Job Complete\n');
                %assert(abs((TimeStamps(end) - TimeStamps(1))/930 - 1) <= .05, 'Failed Validation: Missing TimeStamps')
                fprintf('mdp:%d\n', mdp);
                assert(sum(diff(TimeStamps) == 5) == mdp -1, 'Failed Validation: Missing TimeStamps')
                
                return;
            end

            if obj.m_done == 1
                throw(MException('SERIALCOMSTATE:DONE','Trying to call function when done'));
            end
            c = fgets(obj.m_serial);
            if strncmpi(c, 'o', 1)
                obj.m_dataCounter = obj.m_dataCounter + 1;
                cnt = obj.m_dataCounter;    %for brevity                
                obj.m_results(cnt) = cellstr(c);

                if (mod(cnt, obj.m_effectiveFramesamplingInterval) ~= 0)
                    return;
                end
                
                if (~obj.m_show)
                    return;
                end
                res = regexp(c, ',','split');
                res = res(2:end);
                res = str2double(res);
                obj.updateQuadGraphic(res);
                %obj.updateAltitudeGraphic(res);
                drawnow limitrate % display updates
            end
        end
        function createKalmanSystem(obj)
            T = 1/obj.m_fs;
            A = [...
                1   T   0.5 * T^2  0   ;...
                0   1   T               0   ;...
                0   0   1               0   ;...
                0   0   0               1   ;...
            ];

            obj.m_P = [...
                0        0    0           0           ;...
                0        0    0           0           ;...
                0        0    0           0           ;...
                0        0    0           0   ;...
            ];        
            Q = 0.00000001*eye(4);

            C = [...
                1   0   0   0;
                0   0   1   1;
            ];
            R = [10   0;...
                      0    .25 ];          % Measurement Noise
            obj.m_kf_sys = EstimateWithKalman.KFSYS_A_B_F_Q_C_D_G_R(A, 0, 1, Q, C, 0, 1, R);
        end
        function updateAltitudeGraphic(obj, res)
            altitude = res(15);
            subplot(1,2,2);
            if (obj.m_show && obj.m_dataCounter / obj.m_fs > 5)     % Skip 5 seconds of data since it won't change or will oscillate quite a bit.
                set(obj.m_altitudePlot, 'YData', [NaN, altitude, NaN]);
                drawnow limitrate;
            end            
        end
        function updateQuadGraphic(obj, res)
            q = res(1:4);
            % Update the quadcopter graphic
            axang = SpinCalc('QtoEV', [q(2:4) q(1)], 0.001, 1);
            M = makehgtform('axisrotate', axang(1:3), axang(4)*pi/180);
            set(obj.m_quadGraphic,'Matrix', M);
        end
        function setupGraphic(obj)
            close all;
            if obj.m_show ~= true
                return;
            end
            figure(1000);
            subplot(1,2, 1);
            obj.m_quadGraphic = quadcopter();
            % Fix the axes so you don't get jerky motion    
            ax = gca;
            ax.XLim = [-6 6];
            ax.YLim = [-6 6];
            ax.ZLim = [-5 5];
            ax.XLimMode = 'manual';
            ax.YLimMode = 'manual';
            ax.ZLimMode = 'manual';
            ax.YLabel.String = 'Y';
            ax.ZLabel.String = 'Z';
            ax.XLabel.String = 'X';
            subplot(1,2,2);
            obj.m_altitudePlot = plot([-1,0,1],[0,0.5 + randn(),0]);
            tilefigs([1,2], 50);
        end
    end
    
end