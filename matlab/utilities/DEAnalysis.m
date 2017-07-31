%% Displacement Estimation Analysis
%% Uses figures in range 200 to 210
classdef DEAnalysis
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Static)
        % A should be a Mx3 matrix
        function analyzeAcc(A, cutoff, fs, ntaps, type)
            
            Vorig = (1/fs) * cumtrapz(A);
            Dorig = (1/fs) * cumtrapz(Vorig);
            
            DEAnalysis.plotVD(Vorig, Dorig, 'V, D from Original', 201)
            
            Qalign = IMUQuatUtils.GAlignment(A);
            Arot = quatrotate(Qalign, A);
            Afilt = FIRUtils.runHighPassXYZ(Arot, cutoff, fs, ntaps, type);

            Vrot = (1/fs) * cumtrapz(Afilt);
            Drot = (1/fs) * cumtrapz(Vrot);

            DEAnalysis.plotVD(Vrot, Drot, 'Rotated V, D', 202)
            
            Vdrot = quatrotate(quatinv(Qalign), Vrot);
            Ddrot = quatrotate(quatinv(Qalign), Drot);
            
            DEAnalysis.plotVD(Vdrot, Ddrot, 'Double Rotated V, D', 203);
        end
        
        function [Vmeans, Dmeans] = analyzeDrift(filename, fs, fig)            
            load(filename);
            Vorig = (1/fs) * cumtrapz(A');            
            Dorig = (1/fs) * cumtrapz(Vorig);
            Dmeans = DEAnalysis.analyzeVector(Dorig, fig, 'Displacement Drift Analysis: Mean drift vs # of samples used');
            Vmeans = DEAnalysis.analyzeVector(Vorig, fig + 1, 'Velocity Drift Analysis: Mean drift vs # of samples used');
        end

        function [Vmeans, Dmeans] = analyzeRotatedDrift(filename, fs, fig)            
            load(filename);

            Qalign = IMUQuatUtils.GAlignment(A');
            Arot = quatrotate(Qalign, A');

            Vorig = (1/fs) * cumtrapz(Arot);            
            Dorig = (1/fs) * cumtrapz(Vorig);

            Dmeans = DEAnalysis.analyzeVector(Dorig, fig, 'Displacement Drift Analysis: Mean drift vs # of samples used');
            Vmeans = DEAnalysis.analyzeVector(Vorig, fig + 1, 'Velocity Drift Analysis: Mean drift vs # of samples used');
        end

        function means = analyzeVector(VtoAnalyze, fig, mytitle) 
            m = size(VtoAnalyze, 1);
            figure(fig);
            set(gcf,'name',mytitle,'numbertitle','off')
            skipValues = 1;
            means = zeros(1,3);
            Vds = zeros(m/10-1, m);
            for j = 1:3
                Vd = diff(VtoAnalyze(:,j));
                for i = skipValues:m/10-1
                    Vds(i,j) = mean(diff(Vd(skipValues:i*10)));
                end
                subplot(3, 1, j), plot(Vds(:,j)); title(sprintf('Mean Difference on axis %d vs # of samples used', j));
                means(j) = mean(Vd);
            end
        end
       
        function adjustWithMeanDrift(filename, fs, mytitle, fig, Vmeans, Dmeans)
            %filename = 'imuhorizmove.mat';
            load(filename);
            Vorig = (1/fs) * cumtrapz(A');
            Dorig = (1/fs) * cumtrapz(Vorig);
            %mytitle = 'Estimation by subtracting mean drift';
            m = size(Vorig, 1);
            Vest = Vorig - repmat(Vmeans, [m, 1]) .* repmat((0:m-1)', [1, 3]);
            DEAnalysis.plotVD(Vorig, Vest, sprintf('%s, %s', mytitle, ':Velocity'), fig);
            
            Dest = Dorig - repmat(Dmeans, [m, 1]) .* repmat((0:m-1)', [1, 3]);
            DEAnalysis.plotVD(Dorig, Dest, sprintf('%s, %s', mytitle, ':Displacement'), 'Dorig', 'Dest', fig + 1);
        end
        
        % Calculate directly integrated V, D (orig),
        % Calculate rotated versions using Acceleration rotation (Vrot, Drot) have almost zero x, y components
        % Calculate Vest, Dest by a) Computing Vrot - adjustment using Vmean, Drot - adjustment using Dmean
        %                         b) Then rotating the result back by the same quaternion (inverse rotation)
        function [Vorig, Dorig, Vrot, Drot, Vest, Dest] = adjustWithRotatedMeanDrift(filename, fs, mytitle, fig, Vmeans, Dmeans)
            %filename = 'imuhorizmove.mat';
            load(filename);
            Vorig = (1/fs) * cumtrapz(A');
            Dorig = (1/fs) * cumtrapz(Vorig);

            Qalign = IMUQuatUtils.GAlignment(A');
            Arot = quatrotate(Qalign, A');
            Vrot = (1/fs) * cumtrapz(Arot);
            Drot = (1/fs) * cumtrapz(Vrot);
           
            %mytitle = 'Estimation by subtracting mean drift';
            m = size(Vrot, 1);
            Vestrot = Vrot - repmat(Vmeans, [m, 1]) .* repmat((0:m-1)', [1, 3]);
            Vest = quatrotate(quatinv(Qalign), Vestrot);
            DEAnalysis.plotVD(Vorig, Vest, sprintf('%s, %s', mytitle, ':Velocity'),'Vorig', 'Vest', fig);
            
            Destrot = Drot - repmat(Dmeans, [m, 1]) .* repmat((0:m-1)', [1, 3]);
            Dest = quatrotate(quatinv(Qalign), Destrot);
            DEAnalysis.plotVD(Dorig, Dest, sprintf('%s, %s', mytitle, ':Displacement'), 'Dorig', 'Dest', fig + 1);
            
            Dest2 = (1/fs) * cumtrapz(Vest);
            DEAnalysis.plotVD(Dorig, Dest2, sprintf('%s, %s', mytitle, ':Displacement From Vest'), 'Dorig', 'Dest2', fig + 2);
        end
        
        function plotVD(V, D, mytitle, Vtitle, Dtitle, fig)
            figure(fig);
            set(gcf,'name',mytitle,'numbertitle','off')
            subplot(3, 2, 1), plot(V(:,1)), title(sprintf('%s-%s', Vtitle, 'x'));
            subplot(3, 2, 2), plot(D(:,1)); title(sprintf('%s-%s', Dtitle, 'x'));
            subplot(3, 2, 3), plot(V(:,2)), title(sprintf('%s-%s', Vtitle, 'y'));
            subplot(3, 2, 4), plot(D(:,2)); title(sprintf('%s-%s', Dtitle, 'y'));
            subplot(3, 2, 5), plot(V(:,3)), title(sprintf('%s-%s', Vtitle, 'z'));
            subplot(3, 2, 6), plot(D(:,3)); title(sprintf('%s-%s', Dtitle, 'z'));
        end
        
        function plotXYZ(data, mytitle)
            figure(110);
            set(gcf,'name',mytitle,'numbertitle','off')
            for i = 1:3      
                subplot(3,1,i), pwelch(data(:,i), [], [], freq);
                title(['Data Axis: ', num2str(i), ' ', 'Original']);
            end
        end
        
        function plotState(X)
            hfig = figure(204);
            scrsz = get(groot,'ScreenSize');
            pos = [(scrsz(1) +20), (scrsz(2) +20), (scrsz(3) - 20), scrsz(4) - 20];
            hfig.set('OuterPosition',pos);    
            set(gcf,'name','State Estimation','numbertitle','off')
            axis = ['x', 'y', 'z'];
            vector = ['p', 'v', 'a'];
            for i = 1:3 % x,y,z axis
                for j = 1:3 % pos, vel, acc
                    subplot(3,3,(i-1)*3 + j), plot(X(:,(i-1)*3 + j)), title(sprintf('Axis:%s, Vector:%s', axis(i), vector(j)));
                end
            end
        end
    end
    
end

