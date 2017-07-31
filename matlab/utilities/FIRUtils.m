classdef FIRUtils
    properties
    end
    
    methods (Static)

        function [fdata] = plotXYZ(b, F, A, w, h, data, freq, showGraphs)
            fdata = zeros(size(data));
            for i = 1:3
                fdata(:, i) = filter(b, 1, data(:,i));
            end
            if (isempty(showGraphs) || ~showGraphs)
                return;
            end
            figure
            subplot(1,1,1), plot(F,A,w/pi,abs(h));	% Plot Freq Resp
            for i = 1:3      
                figure;                
                subplot(2,1,1), pwelch(data(:,i), [], [], 952);
                title(['Data Axis: ', num2str(i), ' ', 'Original']);
                subplot(2,1,2), pwelch(fdata(:,i), [], [], freq);
                title(['Data Axis: ', num2str(i), ' ', 'Filtered']);
            end
        end
        function plotXYZRaw(data, freq)
            figure;                
            for i = 1:3      
                subplot(3,1,i), pwelch(data(:,i), [], [], freq);
                title(['Data Axis: ', num2str(i), ' ', 'Original']);
            end
        end
        
        function [b, h, w] = getFilter(ntaps, F, A)
            b = fir2(ntaps,F,A);       % Frequency sampling-based FIR filter design            
            [h,w] = freqz(b,1,128); % Frequency response of filter
        end
        
        function [fdata, b, F, A, h, w] = runLowPassXYZ(data, cutoff, freq, ntaps, showGraphs)
            F = [0, cutoff*2/freq,  cutoff*2/freq,  1];      % Frequency breakpoints
            A = [1, 1,              0,              0];          % Magnitude breakpoints
            [b, h, w] = FIRUtils.getFilter(ntaps, F, A);
            fdata = FIRUtils.plotXYZ(b, F, A, w, h, data, freq, showGraphs);
        end
        
        function [fdata, b, F, A, h, w] = runHighPassXYZ(data, cutoff, freq, ntaps, showGraphs)
            F = [0, cutoff*2/freq,  cutoff*2/freq,  1];      % Frequency breakpoints
            A = [0, 0,              1,              1];          % Magnitude breakpoints
            [b, h, w] = FIRUtils.getFilter(ntaps, F, A);
            fdata = FIRUtils.plotXYZ(b, F, A, w, h, data, freq, showGraphs); % return filtered
        end
    end    
end

