classdef VectorDisplayUtils
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Static)
        function hist(M)
            offset = 0;
            if (size(M,1) == 4) % quaternion
                offset = 1;
            end
            figure(10),
            if (size(M,1) >= 1), subplot(3,1,1), hist(M(offset + 1,:)), end;
            if (size(M,1) >= 2), subplot(3,1,2), hist(M(offset + 2,:)), end;
            if (size(M,1) >= 3), subplot(3,1,3), hist(M(offset + 3,:)), end;
        end
        function psd(M, samplingFreq)
            figure(11),
            if (size(M,1) >= 1), subplot(3,1,1), pwelch(M(1,:), [], [], samplingFreq), end;
            if (size(M,1) >= 2), subplot(3,1,2), pwelch(M(2,:), [], [], samplingFreq), end;
            if (size(M,1) >= 3), subplot(3,1,3), pwelch(M(3,:), [], [], samplingFreq), end;
        end
        function psd_nodc(M, samplingFreq)
            offset = 0;
            if (size(M,1) == 4) % quaternion
                offset = 1;
            end
            if (size(M,1) >= 1), K (1,:) = M(offset + 1,:) - mean(M(offset + 1,:)); end;
            if (size(M,1) >= 2), K (2,:) = M(offset + 2,:) - mean(M(offset + 2,:)); end;
            if (size(M,1) >= 3), K (3,:) = M(offset + 3,:) - mean(M(offset + 3,:)); end;
            VectorDisplayUtils.psd(K, samplingFreq);            
        end
    end
    
end

