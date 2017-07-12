function XYZ = EllipsoidSaveFromSerial( comport, baud, prefix, separator, filename, measurements, pickPeriod)
    XYZ = NaN;
    % Close any open serial terminals and open a new one
    CloseSerial
    s = serial(comport, 'BaudRate', baud);
    s.InputBufferSize = 48000;
    s.ReadAsyncMode = 'continuous';
    fopen(s);

    counter = -1;
    
    fid = fopen(filename, 'w', 'native', 'UTF-8');

    if fid == -1
        error('Cannot create file.');
    end
%     for i = 1:size(headings, 1)
%         fwrite(fid, headings(i,:), 'char');
%     end
    try 
        while(1 == 1)
            if s.BytesAvailable > 0
                c = fscanf(s, '%s', 200);
                if strncmp(c, prefix, length(prefix))
                    if mod(counter, pickPeriod) ~= 0
                        continue;
                    end
                    counter = counter + 1;
                    if (counter > measurements) 
                        break;
                    end
                    strLine = regexp(c, separator,'split');
                    line = str2double(strLine);
                    arrayToWrite = line(:,2:end);
                    fwrite(fid, arrayToWrite, 'double');
                end
            end
        end
    catch ME
        close(fid);
        warning(ME);
        return;
    end
    fclose(fid);
    XYZ = MagnetometerBiasEstimator(filename);
    EllipsoidFitPlot(XYZ);
end