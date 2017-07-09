% Create the quad graphic
t = quadcopter();
    
% Fix the axes so you don't get jerky motion
ax = gca;
ax.XLim = [-6 6]
ax.YLim = [-6 6]
ax.ZLim = [-5 5]
ax.XLimMode = 'manual'
ax.YLimMode = 'manual'
ax.ZLimMode = 'manual'

% Close any open serial terminals and open a new one
CloseSerial
s = serial('COM4', 'BaudRate', 115200);
s.InputBufferSize = 48000;
s.ReadAsyncMode = 'continuous';
fopen(s);

counter = -1;
M = eye(4);
q = [1 0 0 0];
while(1 == 1)
    if s.BytesAvailable > 0
        c = fscanf(s, '%s', 200);
        counter = counter + 1;
        if strncmpi(c, 'orientation', 11)
            res = regexp(c, ',','split');
            res = str2double(res);
            qnew = res(:,2:end);
            if length(qnew) ~= 4
                continue;
            end
            axang = SpinCalc('QtoEV', [qnew(1,2:end) qnew(1)], 0.001, 1);
            Mnew = makehgtform('axisrotate', axang(1:3), axang(4)*pi/180);
            set(t,'Matrix', Mnew);
            M = Mnew;
            q = qnew;
            drawnow limitrate % display updates
        end
    end
end