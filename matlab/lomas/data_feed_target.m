%% 
instrumentObjects = instrfind; % don't pass it anything - find all of them.
delete(instrumentObjects);

clc
clear all
%% 


%% 
% All the serial port configuration here
baud_rate = 115200;
com_port = 'COM19';

log_file_path = '';
%% 


% ----------------------------------------------------
targetUart = serial(com_port, 'BaudRate', baud_rate);
fopen(targetUart);

% load('data_for_controller_test.mat');
load('test_cnt_data.mat')
[maxItr, ~] = size(cnt_data);

% fprintf(targetUart, 'This is a test string !');

% size_of_floats_being_sent = 4;
% fwrite(targetUart, size_of_floats_being_sent, 'int8');
fwrite(targetUart, cnt_data(5, 1:22),'float32');
fprintf(targetUart, '\n');

A = fread(targetUart, 4, 'float32');
disp(A)

% fclose(targetUart);
    
% for i = 1:maxItr
%     fwrite(targetUart, cnt_data(1),'float32');
%     fprintf(targetUart, '\n');
% %     A = fread(targetUart, 1);
% %     disp(A);
% %     a = fscanf(targetUart);
% %     disp(a);
% end

