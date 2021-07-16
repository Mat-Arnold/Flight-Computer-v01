% Teensy Communications Testing

% Connect to serial port
format longG
s = serialport('COM7', 115200);
teensyOut = zeros(10,length(sensorOutputs(1,:)));
ready = 'r';

for j = 1:length(sensorOutputs(1,:))
    data = '';
    for i = 1:15
        data = append(data, num2str(sensorOutputs(i,j),'%.6f'), ',');
    end
    data  = convertCharsToStrings(data);
%     disp(data)
    shake = '';
    
    % check if Teensy is ready to recieve then
    % if ready send data
    while(1)
        if(shake == 'y')
            write(s,data,"string");
%             disp("Data sent");
            break;
        else
            write(s,ready,"char");
            shake = read(s,1,"char");
        end
    end
    pause(0.1);
    for k = 1:12
        out = readline(s);
        if (k == 1)
            disp(out);
        else
            teensyOut(k-1,j) = out;
            disp(out);
        end
    end

end

clear s;