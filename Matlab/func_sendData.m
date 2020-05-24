function [] = func_sendData(target,...
                       pos1, pos2, pos3,...
                       vel1, vel2, vel3,...
                       ramp1, ramp2, ramp3)
    fread(target,1);
    data2S7 = [pos1*factor, pos2*factor, pos3*factor,...
               vel1*factor, vel2*factor, vel3*factor,...
               ramp1, ramp2, ramp3,...
               0.0];
    fwrite(target, data2S7,'float64');
    pause(1)
    data2S7 = [pos1*factor, pos2*factor, pos3*factor,...
               vel1*factor, vel2*factor, vel3*factor,...
               ramp1, ramp2, ramp3,...
               1.0];
    fwrite(target, data2S7,'float64');
end