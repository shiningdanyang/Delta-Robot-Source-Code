function [target] = func_initCommunicate()
    target = tcpip('192.168.0.1', 2000, 'NetworkRole', 'server');
    fopen(target);
    disp('opened, waiting for data from s7');
    pause(1);
    data2S7 = [0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0];
    fwrite(target, data2S7,'float64');
    pause(0.5)
    data2S7 = [0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 1.0];
    fwrite(target, data2S7,'float64');
end