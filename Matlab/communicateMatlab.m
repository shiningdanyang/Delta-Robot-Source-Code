% t = tcpip('192.168.0.1', 2000, 'NetworkRole', 'server');
r = tcpip('192.168.0.1', 2000, 'NetworkRole', 'server');
fopen(r);
disp('opened, waiting for data from s7');
pause(1);

fread(r,1)
data2S7 = [3.6, 3.5, 3.4, 0.0];
fwrite(r, data2S7,'float64');
pause(0.5)
data2S7 = [30.6, 30.5, 30.4, 1.0];
fwrite(r, data2S7,'float64');

fread(r,1)
data2S7 = [4.6, 4.5, 4.4, 0.0];
fwrite(r, data2S7,'float64');
pause(0.5)
data2S7 = [40.6, 40.5, 40.4, 1.0];
fwrite(r, data2S7,'float64');

fread(r,1)
data2S7 = [5.6, 5.5, 5.4, 0.0];
fwrite(r, data2S7,'float64');   
pause(0.5)
data2S7 = [50.6, 50.5, 50.4, 1.0];
fwrite(r, data2S7,'float64');