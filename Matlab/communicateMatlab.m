% t = tcpip('192.168.0.1', 2000, 'NetworkRole', 'server');
r = tcpip('192.168.0.1', 2000, 'NetworkRole', 'server');
fopen(r);
disp('opened, waiting for data from s7');
s7Reques = fread(r,1)
disp('readed, waiting for transfer to s7');
data2S7 = [3.6, 3.5, 3.4];
fwrite(r, data2S7,'float64');