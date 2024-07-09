address = '172.23.42.235';
port = 4560;
t = tcpclient(address, port);

% Send a test message
write(t, uint8('Hello, PX4!'));

% Receive a response
response = read(t);
disp(char(response));
