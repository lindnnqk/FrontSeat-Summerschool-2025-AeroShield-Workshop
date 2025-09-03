%% Create Reader and let it run, data is being collected asynchronously
clc; clear all;
% Create reader on COM-Port (Windows)
reader = SerialReader("COM12", 115200); % TODO: Choose correct COM-Port

% Or on Linux/Mac
% reader = SerialReader("/dev/ttyUSB0", 115200);

%% Stop reading
reader.stop();

%% Access collected data
disp(reader.data);
collecteddata = reader.data;

%% Clean up
delete(reader);

%% Plot
figure
plot(collecteddata(:,1))

%% Save collected data
save('CollectedData.mat','collecteddata') % TODO: Edit file name