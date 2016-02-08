function setupDataset(folder)

% % READ THE ACCELEROMETER DATA FILES
% % (accelerometer + gyroscope information)
% base_name = 'mod (';
% numFiles = 5;
% for i=1:1:numFiles
%     dataFile = fopen([folder base_name num2str(i) ').txt'],'r');
%     data = fscanf(dataFile,'%d\t%d\t%d\t%d\t%d\t%d\n',[6,inf]);
%     fclose(dataFile);
%     
%     % add "-1" at the beginning and "1" and the end of each line
%     out_name = 'new_mod (';
%     first = - ones(1,size(data,2));
%     last = ones(1,size(data,2));
%     new_data = cat(1,first,data,last);
%     size(new_data)
%     new_data = new_data';
%     dlmwrite([folder out_name num2str(i) ').txt'],new_data,'\t');
% end

% READ THE ACCELEROMETER DATA FILES
% (accelerometer only information)
base_name = 'mod (';
numFiles = 5;
for i=1:1:numFiles
    dataFile = fopen([folder base_name num2str(i) ').txt'],'r');
    data = fscanf(dataFile,'%d\t%d\t%d\n',[3,inf]);
    fclose(dataFile);
    
    % add "-1" at the beginning and "1" and the end of each line
    % add a matrix of 0 in place of the gyro info
    out_name = 'new_mod (';
    first = - ones(1,size(data,2));
    last = ones(1,size(data,2));
    gyro = zeros(3,size(data,2));
    new_data = cat(1,first,data,gyro,last);
    size(new_data)
    new_data = new_data';
    dlmwrite([folder out_name num2str(i) ').txt'],new_data,'\t');
end