
% This script will decode an NMEA file. START.

clear all; close all; clc %Clears all
set(0,'defaultfigureWindowStyle','docked'); %Resets MATLAB Windows Style

RefLlh=[39.287714139662370,-81.936696946141353,2.963494235790734e+002]; % GPS average, used this is relative location, exact location would have to be surveyed.
%RefLlh=[39.28763,-82.06325,2.963494235790734e+002]; % from Google Maps
%RefLlh=[39.287709,-82.063313,2.963494235790734e+002]; % from Bing Maps

if 0 % 0 means do not read the text file
  
  Lines=inf;
  fid=fopen('putty_112512_24hrs.log'); %this line opens the file and gets a 'handle' to it. The handle is 'fid'

  
  m=1; % m counts lines in the file
  k=1; % k counts $GPGGA messages
  
  Time=[];
  Lat=[];
  Lon=[];
  while 1
    tline = fgetl(fid);
    if ~ischar(tline) % break when tline is not a character
      break;
    end
    %disp(tline) % display the line
    
    %[Cmd a]=sscanf(tline,'$%s,%s,');
    %Cmd=sscanf(tline,'$%s,%s,');
    
    % your task:
    % use 'doc sscanf' to get only the first type string from the line
    % i.e. GPGGA or GPVTG
    
    Cmd=textscan(tline,'%s',1,'delimiter',','); % get the first delimited word
    
    if strcmpi(Cmd{1},'$GPGGA') % check two strings
      %disp(m)
      %disp(tline) % display the line
      DatT=textscan(tline,'%s',10,'delimiter',',');
      Dat=DatT{1};
      % TO DO: decode time here, use variable 'Time'
      
      % decode time. hhmmss.ss
      Time(k) =str2double(Dat{2}(1:2))+...
        (str2double(Dat{2}(3:4))/60)+...
        (str2double(Dat{2}(5:length(Dat{2})))/3600); %#ok<AGROW>
      
      % decode latitude ddmm.mmmmm
      if strcmpi(Dat{4},'N')
        Lat(k)=str2double(Dat{3}(1:2))+((str2double(Dat{3}(3:length(Dat{3}))))/60); %#ok<AGROW>
      elseif strcmpi(Dat{4},'S')
        %Lat(k)=-1*str2double(Dat{3});
        Lat(k)=-1*str2double(Dat{3}(1:2))+((str2double(Dat{3}(3:length(Dat{3}))))/60); %#ok<AGROW>
      else
        error('Latitude must be N or S');
      end
      
      % decode Longitude. dddmm.mmmmm
      if strcmpi(Dat{6},'E')
        Lon(k)=   str2double(Dat{5}(1:3))+((str2double(Dat{5}(4:length(Dat{5}))))/60); %#ok<AGROW>
      elseif strcmpi(Dat{6},'W')
        Lon(k)=-1*str2double(Dat{5}(1:3))+((str2double(Dat{5}(4:length(Dat{5}))))/60); %#ok<AGROW>
      else
        error('Latitude must be W or E');
      end
      
      % decode quality indicator
      Qual(k)=   str2double(Dat{7}); %#ok<AGROW>
      
      % decode NumSat
      NumSat(k)=   str2double(Dat{8}); %#ok<AGROW>
      
      % decode HDOP
      Hdop(k)=   str2double(Dat{9}); %#ok<AGROW>
      
      % decode Height in meters
      Height(k)=   str2double(Dat{10}); %#ok<AGROW>
      
      k=k+1; % go to next index
      
    end
    
    
    if m==Lines
      break;
    end
    m=m+1;
  end
  fclose(fid); %close the file that we opened
  
  save('putty_112512_24hrs.mat', 'Time', 'Lat', 'Lon', 'Qual', 'NumSat', 'Hdop', 'Height');
else
  load('putty_112512_24hrs.mat')
end


% Take out 24-hour wraps in Time
A=cumsum(diff([0 Time])<0);
Time=Time+(24.*A);


% convert LLH to ENU with respect to Reference position
Deg2Rad=pi/180;
RefLlh=[Deg2Rad*RefLlh(1:2),RefLlh(3)].';
RefEce=llh2ecef(RefLlh);

for k=1:size(Time,2)
  Enu=ECEF2ENU(llh2ecef([Deg2Rad*Lat(k);Deg2Rad*Lon(k);Height(k)]),RefEce,RefLlh);
  East(k)=Enu(1,1); %#ok<AGROW>
  North(k)=Enu(2,1); %#ok<AGROW>
  Up(k)=Enu(3,1); %#ok<AGROW>
end


LineW=2;
fignum=0;

% plot Time
fignum=fignum+1;
figure(fignum)
plot(Time)


% plot Latitude
fignum=fignum+1;
figure(fignum)
plot(Time,Lat)
title('Latitude vs. Time')
xlabel('time');
ylabel('Latitude [deg]')

% plot Longitude
fignum=fignum+1;
figure(fignum)
plot(Time,Lon)
title('Longitude vs. Time')
xlabel('time');
ylabel('Longitude [deg]')

% Plot WGS84 Height
fignum=fignum+1;
figure(fignum)
plot(Time,Height)
title('Height vs. Time')
xlabel('time');
ylabel('Height [meters]')

% plot East
fignum=fignum+1;
figure(fignum)
plot(Time,East)
title('East Error vs. Time')
xlabel('time');
ylabel('East Error [meters]')

% plot North
fignum=fignum+1;
figure(fignum)
plot(Time,North)
title('North Error vs. Time')
xlabel('time');
ylabel('North Error [meters]')

% plot Up
fignum=fignum+1;
figure(fignum)
plot(Time,Up)
title('Up Error vs. Time')
xlabel('time');
ylabel('Up Error [meters]')

% plot number of satellites
fignum=fignum+1;
figure(fignum)
plot(Time,NumSat)
title('Number of Satellites Tracked')
xlabel('time');
ylabel('# of satellites')

% plot HDOP
fignum=fignum+1;
figure(fignum)
hold on
plot(Time,Hdop)
%plot(Time,NumSat,'r.')
hold off
title('HDOP versus Time')
xlabel('time');
ylabel('HDOP')


% Plot ENU in a 3x1 plot
fignum=fignum+1;
figure(fignum)
subplot(3,1,1)
plot(Time,East,'-r','LineWidth',LineW,'LineStyle','-');
title(sprintf('Absolute East Error (Bing Maps). Mean:%.2f m, Std:%.2f m',mean(East),std(East)))
%title(sprintf('East Relative Error. Mean:%.2f m, Std:%.2f m',mean(East),std(East)))
xlabel('time [hours]');
ylabel('East Error [meters]')
%ylim([-10 10])
grid on

subplot(3,1,2)
plot(Time,North,'-g','LineWidth',LineW,'LineStyle','-');
title(sprintf('Absolute North Error (Bing Maps). Mean:%.2f m, Std:%.2f m',mean(North),std(North)))
%title(sprintf('North Relative Error. Mean:%.2f m, Std:%.2f m',mean(North),std(North)))
xlabel('time [hours]');
ylabel('North Error [meters]')
%ylim([-10 10])
grid on

subplot(3,1,3)
plot(Time,Up,'-b','LineWidth',LineW,'LineStyle','-');
title(sprintf('Absolute Vertical Error (Bing Maps). Mean:%.2f m, Std:%.2f m',mean(Up),std(Up)))
%title(sprintf('Vertical Relative Error. Mean:%.2f m, Std:%.2f m',mean(Up),std(Up)))
xlabel('time [hours]');
ylabel('Vertical Error [meters]')
%ylim([-10 10])
grid on



% Plot ENU, HDOP, #SV in a 3x1 plot
fignum=fignum+1;
figure(fignum)
subplot(3,1,1)
hold on
plot(Time,East,'-r','LineWidth',LineW,'LineStyle','-');
plot(Time,North,'-g','LineWidth',LineW,'LineStyle','-');
plot(Time,Up,'-b','LineWidth',LineW,'LineStyle','-');
hold off
title(sprintf('Relative ENU Errors'))
xlabel('time [hours]');
ylabel('[meters]')
ylim([-10 10])
grid on
%legend('East','North','Vertical','location','best');
%Weather
subplot(3,1,2)
plot(Time,Hdop,'-k','LineWidth',LineW,'LineStyle','-');
title('Horizontal Dilution of Precision (HDOP)')
xlabel('time [hours]');
ylabel('HDOP')
grid on
%ylim([-10 10])

subplot(3,1,3)
plot(Time,NumSat,'r*')
title('Number of Satellites Tracked')
xlabel('time [hours]');
ylabel('# of Satellites')
ylim([4 12])
grid on

%STOP!
