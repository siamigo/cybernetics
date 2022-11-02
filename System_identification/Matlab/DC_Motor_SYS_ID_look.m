%% System identification of DC motor with measurements from Arduino
% By Øystein Bjelland, IIR, NTNU

clear;
clc;
close all;

%% Import file into Matlab. File should be in txt-format.
filename = 'Data_DC_motor_2.txt'; %File must be located in same folder

A = importdata('Test_data_1.txt');
B = importdata('Test_data_2.txt');
C = importdata('Test_data_3.txt');
D = importdata('Test_data_4.txt');
%% We need to rearrange the data on the form iddata(output,input,sampling time)

inputVoltage_raw = D(:,2);  %Input voltage [mV]
outputShaftAngle_raw = D(:,3);   %Output shaft angle [deg]
time_raw = D(:,4);  %Raw time from millis() in Arduino [milliseconds]

inputVoltage = inputVoltage_raw / 1000; % Converting to volt
outputShaftAngle = abs(outputShaftAngle_raw); %Taking absolute value
time = zeros(length(time_raw),1);
Ts_vect = [];

%% First, we check the sampling time

for i = 2:length(time_raw)
  
   Ts = time_raw(i) - time_raw(i-1);
   Ts_vect = [Ts_vect, Ts];
   
   time(i) = (time_raw(i) - time_raw(1))/1000; %Starting the time vector from zero and converting from ms to s.
   
end

maxTs = max(Ts_vect);
disp('Maximum sample time [ms]: ')
disp(maxTs)

minTs = min(Ts_vect);
disp('Minimum sample time [ms]: ') 
disp(minTs)

Ts_average = mean(Ts_vect);
disp('The average sample time is [ms]')
disp(Ts_average)

disp('Our sampling time is, Ts [sec]')
Ts = round(Ts_average)*10^-3;
disp(Ts)

%% Now, lets structure our data as iddata

DC_motor_DATA = iddata(outputShaftAngle, inputVoltage, Ts);

%% Visualize the data before starting system identification

figure(1)

subplot(2,1,1)
plot(time, inputVoltage)
grid on
xlabel('time [s]')
ylabel('Voltage [V]')

subplot(2,1,2)
plot(time, outputShaftAngle)
grid on
xlabel('time [s]')
ylabel('Shaft Angle [deg]')

%% Going into system identification

Gp = tfest(DC_motor_DATA, 2, 0) %Estimating our system as a transfer function with no zeros and two poles.

% Alternatively go into System Identification app and try other model structures: 
%systemIdentification

%% Now, let's try to tune a PID using our estimated system as a digital twin.

% First, we define our feedback transfer function, H
H = [1];

% For reference, we define an open loop system (uncontrolled) and have a look at the
% response
figure(2)
M = feedback(Gp, H);
step(M)
hold on
legend('Uncontrolled system')

%% We then define our PID

Kp = 0.007485 %0.003687; %Proportional
Ki = 0.01656 %0.0006871;  %Integral
Kd = 0.0001093 %0.004946; %Differential

% Our control transfer function is
Gc = pid(Kp, Ki, Kd);

% And we can define our controlled system
Mc = feedback(Gc*Gp, H);

step(Mc)
grid on
legend('controlled system')

%After running, try to change the three parameters Kp, Ki and Kd by marking the value -->
%right clicking --> select 'increment value and run section'

% Finally, try to run the PID controller on the Arduino :)



