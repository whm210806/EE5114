%% Clear past plots, variables and commands
close all; clear all; clc;

% Load data 
load 'EE5114_CA1.mat';

% acx = x-axis accelerometer reading
% acy = y-axis accelerometer reading
% acz = z-axis accelerometer reading
% 
% phi = Roll angle computed by the drone's on-board computer
% tht = Pitch angle computed by the drone's on-board computer
% psi = Yaw angle computed by the drone's on-board computer 
% 
% fix = GPS position fix signal 
% eph = GPS horizontal variance 
% epv = GPS vertical variance 
% lat = GPS Latitude
% lon = GPS Longitude
% alt = GPS altitude
% gps_nSat = Number of GPS satellites
% 
% out1 = Motor 1 signal
% out2 = Motor 2 signal
% out3 = Motor 3 signal
% out4 = Motor 4 signal

%% Accelerometer plot
figure; set(gcf,'numbertitle','off','name','Acceleration');  
subplot(3,1,1); plot(t, acx, 'b'); ylim([-2 2]); ylabel('acx (m/s^2)'); grid on; 
subplot(3,1,2); plot(t, acy, 'b'); ylim([-2 2]); ylabel('acy (m/s^2)'); grid on; 
subplot(3,1,3); plot(t, acz, 'b'); ylabel('acz (m/s^2)'); grid on; 

%% Euler angles plot
figure; set(gcf,'numbertitle','off','name','Euler Angles');  
subplot(3,1,1); plot(t, rad2deg(phi), 'b'); ylabel('Roll (degree)'); grid on; 
subplot(3,1,2); plot(t, rad2deg(tht), 'b'); ylabel('Pitch (degree)'); grid on; 
subplot(3,1,3); plot(t, rad2deg(psi), 'b'); ylabel('Yaw (degree)'); grid on; 

%% GPS plot
figure; set(gcf,'numbertitle','off','name','GPS');  
subplot(3,2,1); plot(t, lon); ylabel('Longitude'); grid on;
subplot(3,2,3); plot(t, lat); ylabel('Latitude'); grid on;
subplot(3,2,5); plot(t, alt); ylabel('Altitude'); grid on; xlabel('time (s)');

subplot(3,2,2); plot(t, gps_nSat, '.'); ylabel('Sat'); grid on;
subplot(3,2,4); plot(t, eph); ylabel('Eph'); grid on; ylim([0 5]);
subplot(3,2,6); plot(t, epv); ylabel('Epv'); grid on; ylim([0 5]);

figure; set(gcf,'numbertitle','off','name','GPS-1');  
subplot(3,1,1); plot(t(6500:9000,1), lon(6500:9000,1)); ylabel('Longitude'); grid on;
subplot(3,1,2); plot(t(6500:9000,1), lat(6500:9000,1)); ylabel('Latitude'); grid on;
subplot(3,1,3); plot(t(6500:9000,1), alt(6500:9000,1)); ylabel('Altitude'); grid on; xlabel('time (s)');

%% Motor signal plot
figure; set(gcf,'numbertitle','off','name','Motor Signal');  
hold on;
plot(t,out1,'r');
plot(t,out2,'g');
plot(t,out3,'b');
plot(t,out4,'y');
legend('Motor1','Motor2','Motor3','Motor4'); 
ylabel('Motor inputs'); xlabel('time (s)'); ylim([1000 2000]); grid on;


%%%%%%%%%%%%%%%%%%%%%% Your own coding work start from here %%%%%%%%%%%%%%%%%%%%%%%%%

%% Convert GPS raw measurements to local NED position values

e=0.08181919;% the first eccentricity
R_Ea=6378137;%the semi-major axis
f=1/298.257223563;% the flattening factor
R_Eb=R_Ea*(1-f);%he semi-minor axis
M_E=zeros(10215,1);% the meridian radius of curvature
N_E=zeros(10215,1);%the prime vertical radius of curvature
for i=1:10215
    M_E(i,1)=R_Ea*(1-e^2)/(1-e^2*sind(lat(i,1))^2)^(3/2);
    N_E(i,1)=R_Ea/sqrt(1-e^2*sind(lat(i,1))^2);
end
P_g=[lon,lat,alt];
P_e=zeros(10215,3);
for i=1:10215
    P_e(i,1)=(N_E(i,1)+P_g(i,3))*cosd(P_g(i,2))*cosd(P_g(i,1));
    P_e(i,2)=(N_E(i,1)+P_g(i,3))*cosd(P_g(i,2))*sind(P_g(i,1));
    P_e(i,3)=(N_E(i,1)*(1-e^2)+P_g(i,3))*sind(P_g(i,2));
end
P_ref=P_e(1,:);
P_n=zeros(3,10215);
for i=1:10215
    P_n(:,i)=R_ne(P_g(1,1),P_g(1,2))*(P_e(i,:)-P_ref)';
end


%% Implement EKF to estimate NED position and velocity
%Sigmoid Bias
g=10;
ac_NED=zeros(10215,3);
ac=[acx,acy,acz];
x=zeros(18,10215);
P=eye(18,18);
%convert output from accelerometer into acceleration in the local NED frame
for i=1:10215
    ac_NED(i,:)=(Body_NED(phi(i,1),tht(i,1),psi(i,1))*ac(i,:)')'+[0,0,g];
end
x(1:3,3999)=P_n(:,3999);
for i=4000:10215
    x(1:3,3999)=P_n(:,3999);%Initialization of state variables
    dt=t(i,1)-t(i-1,1);
    j=0;
    % Find the takeoff time
    while j<1
        if out1(i,1)>0 && out2(i,1)>0 && out3(i,1)>0 && out4(i,1)>0
            k=i;
            j=j+1;
        else
            break
        end
    end
    % Determine the derivative of the bias function.
    if out1(i,1)>0 && out2(i,1)>0 && out3(i,1)>0 && out4(i,1)>0
        delta1=10*sigmoid_basis(10*(t(i-1,1)-t(k,1)))*(1-sigmoid_basis(10*(t(i-1,1)-t(k,1))));
        delta2=10*sigmoid_basis(10*(t(i-1,1)-t(k,1)-100))*(1-sigmoid_basis(10*(t(i-1,1)-t(k,1)-100)));
        delta3=10*sigmoid_basis(10*(t(i-1,1)-t(k,1)+100))*(1-sigmoid_basis(10*(t(i-1,1)-t(k,1)+100)));
    else
        delta1=0;
        delta2=0;
        delta3=0;
    end
        F=[eye(3,3),dt*eye(3,3),-1/2*dt^2*eye(3,3),-1/6*dt^3*delta1*eye(3,3),-1/6*dt^3*delta2*eye(3,3),-1/6*dt^3*delta3*eye(3,3);
        zeros(3,3),eye(3,3),-dt*eye(3,3),-1/2*dt^2*delta1*eye(3,3),-1/2*dt^2*delta2*eye(3,3),-1/2*dt^2*delta3*eye(3,3);
        zeros(3,3),zeros(3,3),eye(3,3),dt*delta1*eye(3,3),dt*delta2*eye(3,3),dt*delta3*eye(3,3);
        zeros(3,3),zeros(3,3),zeros(3,3),eye(3,3),zeros(3,6);
        zeros(3,12),eye(3,3),zeros(3,3);
        zeros(3,15),eye(3,3)];
        F(1:6,:)=F(1:6,:)*[eye(6,18);
        zeros(3,6),Body_NED(phi(i,1),tht(i,1),psi(i,1)),zeros(3,9);
        zeros(3,9),Body_NED(phi(i,1),tht(i,1),psi(i,1)),zeros(3,6);
        zeros(3,12),Body_NED(phi(i,1),tht(i,1),psi(i,1)),zeros(3,3);
        zeros(3,15),Body_NED(phi(i,1),tht(i,1),psi(i,1));];
    G=[1/2*dt^2*eye(3,3);
        dt*eye(3,3);
        zeros(3,3);
        zeros(9,3)];
    H=eye(3,18);
    Q=[1.5*rand(1,1);1.5*rand(1,1);0.5*rand(1,1)].*eye(3,3);
    R=0.1*rand(3,1).*eye(3,3);
    x(:,i)=F*x(:,i-1)+G*[ac_NED(i,:)]';
%     x(:,i)=F*x(:,i-1)+G*[ac_NED(i-1,:),ac_NED(i,:)]';
    P_k=F*P*F'+G*Q*G';
    K=P_k*H'/(H*P_k*H'+R);
    y_k=[P_n(:,i);zeros(3,1);zeros(3,1);zeros(3,1)];
    x(:,i)=x(:,i)+K*(P_n(:,i)-H*x(:,i));
    P_k=P_k-K*H*P_k;
    P=P_k;
    bias(1:3,i)=x(7:9,i)+delta1*x(10:12,i)*dt+delta2*x(13:15,i)*dt+delta3*x(16:18,i)*dt;
end

% constant bias
g=10;%gravity
ac_NED=zeros(10215,3);
ac=[acx,acy,acz];
x1=zeros(9,10215);
P=eye(9,9);
%convert output from accelerometer into acceleration in the local NED frame
for i=1:10215
    ac_NED(i,:)=(Body_NED(phi(i,1),tht(i,1),psi(i,1))*ac(i,:)')'+[0,0,g];
end
x1(1:3,3999)=P_n(:,3999);
for i=4000:10215
    %Initialization of state variables
    x1(1:3,3999)=P_n(:,3999);
    dt=t(i,1)-t(i-1,1);
    F=[eye(3,3),dt*eye(3,3),-1/2*dt^2*eye(3,3);
    zeros(3,3),eye(3,3),-dt*eye(3,3);
    zeros(3,3),zeros(3,3),eye(3,3);];
    F(1:6,:)=F(1:6,:)*[eye(6,9);
    zeros(3,6),Body_NED(phi(i,1),tht(i,1),psi(i,1))];
    G=[1/2*dt^2*eye(3,3);
        dt*eye(3,3);
        zeros(3,3);];
    H=eye(3,9);
    Q=[0.5*rand(1,1);1*rand(1,1);1*rand(1,1)].*eye(3,3);
    R=0.05*rand(3,1).*eye(3,3);
    x1(:,i)=F*x1(:,i-1)+G*[ac_NED(i,:)]';
%     x(:,i)=F*x(:,i-1)+G*[ac_NED(i-1,:),ac_NED(i,:)]';
    P_k=F*P*F'+G*Q*G';
    K=P_k*H'/(H*P_k*H'+R);
    x1(:,i)=x1(:,i)+K*(P_n(:,i)-H*x1(:,i));
    P_k=P_k-K*H*P_k;
    P=P_k;
end
bias1=x1(7:9,:);
for i=2:10215
    dt=t(i,1)-t(i-1,1);
    bias(1:3,i)=x(7:9,i)+[1000;1e4;10].*x(10:12,i)*dt;
end
%% Result plots
figure; set(gcf,'numbertitle','off','name','Position');  
subplot(3,1,1); plot(t(4000:10000,1), x(1,4000:10000), 'b','LineWidth',1);  ylabel('x (m)'); grid on;hold on 
subplot(3,1,2); plot(t(4000:10000,1), x(2,4000:10000), 'b','LineWidth',1);  ylabel('y (m)'); grid on; hold on
subplot(3,1,3); plot(t(4000:10000,1), x(3,4000:10000), 'b','LineWidth',1); ylabel('z (m)'); grid on;hold on
subplot(3,1,1); plot(t(4000:10000,1), x1(1,4000:10000), 'r');  ylabel('x (m)'); grid on; hold on
subplot(3,1,2); plot(t(4000:10000,1), x1(2,4000:10000), 'r');  ylabel('y (m)'); grid on; hold on
subplot(3,1,3); plot(t(4000:10000,1), x1(3,4000:10000), 'r'); ylabel('z (m)'); grid on;hold on
legend('Sigmoid Bias','Constant Bias','Location','southeast'); 

figure; set(gcf,'numbertitle','off','name','V_n');  
subplot(3,1,1); plot(t(4000:10000,1), x(4,4000:10000), 'b');  ylabel('v_x (m/s)'); grid on;hold on 
subplot(3,1,2); plot(t(4000:10000,1), x(5,4000:10000), 'b');  ylabel('v_y (m/s)'); grid on; hold on
subplot(3,1,3); plot(t(4000:10000,1), x(6,4000:10000), 'b'); ylabel('v_z (m/s)'); grid on;hold on
subplot(3,1,1); plot(t(4000:10000,1), x1(4,4000:10000), 'r');  ylabel('v_x (m/s)'); grid on; hold on
subplot(3,1,2); plot(t(4000:10000,1), x1(5,4000:10000), 'r');  ylabel('v_y (m/s)'); grid on; hold on
subplot(3,1,3); plot(t(4000:10000,1), x1(6,4000:10000), 'r'); ylabel('v_z (m/s)'); grid on;hold on
legend('Sigmoid Bias','Constant Bias','Location','southeast');  

figure; set(gcf,'numbertitle','off','name','bias');  
subplot(3,1,1); plot(t(4000:10215,1), bias(1,4000:10215), 'b','LineWidth',1);  ylabel('b_x (m/s^2)'); grid on; hold on
subplot(3,1,2); plot(t(4000:10215,1), bias(2,4000:10215), 'b','LineWidth',1);  ylabel('b_y (m/s^2)'); grid on; hold on
subplot(3,1,3); plot(t(4000:10215,1), bias(3,4000:10215), 'b','LineWidth',1); ylabel('b_z (m/s^2)'); grid on;hold on
subplot(3,1,1); plot(t(4000:10215,1), bias1(1,4000:10215), 'r');  ylabel('b_x (m/s^2)'); grid on; hold on
subplot(3,1,2); plot(t(4000:10215,1), bias1(2,4000:10215), 'r');  ylabel('b_y (m/s^2)'); grid on; hold on
subplot(3,1,3); plot(t(4000:10215,1), bias1(3,4000:10215), 'r'); ylabel('b_z (m/s^2)'); grid on;hold on
legend('Sigmoid Bias','Constant Bias','Location','northeast');


