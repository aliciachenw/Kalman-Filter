% Adaptive Kalman Filter for MEMS-IMU based Attitude Estimation under External Acceleration and 
% Parsimonious use of Gyroscopes

clear all;close all;
%% 加载数据
% 1:左腿Yes 2:左腿IMU 3:右腿Yes 4:右腿IMU
% Ch1=1yaw Ch2=1pitch Ch3=1roll Ch4=1AccX Ch5=1AccY Ch6=1AccZ Ch7=1GyroX Ch8=1GyroY Ch9=1GyroZ
% Ch10=1MagX Ch11=1MagY Ch12=1MagZ Ch13=1qaut0 Ch14=1qaut1 Ch15=1qaut2 Ch16=1qaut3 
% Ch17=2yaw Ch18=2pitch Ch19=2roll Ch20=2AccX Ch21=2AccY Ch22=2AccZ Ch23=2GyroX Ch24=2GyroY Ch25=2GyroZ
% Ch26=2MagX Ch27=2MagY Ch28=2MagZ Ch29=2qaut0 Ch30=2qaut1 Ch31=2qaut2 Ch32=2qaut3
% Ch33=3yaw Ch34=3pitch Ch35=3roll Ch36=3AccX Ch37=3AccY Ch38=3AccZ Ch39=3GyroX Ch40=3GyroY Ch41=3GyroZ
% Ch42=3MagX Ch43=3MagY Ch44=3MagZ Ch45=3qaut0 Ch46=3qaut1 Ch47=3qaut2 Ch48=3qaut3
% Ch49=4yaw Ch50=4pitch Ch51=4roll Ch52=4AccX Ch53=4AccY Ch54=4AccZ Ch55=4GyroX Ch56=4GyroY Ch57=4GyroZ
% Ch58=4MagX Ch59=4MagY Ch60=4MagZ Ch61=4qaut0 Ch62=4qaut1 Ch63=4qaut2 Ch64=4qaut3

% D:\matlab_program\Ski IMU System\DATA\20190122 zm wjs lyy lky lgl hyp
path='D:\matlab_program\Ski IMU System\DATA\20190122\';
subject={'zm','wjs','lyy','lky','lgl','hyp'};
subject_num=3;
subject_file=[path,subject{subject_num},'.txt'];
AllData=load(subject_file);
if ismember(subject_num,[4,5])
    trans_leg='left';
else
    trans_leg='right';
end

g=9.80665;%重力加速度
dip=1.0329*pi/180;
vec_g=[0;0;-g];
vec_m=[cos(dip);0;-sin(dip)];
frame=size(AllData,1);

%% Yesense 左腿
Ts=1/100;
Yes_acc_filted=AllData(:,4:6);
Yes_gyro_filted=AllData(:,7:9);
Yes_mag_filted=AllData(:,10:12);
Yes_t=1:size(Yes_mag_filted,1);Yes_t=Yes_t.*Ts;

%% IMU
Ts=0.01;
IMU_acc_filted=AllData(:,20:22);IMU_acc_filted(:,1:2)=-IMU_acc_filted(:,1:2);
IMU_gyro_filted=AllData(:,23:25);IMU_gyro_filted(:,1:2)=-IMU_gyro_filted(:,1:2);
IMU_mag_filted=AllData(:,26:28);
IMU_t=1:size(IMU_mag_filted,1);IMU_t=IMU_t.*Ts;

%% 原始数据比较
% figure;
% subplot(3,1,1);
% plot(Yes_t,Yes_mag_filted(:,1));hold on;plot(IMU_t,IMU_mag_filted(:,1));
% subplot(3,1,2);
% plot(Yes_t,Yes_mag_filted(:,2));hold on;plot(IMU_t,IMU_mag_filted(:,2));
% subplot(3,1,3);
% plot(Yes_t,Yes_mag_filted(:,3));hold on;plot(IMU_t,IMU_mag_filted(:,3));
% figure;
% subplot(3,1,1);
% plot(Yes_t,Yes_gyro_filted(:,1));hold on;plot(IMU_t,IMU_gyro_filted(:,1));
% subplot(3,1,2);
% plot(Yes_t,Yes_gyro_filted(:,2));hold on;plot(IMU_t,IMU_gyro_filted(:,2));
% subplot(3,1,3);
% plot(Yes_t,Yes_gyro_filted(:,3));hold on;plot(IMU_t,IMU_gyro_filted(:,3));
% figure;
% subplot(3,1,1);
% plot(Yes_t,Yes_acc_filted(:,1));hold on;plot(IMU_t,IMU_acc_filted(:,1));
% subplot(3,1,2);
% plot(Yes_t,Yes_acc_filted(:,2));hold on;plot(IMU_t,IMU_acc_filted(:,2));
% subplot(3,1,3);
% plot(Yes_t,Yes_acc_filted(:,3));hold on;plot(IMU_t,IMU_acc_filted(:,3));

%% 标定
acc_err=[0,0,0];mag_err=[0,0,0];gyro_err=[0,0,0];
for i=1:size(Yes_t,2)
    acc_err=acc_err+Yes_acc_filted(i,:)-IMU_acc_filted(i,:);
    mag_err=mag_err+Yes_mag_filted(i,:)-IMU_mag_filted(i,:);
    gyro_err=gyro_err+Yes_gyro_filted(i,:)-IMU_gyro_filted(i,:);
end
acc_err=acc_err./size(Yes_t,2);mag_err=mag_err./size(Yes_t,2);gyro_err=gyro_err./size(Yes_t,2);
for i=1:size(IMU_t,2)
    IMU_acc_filted(i,:)=IMU_acc_filted(i,:)+acc_err;
    IMU_gyro_filted(i,:)=IMU_gyro_filted(i,:)+gyro_err;
    IMU_mag_filted(i,:)=IMU_mag_filted(i,:)+mag_err;
end

% figure;
% subplot(3,1,1);
% plot(Yes_t,Yes_mag_filted(:,1));hold on;plot(IMU_t,IMU_mag_filted(:,1));
% subplot(3,1,2);
% plot(Yes_t,Yes_mag_filted(:,2));hold on;plot(IMU_t,IMU_mag_filted(:,2));
% subplot(3,1,3);
% plot(Yes_t,Yes_mag_filted(:,3));hold on;plot(IMU_t,IMU_mag_filted(:,3));
% figure;
% subplot(3,1,1);
% plot(Yes_t,Yes_gyro_filted(:,1));hold on;plot(IMU_t,IMU_gyro_filted(:,1));
% subplot(3,1,2);
% plot(Yes_t,Yes_gyro_filted(:,2));hold on;plot(IMU_t,IMU_gyro_filted(:,2));
% subplot(3,1,3);
% plot(Yes_t,Yes_gyro_filted(:,3));hold on;plot(IMU_t,IMU_gyro_filted(:,3));
% figure;
% subplot(3,1,1);
% plot(Yes_t,Yes_acc_filted(:,1));hold on;plot(IMU_t,IMU_acc_filted(:,1));
% subplot(3,1,2);
% plot(Yes_t,Yes_acc_filted(:,2));hold on;plot(IMU_t,IMU_acc_filted(:,2));
% subplot(3,1,3);
% plot(Yes_t,Yes_acc_filted(:,3));hold on;plot(IMU_t,IMU_acc_filted(:,3));

%% 原始信号滤波
%figure;
ap=0.1;as=100;wp=10;ws=45;fs=100;N=size(IMU_acc_filted,1);
n=0:N-1;t=n/fs;
wpp=wp/(fs/2);wss=ws/(fs/2);
[n,wn]=buttord(wpp,wss,ap,as);
[b,a]=butter(n,wn);
%freqz(b,a);
IMU_acc_filted=filter(b,a,IMU_acc_filted);
IMU_gyro_filted=filter(b,a,IMU_gyro_filted);
IMU_mag_filted=filter(b,a,IMU_mag_filted);

%% Integration
Ts=0.01;%采样时间
eps_gyro=0.001;
IMU_q_KF=zeros(4,size(IMU_acc_filted,1));
IMU_euler_KF=zeros(3,size(IMU_acc_filted,1));

for i=1:size(IMU_acc_filted,1)
    gyro=IMU_gyro_filted(i,:); 
    acc=IMU_acc_filted(i,:);
    mag=IMU_mag_filted(i,:);mag=mag./norm(mag);  
    acc=acc';
    gyro=gyro';gyro=gyro.*(pi/180);
    mag=mag';
    
    Omega=zeros(4,4);
    Omega(1,2)=-gyro(1);
    Omega(1,3)=-gyro(2);
    Omega(1,4)=-gyro(3);
    Omega(2,3)=gyro(3);
    Omega(2,4)=-gyro(2);
    Omega(3,4)=gyro(1);
    Omega(2,1)=-Omega(1,2);Omega(3,1)=-Omega(1,3);Omega(3,2)=-Omega(2,3);Omega(4,1)=-Omega(1,4);Omega(4,2)=-Omega(2,4);Omega(4,3)=-Omega(3,4);

    if i==1
%         pitch=asin(acc(1)/g);
%         roll=atan(acc(1)/acc(2));
%         yaw=atan((mag(2)*cos(roll)-mag(1)*sin(roll))/(mag(1)*cos(pitch)+mag(2)*sin(pitch)*sin(roll)+mag(3)*sin(pitch)*cos(roll)));
        yaw=AllData(1,1);
        pitch=AllData(1,2);
        roll=AllData(1,3);
        Xpos=Euler2Qua(yaw/180*pi,pitch/180*pi,roll/180*pi);
        Omega_last=Omega;
    else
%         if norm(gyro)<eps_gyro
%             Omega=eye(4)+Omega.*(Ts/2);
%         else
%             Omega=eye(4).*cos(norm(gyro)/2*Ts)+Omega.*(sin(norm(gyro)/2*Ts)/norm(gyro));
%         end
        Int=eye(4)+Omega.*(Ts/2)+(Omega*Omega_last-Omega_last*Omega).*(Ts^2/48); % first-order
        Xpos=Int*Xpos;
        Omega_last=Omega;
        Xpos=Xpos./norm(Xpos);
    end
    IMU_q_KF(:,i)=Xpos;
    IMU_euler_KF(:,i)=Qua2Euler(Xpos(1),Xpos(2),Xpos(3),Xpos(4));    
end
figure;
subplot(2,2,1);plot(AllData(:,13));hold on;plot(IMU_q_KF(1,:));
subplot(2,2,2);plot(AllData(:,14));hold on;plot(IMU_q_KF(2,:));
subplot(2,2,3);plot(AllData(:,15));hold on;plot(IMU_q_KF(3,:));
subplot(2,2,4);plot(AllData(:,16));hold on;plot(IMU_q_KF(4,:));

figure;
subplot(3,1,1);plot(AllData(:,1));hold on;plot(IMU_euler_KF(1,:)); %yaw
subplot(3,1,2);plot(AllData(:,2));hold on;plot(IMU_euler_KF(2,:)); %pitch
subplot(3,1,3);plot(AllData(:,3));hold on;plot(IMU_euler_KF(3,:)); %roll

% Yes_euler_cal=zeros(3,frame);
% for i=1:frame
%     Yes_euler_cal(:,i)=Qua2Euler(AllData(i,13),AllData(i,14),AllData(i,15),AllData(i,16));
% end
% 
% figure;
% subplot(3,1,1);plot(AllData(:,1));hold on;plot(Yes_euler_cal(1,:),'--'); %yaw
% subplot(3,1,2);plot(AllData(:,2));hold on;plot(Yes_euler_cal(2,:),'--'); %pitch
% subplot(3,1,3);plot(AllData(:,3));hold on;plot(Yes_euler_cal(3,:),'--'); %roll
% 
% Yes_q_cal=zeros(4,frame);
% for i=1:frame
%     Yes_q_cal(:,i)=Euler2Qua(AllData(i,1)/180*pi,AllData(i,2)/180*pi,AllData(i,3)/180*pi);
% end
% 
% figure;
% subplot(2,2,1);plot(AllData(:,13));hold on;plot(Yes_q_cal(1,:),'--'); 
% subplot(2,2,2);plot(AllData(:,14));hold on;plot(Yes_q_cal(2,:),'--'); 
% subplot(2,2,3);plot(AllData(:,15));hold on;plot(Yes_q_cal(3,:),'--'); 
% subplot(2,2,4);plot(AllData(:,16));hold on;plot(Yes_q_cal(4,:),'--'); 