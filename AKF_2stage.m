function [all_state,hh] = AKF_2stage(data, stage, initial_euler)
% data格式统一为：
% Ch1=1yaw Ch2=1pitch Ch3=1roll Ch4=1AccX Ch5=1AccY Ch6=1AccZ Ch7=1GyroX Ch8=1GyroY Ch9=1GyroZ
% Ch10=1MagX Ch11=1MagY Ch12=1MagZ Ch13=1qaut0 Ch14=1qaut1 Ch15=1qaut2 Ch16=1qaut3 

% acc mag全部角度都纠正
% siga = 0.001;
% sigm = 0.001;
% sigq= 0.000001;
% ka = 1000;
% km = 10000;

siga = 0.00001;
sigm = 0.00001;
sigq= 0.0000001;
ka = 100;
km = 10000;
% walk right: ka=100 km = 100000

Ts = 0.01;%采样时间
g = 9.80665;%重力加速度
all_state = zeros(4,size(data,1));
vec_g = [0;0;g];

hh = zeros(3,size(data,1));

for i=1:size(data,1)
    acc = data(i,4:6);
    gyro = data(i,7:9); 
    mag = data(i,10:12);
    mag = mag./norm(mag);  
    acc=acc';
    gyro=gyro';
    gyro = gyro.*(pi/180);
    mag = mag';
    
    Omega = AngRate2Rot(gyro);

    if i==1
        yaw = initial_euler(1,1);
        pitch = initial_euler(1,2);
        roll = initial_euler(1,3);
        Xpos = Euler2Qua(yaw,pitch,roll);
        Omega_last = Omega;
        Ppos = eye(4).*0.001+ones(4,4).*0.0005;
        mag_r=Qua2Mat(Xpos(1),Xpos(2),Xpos(3),Xpos(4))^(-1)*mag;
        acc_r=Qua2Mat(Xpos(1),Xpos(2),Xpos(3),Xpos(4))*acc;
%         dip=mag_r'*acc_r/(norm(acc)*norm(mag));
%         vec_m=[cos(dip);0;-sin(dip)];
        vec_m = mag_r;
        dip = acos(mag_r(2));
    else
        Int = eye(4)+Omega.*(Ts/2)+(Omega*Omega_last-Omega_last*Omega).*(Ts^2/48); % first-order
        Int = eye(4)+Omega.*(Ts/2);
        Omega_last = Omega;
        Xpri = Int * Xpos;
        Q = eye(4,4).*sigq;
        Ppri = Int*Ppos*Int'+Q;

        %% first stage: update pitch & roll from acc
        H1 = [-2*Xpri(3) 2*Xpri(4) -2*Xpri(1) 2*Xpri(2);
            2*Xpri(2) 2*Xpri(1) 2*Xpri(4) 2*Xpri(3);
            2*Xpri(1) -2*Xpri(2) -2*Xpri(3) 2*Xpri(4)];
        H1 = H1.*(1); 
        R1 = eye(3).*(ka*abs(norm(acc)-g)) + eye(3).*siga;
        K1 = Ppri*H1'/(H1*Ppri*H1'+R1);
        h1 = Qua2Mat(Xpri(1),Xpri(2),Xpri(3),Xpri(4))*vec_g;
        hh(:,i) = h1;
        Xe1 = K1*(acc-h1);
        % Xe1(4) = 0;
        X1 = Xpri+Xe1;
        P1 = (eye(4)-K1*H1)*Ppri;
        
        %% second stage: update yaw from mag
%         H2 = [2*Xpri(4) 2*Xpri(3) 2*Xpri(2) 2*Xpri(1);
%             2*Xpri(1) -2*Xpri(2) 2*Xpri(3) -2*Xpri(4);
%             -2*Xpri(2) -2*Xpri(1) 2*Xpri(4) 2*Xpri(3)];
%         H2 = [2*Xpri(1)*vec_m(1)+2*Xpri(4)*vec_m(2)-2*Xpri(3)*vec_m(3), 2*Xpri(2)*vec_m(1)+2*Xpri(3)*vec_m(2)+2*Xpri(4)*vec_m(3), -2*Xpri(3)*vec_m(1)+2*Xpri(2)*vec_m(2)-2*Xpri(1)*vec_m(3), -2*Xpri(4)*vec_m(1)+2*Xpri(1)*vec_m(2)+2*Xpri(2)*vec_m(3);
%             -2*Xpri(4)*vec_m(1)+2*Xpri(1)*vec_m(2)+2*Xpri(2)*vec_m(3), 2*Xpri(3)*vec_m(1)-2*Xpri(2)*vec_m(2)+2*Xpri(1)*vec_m(3), 2*Xpri(2)*vec_m(1)+2*Xpri(3)*vec_m(2)+2*Xpri(4)*vec_m(3), -2*Xpri(1)*vec_m(1)-2*Xpri(4)*vec_m(2)+2*Xpri(3)*vec_m(3);
%             2*Xpri(3)*vec_m(1)-2*Xpri(2)*vec_m(2)+2*Xpri(1)*vec_m(3), 2*Xpri(4)*vec_m(1)-2*Xpri(1)*vec_m(2)-2*Xpri(2)*vec_m(3), 2*Xpri(1)*vec_m(1)+2*Xpri(4)*vec_m(2)-2*Xpri(3)*vec_m(3), 2*Xpri(2)*vec_m(1)+2*Xpri(3)*vec_m(2)+2*Xpri(4)*vec_m(3)];
%         
%         mag_r=Qua2Mat(Xpri(1),Xpri(2),Xpri(3),Xpri(4))^(-1)*mag;
        H2 = [2*X1(1)*vec_m(1)+2*X1(4)*vec_m(2)-2*X1(3)*vec_m(3), 2*X1(2)*vec_m(1)+2*X1(3)*vec_m(2)+2*X1(4)*vec_m(3), -2*X1(3)*vec_m(1)+2*X1(2)*vec_m(2)-2*X1(1)*vec_m(3), -2*X1(4)*vec_m(1)+2*X1(1)*vec_m(2)+2*X1(2)*vec_m(3);
            -2*X1(4)*vec_m(1)+2*X1(1)*vec_m(2)+2*X1(2)*vec_m(3), 2*X1(3)*vec_m(1)-2*X1(2)*vec_m(2)+2*X1(1)*vec_m(3), 2*X1(2)*vec_m(1)+2*X1(3)*vec_m(2)+2*X1(4)*vec_m(3), -2*X1(1)*vec_m(1)-2*X1(4)*vec_m(2)+2*X1(3)*vec_m(3);
            2*X1(3)*vec_m(1)-2*X1(2)*vec_m(2)+2*X1(1)*vec_m(3), 2*X1(4)*vec_m(1)-2*X1(1)*vec_m(2)-2*X1(2)*vec_m(3), 2*X1(1)*vec_m(1)+2*X1(4)*vec_m(2)-2*X1(3)*vec_m(3), 2*X1(2)*vec_m(1)+2*X1(3)*vec_m(2)+2*X1(4)*vec_m(3)];
        
        mag_r=Qua2Mat(X1(1),X1(2),X1(3),X1(4))^(-1)*mag;

        obs_dip = acos(mag_r(2));
        R2=eye(3).*(km*abs(obs_dip-dip)) + eye(3).*sigm;
        % R2 = eye(3).*sigm;
        
        K2 = Ppri*H2'*(H2*Ppri*H2'+R2)^(-1);
        
%         h2 = [2*Xpri(2)*Xpri(3)+2*Xpri(1)*Xpri(4);
%             Xpri(1)^2-Xpri(2)^2-Xpri(3)^2-Xpri(4)^2;
%             2*Xpri(3)*Xpri(4)-2*Xpri(1)*Xpri(2)];
        h2 = Qua2Mat(X1(1),X1(2),X1(3),X1(4))*vec_m;
        Xe2 = K2*(mag-h2);
        % Xe2(2) = 0;
        % Xe2(3) = 0;
        X2 = X1+Xe2;
        P2 = (eye(4)-K2*H2)*P1; 
        if stage == 0
            Xpos = Xpri;
            Ppos = Ppri;
        else
            if stage==1
                Xpos = X1;
                Ppos = P1;
            else
                Xpos = X2;
                Ppos = P2;
            end
        end
        Xpos = Xpos./norm(Xpos);
               
    end
    all_state(:,i)=Xpos; 
end
