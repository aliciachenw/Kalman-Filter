function [all_state,hh] = UKF_2stage(data, stage, initial_euler)
% data格式统一为：
% Ch1=1yaw Ch2=1pitch Ch3=1roll Ch4=1AccX Ch5=1AccY Ch6=1AccZ Ch7=1GyroX Ch8=1GyroY Ch9=1GyroZ
% Ch10=1MagX Ch11=1MagY Ch12=1MagZ Ch13=1qaut0 Ch14=1qaut1 Ch15=1qaut2 Ch16=1qaut3 

% acc mag全部角度都纠正
siga = 0.00001;
sigq = 0.000001;
sigm = 4e-05;

Ts = 0.01;%采样时间
g = 9.80665;%重力加速度

dim = 4;
all_state = zeros(4,size(data,1));
vec_g = [0;0;-g];
alpha = 0.001;
kappa = 0;
beta = 2;
lambda = alpha^2*(dim+kappa)-dim;
gamma = 1;
weight_0 = lambda/(dim+lambda);
weight_c = lambda/(dim+lambda)+(1-alpha^2+beta);
weight_i =1 /(dim+lambda)/2;
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
        % Omega_last = Omega;
        Ppos = eye(4).*0.0001;
        Chi = zeros(dim,2*dim+1);
        Chi(:,1) = Xpos;
        A = chol(Ppos)';       
        for j = 1:dim
            Chi(:,1+j) = Xpos+A(:,j).*sqrt(dim+lambda);
        end
        for j = 1:dim
            Chi(:,1+dim+j) = Xpos-A(:,j).*sqrt(dim+lambda);
        end
    else
        % Int = eye(4)+Omega.*(Ts/2)+(Omega*Omega_last-Omega_last*Omega).*(Ts^2/48); % first-order
        Int = eye(4)+Omega.*(Ts/2);
        % Omega_last = Omega;
        for j = 1:2*dim+1
            Chi(:,j) = Int*Chi(:,j);
        end
        Xpri = Chi(:,1).*weight_0;
        for j = 1:2*dim
            Xpri = Xpri+Chi(:,j+1).*weight_i;
        end
        
        Ppri = (Chi(:,1)-Xpri)*(Chi(:,1)-Xpri)'.*weight_c;
        for j = 1:2*dim
            Ppri = Ppri+(Chi(:,1+j)-Xpri)*(Chi(:,1+j)-Xpri)'.*weight_i;
        end
        
        Q = eye(4,4).*sigq;
        Ppri = Ppri + Q;
        
        %% first stage: update pitch & roll from acc
        Y1 = zeros(3,2*dim+1);
        for j = 1:2*dim+1
            Y1(:,j) = [2*Chi(2,j)*Chi(4,j)-2*Chi(1,j)*Chi(3,j);
                2*Chi(1,j)*Chi(2,j)+2*Chi(3,j)*Chi(4,j);
                Chi(1,j)^2-Chi(2,j)^2-Chi(3,j)^2+Chi(4,j)^2];
            Y1(:,j) = Y1(:,j).*(-g);
        end
        y1 = Y1(:,1).*weight_0;
        for j = 1:2*dim
            y1 = y1+Y1(:,1+j).*weight_i;
        end
        
        Pyy1 = (Y1(:,1)-y1)*(Y1(:,1)-y1)'.*weight_c;
        for j = 1:2*dim
            Pyy1 = Pyy1+(Y1(:,j+1)-y1)*(Y1(:,j+1)-y1)'.*weight_i;
        end
        % R1 = eye(3).*(ka*abs(norm(acc)-g));
        R1 = eye(3).*siga;
        Pyy1 = Pyy1+R1;
        
        Pxy1 = (Chi(:,1)-Xpri)*(Y1(:,1)-y1)'.*weight_c;
        for j = 1:2*dim
            Pxy1 = Pxy1+(Chi(:,1+j)-Xpri)*(Y1(:,1+j)-y1)'.*weight_i;
        end
        
        K1 = Pxy1*Pyy1^(-1);
        Xe1 = K1*(acc-y1);
        % Xe1(4)=0;
        X1 = Xpri+Xe1;
        P1 = Ppri-K1*Pyy1*K1';
        hh(:,i) = y1;
        
        %% second stage: update yaw from mag
        R2 = eye(3) * sigm;
        Y2 = zeros(3,2*dim+1);
        for j = 1:2*dim+1
            Y2(:,j) = [2*Chi(2,j)*Chi(3,j)+2*Chi(1,j)*Chi(4,j);
                Chi(1,j)^2-Chi(2,j)^2-Chi(3,j)^2-Chi(4,j)^2;
                2*Chi(3,j)*Chi(4,j)-2*Chi(1,j)*Chi(2,j)];
        end        
        y2 = Y2(:,1).*weight_0;
        for j = 1:2*dim
            y2 = y2+Y2(:,1+j).*weight_i;
        end
        Pyy2 = (Y2(:,1)-y2)*(Y2(:,1)-y2)'.*weight_c;
        for j = 1:2*dim
            Pyy2 = Pyy2+(Y2(:,j+1)-y2)*(Y2(:,j+1)-y2)'.*weight_i;
        end        
        Pyy2 = Pyy2+R2;
        Pxy2 = (Chi(:,1)-Xpri)*(Y2(:,1)-y2)'.*weight_c;
        for j = 1:2*dim
            Pxy2 = Pxy2+(Chi(:,1+j)-Xpri)*(Y2(:,1+j)-y2)'.*weight_i;
        end
        K2 = Pxy2*Pyy2^(-1);
        Xe2 = K2*(mag-y2);
        % Xe2(2)=0;Xe2(3)=0;
        X2 = X1+Xe2;
        P2 = P1-K2*Pyy2*K2'; 
        
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
