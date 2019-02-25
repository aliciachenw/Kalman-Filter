function euler=Qua2Euler(q0,q1,q2,q3)
n=sqrt(q0^2+q1^2+q2^2+q3^2);
q0=q0/n;q1=q1/n;q2=q2/n;q3=q3/n;
% euler=zeros(3,1);
% euler(1)=atan2(2*(q0*q1+q2*q3),1-2*(q1^2+q2^2));%roll
% euler(2)=asin(2*(q0*q2-q1*q3));%pitch
% euler(3)=atan2(2*(q0*q3+q1*q2),1-2*(q2^2+q3^2));%yaw
% euler=euler.*(180/pi);
c=Qua2Mat(q0,q1,q2,q3);
euler(1)=atan2(c(1,2),c(1,1));%yaw
euler(2)=asin(-c(1,3)); %pitch
euler(3)=atan2(c(2,3),c(3,3)); %roll
euler=euler.*(180/pi);
end