function q=Euler2Qua(yaw,pitch,roll)
q=zeros(4,1);
q(1)=sin(pitch/2)*sin(roll/2)*sin(yaw/2)+cos(pitch/2)*cos(roll/2)*cos(yaw/2);
q(2)=sin(roll/2)*cos(pitch/2)*cos(yaw/2)-sin(pitch/2)*sin(yaw/2)*cos(roll/2);
q(3)=sin(roll/2)*sin(yaw/2)*cos(pitch/2)+sin(pitch/2)*cos(roll/2)*cos(yaw/2);
q(4)=-sin(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(yaw/2)*cos(pitch/2)*cos(roll/2);
q=q./norm(q);
end