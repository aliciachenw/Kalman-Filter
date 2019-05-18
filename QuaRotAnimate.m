function QuaRotAnimate(quat)
frame_num = size(quat,1);
xAxis = [1;0;0];
yAxis = [0;1;0];
zAxis = [0;0;1];
fmat = moviein(frame_num);
for i = 1:frame_num
    q = quat(i,:);
    x = Qua2Mat(q(1),q(2),q(3),q(4))*xAxis;
    y = Qua2Mat(q(1),q(2),q(3),q(4))*yAxis;
    z = Qua2Mat(q(1),q(2),q(3),q(4))*zAxis;
    clf;
    plot3([0,x(1)],[0,x(2)],[0,x(3)],'parent',gca);
    hold on;
    plot3([0,y(1)],[0,y(2)],[0,y(3)],'parent',gca);
    hold on;
    plot3([0,z(1)],[0,z(2)],[0,z(3)],'parent',gca);
    legend('x','y','z');
    set(gca,'xlim',[-1.5 1.5]);
    set(gca,'ylim',[-1.5 1.5]);
    set(gca,'zlim',[-1.5 1.5]);
    grid on;
    view(-210,30);
    fmat(i) = getframe(gca);
end
movie(fmat);
% for i = 1:frame_num
%     im = frame2im(fmat(i));
%     [I,map]=rgb2ind(im,256);
%     if i == 1
%         imwrite(I,map,'out.gif','gif','LoopCount',inf);
%     else
%         imwrite(I,map,'out.gif','gif','WriteMode','append');
%     end
% end
end