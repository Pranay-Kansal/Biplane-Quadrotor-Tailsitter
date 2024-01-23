function [Tlmn,phi_c2,theta_c2] = control(x,x_dot,xd,phi_c1,theta_c1,K_DD,K_D,K_P,J,m,g,dt)
    xyz_ddot=(rotate(x(7:9)).')*x_dot(4:6);
    
    dx=K_DD(1)*(xd(7)-xyz_ddot(1))+K_D(1)*(xd(4)-x_dot(1))+K_P(1)*(xd(1)-x(1));
    dy=K_DD(2)*(xd(8)-xyz_ddot(2))+K_D(2)*(xd(5)-x_dot(2))+K_P(2)*(xd(2)-x(2));
    dz=K_DD(3)*(xd(9)-xyz_ddot(3))+K_D(3)*(xd(6)-x_dot(3))+K_P(3)*(xd(3)-x(3));
    
    phi_c2=asin((dx*sin(x(9))-dy*cos(x(9)))/(dx^2+dy^2+(dz+g)^2));
    theta_c2=atan((dx*cos(x(9))+dy*sin(x(9)))/(dz+g));
    
    phi_dot=(phi_c2-phi_c1)/dt;
    theta_dot=(theta_c2-theta_c1)/dt;
    
    Tlmn=[(dx*(sin(x(8))*cos(x(9))*cos(x(7))+sin(x(9))*sin(x(7)))+dy*(sin(x(8))*sin(x(9))*cos(x(7))-cos(x(9))*sin(x(7)))+(dz+g)*cos(x(8))*cos(x(7)))*m
            (K_D(4)*(phi_dot-x_dot(7))+K_P(4)*(phi_c2-x(7)))*J(1,1)
            (K_D(5)*(theta_dot-x_dot(8))+K_P(5)*(theta_c2-x(8)))*J(2,2)
            (K_D(6)*(xd(11)-x_dot(9))+K_P(6)*(xd(10)-x(9)))*J(3,3)];
    
end