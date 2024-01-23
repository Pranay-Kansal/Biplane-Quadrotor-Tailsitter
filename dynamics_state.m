t_max=50;               % time of running                 
dt=0.01 ;               % dt
time = 0:dt:t_max;      %time array
nt=numel(time);         %time step

X=zeros(12,nt);         % state function - x y z u v w phi theta psi p q r
X_dot=zeros(12,nt);     % change of rate

%initial_conditions
X(:,1)=[0 0 0 0 0 0 0 0 0 0 0 0].'; % x y z u v w phi theta psi p q r

%desired_state
Xd=[0.7 0.7 -0.7 0 0 0 0 0 0 0 0 0].';         % x y z xdot ydot zdot xddot yddot zddot psi psidot

X(7:12,1)=(pi/180)*X(7:12,1);
Xd(10:12)=(pi/180)*Xd(10:12);

%parameters
m=12;
g=10;
J=[1.86 0 0
   0 2.031 0
   0 0 3.617];
d1=0.5;
d2=0.5;
c=0.01;
K_DD=[0 0 0];
K_D=[0 0 0 0 0 0];
K_P=[0.3 0.3 0.3 0 0 0];

%input
f=zeros(4,nt);          % declaring a variable
Acoeff=[1 1 1 1
        d1 -d1 -d1 d1
        d2 d2 -d2 -d2
        c -c c -c];     % force of each rotor X Acoeff = Thrust roll pitch yaw
Fg_i=[0 0 m*g].';       % gravity force

phi_c=0;
theta_c=0;
[Tlmn_b,phi_c,theta_c]=control(X(:,1),X_dot(:,1),Xd,phi_c,theta_c,K_DD,K_D,K_P,J,m,g,dt); %initialising U1(input)

for i=1:nt-1
    
    f(:,i)=Acoeff\Tlmn_b;                             %Thrust of each rotor
    
    F_b=[0 0 -Tlmn_b(1)].'+rotate(X(7:9,i))*Fg_i;     %calculating force on quadplane wrt body frame
    M_b=Tlmn_b(2:4);                                  %calculating moment on quadplane wrt body frame
    
    X_dot(:,i)=dynamics(X(:,i),F_b,M_b,J,m);          %rate of change of state matrix
    X(:,i+1)=X(:,i)+dt*X_dot(:,i);                    %calculating X2 from X1 and X1_dot
    
    [Tlmn_b,phi_c,theta_c]=control(X(:,i+1),X_dot(:,i),Xd,phi_c,theta_c,K_DD,K_D,K_P,J,m,g,dt); %U2 from X2,X1dot,Xd
    
end

for i=1:nt
    X(7:12,i)=(180/pi)*X(7:12,i);
end

figure("Name","Angles")
plot(time,X(7,:))
hold on
plot(time,X(8,:))
plot(time,X(9,:))
legend("phi","theta","psi")
hold off

figure("Name","Position")
plot(time,X(1,:))
hold on
plot(time,X(2,:))
plot(time,X(3,:))
legend("x","y","z")
hold off
% 
% figure("Name","control inputs")
% plot(time,f(1,:))
% hold on
% plot(time,f(2,:))
% plot(time,f(3,:))
% plot(time,f(4,:))
% legend("f1","f2","f3","f4")
% hold off
