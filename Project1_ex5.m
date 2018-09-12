%% Iterative Inverse Kinematics calculation

%Iterations auxiliar
iterations = 200;

%Tolerance definition
tolerance = 0.01;
n_error = 100;

%Desired pose according to RPY
R = [0 0 1; 1 0 0; 0 1 0];
angle_rpy = R2rpy(R);
xd = [0.7;0.1;0.0;angle_rpy];

%Desired pose according to eul

tr_des = [R [0.7;0.1;0.0]; 0 0 0 1];
angle_eul = tr2eul(tr_des);
xd = [0.7;0.1;0.0;angle_eul'];

%Joint angle initial guess (rad)
q0 = [0 0 0 0 0 0]';

qk = q0;
Q = [qk];
Xk = zeros(6,1);
i = 0;

while (i < iterations && n_error > tolerance)

    jk=UR5.jacob0(qk,'eul');
    Tk=UR5.fkine(qk);
    
    %Position and orientation acquired
    xk=Tk.t;
    
    %For RPY
    %R_ori = [Tk.n Tk.o Tk.a];
    %ok= R2rpy(R_ori);
    
    %For Euler angles
    ok = tr2eul(Tk)';
    
    %Xk
    xk = [xk;ok];
    
    qk=qk+0.05*pinv(jk)*(xd-xk);
    n_error = norm(xd-xk);
    
    Q=[Q qk];
    Xk = [Xk xk];
    %pause
    i = i + 1;
end


%UR5.plot(Q', 'delay', 0.5, 'trail', '*')