%% Iterative Inverse Kinematics calculation

%Iterations auxiliar
iterations = 200;

%Tolerance definition
tolerance = 1;
n_error = 100;

%Desired pose according to RPY
R = [0 0 1; 1 0 0; 0 1 0];
angle_rpy = R2rpy(R);

xd = [0.3;0.3;0.3;angle_rpy];

%Joint angle initial guess (rad)
q0 = [0 -pi/4 0.7 0 0 0]';

qk = q0;
Q = [qk];

while n_error > tolerance

    %p2.plot(qk')
    jk=UR5.jacob0(qk,'rpy');

    %jk=jk(1:6,1:6);

    Tk=UR5.fkine(qk);
    
    %Position and orientation acquired
    xk=Tk.t;
    R_ori = [Tk.n Tk.o Tk.a];
    ok= R2rpy(R_ori);
    xk = [xk;ok];
    
    qk=qk+0.3*pinv(jk)*(xd-xk);
    n_error = norm(xd-xk);
    
    Q=[Q qk];
    %pause

end


UR5.plot(Q', 'delay', 1, 'trail', '*')