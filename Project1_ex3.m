p_ef = [0.7 0.1 0]';
Rex3 = [0 0 1;1 0 0;0 1 0];
Tex3 = [Rex3 p_ef; 0 0 0 1];
UR5.ikine(Tex3)
UR5.fkine(UR5.ikine(Tex3))

