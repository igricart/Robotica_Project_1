%% UR5 Project 1 

%All link lengths and offsets are measured in m
clear L
%            theta    d                     a         alpha
L(1) = Link([0        89.159e-3           0             0           0  0], 'modified');
L(2) = Link([0        135.85e-3           0             -pi/2       0  0], 'modified');
L(3) = Link([0        -119.7e-3           425e-3        0           0  0], 'modified');
L(4) = Link([0        93e-3              392.25e-3     0          0  pi], 'modified');
L(5) = Link([0        -94.65e-3           0            -pi/2         0  pi], 'modified');
L(6) = Link([0        82.3e-3               0            -pi/2        0  pi], 'modified');

UR5=SerialLink(L, 'name', 'UR5');
% 
%UR5.plotopt = {'workspace', [-4 4 -4], 'jointdiam' 2};

qz = [0 0 0 0 0 0];
