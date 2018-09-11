%
% Antropomorphic arm example
%

close all;

% parameters

% Joint Axis
h1=[0;0;1];
h2=[0;1;0];
h3=h2;
h4=h3;
h5=[0;0;1];
h6=[0;1;0];
H=[h1 h2 h3 h4 h5 h6];

% Links
p01=[0;0;89.159e-3];
p12=[0;135.85e-3;0];
p23=[425e-3;-119.7e-3;0];
p34=[392.25e-3;0;0];
p45=[0;93e-3;0];
p56=[0;0;-94.65e-3];
p6e=[0;82.3e-3;0];

P=[p01 p12 p23 p34 p45 p56];
type=[0 0 0 0 0 0]; % 6R robot
n=6;

%End effector
R_offset = [[-1;0;0],[0;0;1],[0;1;0]];
p_offset = p6e;

scale=0.3;plotsize=3;
fignum=1;figure(fignum)
theta=1;
while size(theta)>0
  theta=input(['enter theta vector (',num2str(n),'x1): ']);
  if size(theta)>0
    clf(fignum);
%     [R,p]=fwdkin2(theta,type,H,P,n,R_offset,p_offset);
%     disp(R);disp(p)
    [R,p,jacob]=jacob_rec(theta,type,H,P,n,R_offset,p_offset);
    disp(R);disp(p);disp(jacob);
%     showarm(theta,type,H,P,n,fignum,scale,plotsize);
    disp('press any key to continue ...');
    pause;
  end
end
