%
% jacob.m
%
% Differential kinematics for serial chain
% 
% theta: n-vector of rotational angle / translational displacement
% type: 0 = rotational  nonzero = prismatic
% H = [ h1 h2 ... hn ] axis of rotation or translation
% P = [p01 p12 p23 .. p_{n-1}n] inter-link vectors
% n: # of links (>1)
% 
function [R,p,jacob_base]=jacob_rec(theta,type,H,P,n,R_offset,p_offset)

    if type(1) == 0
        R=expm(crossmat(H(1:3,1))*theta(1));
        p=P(1:3,1);
        H_jac = [zeros(3,1); H(:,1)];
    else
        R=eye(3,3);
        p=P(1:3,1)+theta(1)*H(1:3,1);
        H_jac = [H(:,1); zeros(3,1)];
    end

    jacob = H_jac;
    phi_acc(1,1:6,1:6) = eye(6);
    H_jac_acc = H_jac;
    %Complete H_jac matrix
    %H_jac = [H_jac(:,1:5) R_offset'*H_jac(:,6)]
    
    %Rotacional and position update
    for i = 2:n
        if type(i) == 0
          p=p+R*P(1:3,i);
          R_i = expm(crossmat(H(1:3,i))*theta(i)); %R_{i-1,i}
          R=R * R_i;
          H_jac = [zeros(3,1);H(:,i)];
        else
          p=p+R*(P(1:3,i)+theta(i)*H(1:3,i));
          R=R;
          H_jac = [H(:,i):zeros(3,1)];
        end

    % -----Offset correction
    
    %Jacobian computation
    phi = [R_i.', -R_i.'*crossmat(P(1:3,i)); zeros(3,3), R_i.'];
    jacob = [phi*jacob, H_jac]; 
    
    end
    
    % -----End-effector translation
    p = p + R*p_offset;
    
    %End-effector referential move (J6)6 -> (Je)6
    mov = [eye(3), -crossmat(p_offset); zeros(3,3), eye(3)];
    J_e = mov*jacob;

    % -----Jacobian on base coordinate 
    jacob_base = [R, zeros(3,3); zeros(3,3), R] * J_e;
    
    % -----End-effector rotation
    R = R*R_offset;
    