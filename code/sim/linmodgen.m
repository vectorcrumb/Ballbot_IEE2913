function [A,B,C,D, x0, u0] = linmodgen(op_point)
    %LINMODGEN Summary of this function goes here
    %   Detailed explanation goes here

    syms phi dphi theta dtheta mom

    %% Define dynamic equations and non linear system matrix
    dddem = cos(theta)*(cos(theta) - 0.025699909063352528 ) - 1.817237831568766;
    ddphi = (mom*(26.94419184264592*cos(theta) + 52.89241687557496) - 1.9758858913475466*dtheta*dtheta*sin(theta) - 1.0504837829645348*sin(theta) + 40.875*sin(2*theta))/dddem;
    ddtheta = (-mom*(26.944191842645914*cos(theta) + 24.436805696634316) + dtheta*dtheta*(0.5*sin(2*theta) - 0.012849954531676264*sin(theta)) - 75.19295119733256*sin(theta))/dddem;

    x = [phi; theta; dphi; dtheta];
    u = mom;
    f_x = [dphi;
           dtheta;
           ddphi;
           ddtheta];
    g_x = [phi; theta; dphi; dtheta];

    %% Define initial conditions, calculate equilibrium input and define init vectors
    phi_0 = op_point(1);
    theta_0 = op_point(2);
    dphi_0 = op_point(3);
    dtheta_0 = op_point(4);

    ddtheta_eq = subs(ddtheta, {phi, theta, dphi, dtheta}, {phi_0, theta_0, dphi_0, dtheta_0});
    momeq = solve(ddtheta_eq == 0, mom);

    x0 = [phi_0; theta_0; dphi_0; dtheta_0];
    u0 = momeq;

    %% Linearization by Taylor approximation, evaluated in operating point
    A_sym = jacobian(eval(f_x),x);
    B_sym = jacobian(eval(f_x),u);
    C_sym = jacobian(eval(g_x),x);
    D_sym = jacobian(eval(g_x),u);

    A = eval(subs(A_sym,{x(1),x(2),x(3),x(4), u},{x0(1),x0(2),x0(3),x0(4), u0}));
    B = eval(subs(B_sym,{x(1),x(2),x(3),x(4), u},{x0(1),x0(2),x0(3),x0(4), u0}));
    C = eval(subs(C_sym,{x(1),x(2),x(3),x(4), u},{x0(1),x0(2),x0(3),x0(4), u0}));
    D = eval(subs(D_sym,{x(1),x(2),x(3),x(4), u},{x0(1),x0(2),x0(3),x0(4), u0}));
end

