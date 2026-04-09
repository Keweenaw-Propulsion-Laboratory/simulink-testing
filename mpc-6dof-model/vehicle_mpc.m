% states: 4
% s: vehicle position
% s_dot: vehicle velocity
% q: body quaternion
% omega: vehicle rot. velocity

% outputs: 4
% same as the states (since these are our controls)

% inputs: 3
% n: RPS of the vehicle's propeller
% phi: propeller pitch
% psi: propeller yaw

mpc_obj = nlmpc(4, 4, 3);

mpc_obj.Model.NumberOfParameters = 5; % specified below
mpc_obj.Model.StateFcn = @vehicle_state_func;

% omitting the "OutputFcn," as all our states are the outputs


% extra params needed:
% 1: kP
% 2: body_mass
% 3: gravitational_const
% 4: intertia_tensor
% 5: lever_arm
% Note, these are provided by the Simulink model
function x_dot = vehicle_state_func(x, u, params)
    [~, s_dot, q, omega, n, phi, psi] = get_system_props(x, u);

    % quaternion object for the quaternion coefficients
    body_quat = quaternion(q);

    % referenced from the vehicle gimbal sfunc
    body_thrust = get_vehicle_body_thrust_vec(params(1), n, phi, psi);
    earth_thrust = quatrotate(q, body_thrust);

    % velocity, passed through
    x_dot(1) = s_dot;

    % accel -> based only on the thrust of the gimbal
    x_dot(2) = ((1/params(2)) * earth_thrust) - [0 0 params(3)];

    % quaternion time derivative
    % from notes: q_dot = 0.5 q (x) omega
    x_dot(3) = 0.5 * body_quat * omega;

    % rotational accel
    % using Euler's equations of rigid body rotations
    % w_dot = I^-1([R x Tb] - w x Iw)
    % where I is the body's inertia tensor, and R is the lever arm for
    % the gimbal's forces on the body (distance from thrust to CoM)
    % note, the "R x Tb" term is just the body torque
    i_tens = params(4);
    lever_arm = [0 0 params(5)];
    % "\" operator used to make the inverse multiplication more efficient
    % and accurate (according to a MATLAB tooltip)
    x_dot(4) = inv(i_tens) \ (cross(lever_arm, body_thrust) - cross(omega, i_tens * omega));
end

function [s, s_dot, q, omega, n, phi, psi] = get_system_props(x, u)
    s = x(1);
    s_dot = x(2);
    q = x(3);
    omega = x(4);

    n = u(1);
    phi = u(2);
    psi = u(3);
end