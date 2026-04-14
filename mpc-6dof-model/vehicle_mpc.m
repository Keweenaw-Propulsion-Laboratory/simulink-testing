% states: 4
% s: vehicle position
% s_dot: vehicle velocity
% q: body quaternion
% omega: vehicle rot. velocity
% however, with all the expanded components of the states
% (as is shown below in the fetching of sys params)
% there are 13 inputs

% outputs: 4 (13 components)
% same as the states (since these are our controls)

% inputs: 3
% n: RPS of the vehicle's propeller
% phi: propeller pitch
% psi: propeller yaw

mpc_obj = nlmpc(13, 13, 3);

mpc_obj.Model.NumberOfParameters = 5; % specified below
mpc_obj.Model.StateFcn = @vehicle_state_func;

% all constraints are row vectors
% values are based on the AFC drone spec sheet

% mpc_obj.Optimization.CustomIneqConFcn  = @vehicle_input_constraint;
% based on 0 <= n <= 167 RPS
% -20 <= phi <= 20 degrees
% -20 <= psi <= 20 degrees
% function ineq_constraint = vehicle_input_constraint(~, u, ~, ~, ~, ~, ~, ~, ~)
%     deg_conv = deg2rad(20);
%     ineq_constraint = [u(1) - 167; -u(1); u(2) - deg_conv; -u(2) - deg_conv; u(3) - deg_conv; -u(3) - deg_conv];
% end

% alternate method of setting mins and maxes: did not work initially
mpc_obj.ManipulatedVariables(1).Min = 0;
mpc_obj.ManipulatedVariables(2).Min = deg2rad(-20);
mpc_obj.ManipulatedVariables(3).Min = deg2rad(-20);
mpc_obj.ManipulatedVariables(1).Max = 167;
mpc_obj.ManipulatedVariables(2).Max = deg2rad(20);
mpc_obj.ManipulatedVariables(3).Max = deg2rad(20);


% omitting the "OutputFcn," as all our states are the outputs

% default values for params (all zero)
% note, these variables (I believe) are completely ignored in the context
% of this file. These are purely for assignment in the "createParameterBus"
% func
[kP, body_mass, gravity_accel, lever_arm] = deal(1);
inertia_tensor = eye(3);

% TODO: move all states to be in a single column vector

% extra params needed:
% 1: kP
% 2: body_mass
% 3: gravity_accel
% 4: intertia_tensor
% 5: lever_arm
% Note, these are provided by the Simulink model
% parameter values setting global variables
% TODO: verify
function x_dot = vehicle_state_func(x, u, kP, body_mass, gravity_accel, inertia_tensor, lever_arm)
    [~, s_dot, q, omega, n, phi, psi] = get_system_props(x, u);

    % quaternion object for the quaternion coefficients
    body_quat = quaternion(q);

    % referenced from the vehicle gimbal sfunc
    body_thrust = get_vehicle_body_thrust_vec(kP, n, phi, psi);
    earth_thrust = quatrotate(q, body_thrust')'; % need row vec input for TB
    % converted back to a column vec after operation

    % velocity, passed through
    % so, velocity = s_dot specified later in x_dot assignment

    % accel -> based only on the thrust of the gimbal
    accel = ((1/body_mass) * earth_thrust) - [0; 0; gravity_accel];

    % quaternion time derivative
    % from notes: q_dot = 0.5 q (x) omega
    % converts back to the 4 quaternion coefficients
    omega_quat_conv = quaternion(omega(1), omega(2), omega(3), 0);
    [q_dot_1, q_dot_2, q_dot_3, q_dot_4] = parts(0.5 * mtimes(body_quat, omega_quat_conv));

    % rotational accel
    % using Euler's equations of rigid body rotations
    % w_dot = I^-1([R x Tb] - w x Iw)
    % where I is the body's inertia tensor, and R is the lever arm for
    % the gimbal's forces on the body (distance from thrust to CoM)
    % note, the "R x Tb" term is just the body torque
    lever_arm_vec = [0; 0; lever_arm];
    % "\" operator used to make the inverse multiplication more efficient
    % and accurate (according to a MATLAB tooltip)
    % "inv(A) * b" -> "A \ b"
    rot_accel = inertia_tensor \ (cross(lever_arm_vec, body_thrust) - cross(omega, inertia_tensor * omega));
    
    x_dot = zeros(13, 1);

    for i = 1:3
        x_dot(i) = s_dot(i);
        x_dot(i + 3) = accel(i);
        x_dot(i + 10) = rot_accel(i);
    end
    
    % I hate this, but it is what it is
    x_dot(7) = q_dot_1;
    x_dot(8) = q_dot_2;
    x_dot(9) = q_dot_3;
    x_dot(10) = q_dot_4;
    % for j = 1:4
    %     x_dot(j + 6) = quat_dot(j);
    % end
end

% states (all scalars):
% 1-3: s(xyz)
% 4-6: v(xyz)
% 7-10: q coeffs
% 11-13: omega(xyz)
% TODO: fix NaN values potentially being thrown to other functions
% note, this may be caused by bad formatting of returned data
function [s, s_dot, q, omega, n, phi, psi] = get_system_props(x, u)
    % set all the 4 element vectors to 3x1 zero vecs
    [s, s_dot, omega] = deal(zeros(3, 1));
    % set the q coeffs to 0, as well (but 1x4, keeping a row vector)
    q = zeros(1, 4);

    for i = 1:3
        s(i) = x(i);
        s_dot(i) = x(i + 3);
        omega(i) = x(i + 10);
    end

    % disp(s_dot);

    for j = 1:4
        q(j) = x(j + 6);
    end
    
    % inputs are all scalars
    n = u(1);
    phi = u(2);
    psi = u(3);
end


% copied from the gimbal_6dof_sfunc file
% would like to remove duplication
function body_thrust = get_vehicle_body_thrust_vec(kP, n, phi, psi)
    prop_force_mag = kP * (n^2);
    prop_force_direction = rotx(rad2deg(phi)) * roty(rad2deg(psi)) * [0; 0; 1];
    body_thrust = prop_force_mag * prop_force_direction; % keep as column
end

createParameterBus(mpc_obj, ['root_model' '/Nonlinear MPC Controller'], 'mpc_params_bus', {kP, body_mass, gravity_accel, inertia_tensor, lever_arm});

x0 = zeros(1,13);
x0(1, 10) = 1;
u0 = zeros(1,3); 
validateFcns(mpc_obj,x0,u0, [], {kP, body_mass, gravity_accel, inertia_tensor, lever_arm});
