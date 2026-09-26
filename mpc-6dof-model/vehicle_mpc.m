% states: 6
% s: vehicle position
% s_dot: vehicle velocity
% roll: vehicle roll (rot. about x-axis)
% pitch: vehicle pitch (rot. about y-axis)
% yaw: vehicle yaw (rot. about z-axis)
% omega: vehicle rot. velocity
% however, with all the expanded components of the states
% (as is shown below in the fetching of sys params)
% there are 12 inputs

% outputs: 1 (3 components)
% s: vehicle position (expands to 3 components)

% inputs: 3
% n: RPS of the vehicle's propeller
% phi: propeller pitch
% psi: propeller yaw

mpc_obj = nlmpc(12, 3, 3);

mpc_obj.Model.NumberOfParameters = 5; % specified below
mpc_obj.Model.OutputFcn = @vehicle_output_func;
mpc_obj.Model.StateFcn = @vehicle_state_func;


% TESTING
% arbitrary weight values
% prioritize position and keep the others stable
mpc_obj.Weights.OutputVariables = [1, 1, 1,];

% MV Weights: Don't penalize RPS too hard (or it won't fight gravity)
% penalize MV Rate to prevent the gimbal from vibrating (jitter)
mpc_obj.Weights.ManipulatedVariables = [0.1, 0.1, 0.1]; 
mpc_obj.Weights.ManipulatedVariablesRate = [0.1, 0.5, 0.5];

% values are based on the AFC drone spec sheet
% AERE Google Drive > KPL > AFC Drone > AFC Documents > "AFC Drone Spec Sheet"

% mv constraints
mpc_obj.ManipulatedVariables(1).Min = 0;
mpc_obj.ManipulatedVariables(2).Min = deg2rad(-20);
mpc_obj.ManipulatedVariables(3).Min = deg2rad(-20);
mpc_obj.ManipulatedVariables(1).Max = 167;
mpc_obj.ManipulatedVariables(2).Max = deg2rad(20);
mpc_obj.ManipulatedVariables(3).Max = deg2rad(20);

% state constraints
% rotation about x-axis
mpc_obj.States(7).Min = deg2rad(-17);
mpc_obj.States(7).Max = deg2rad(17);
% rotation about y-axis
mpc_obj.States(8).Min = deg2rad(-17);
mpc_obj.States(8).Max = deg2rad(17);

% output variables (just with respect to states)
function y = vehicle_output_func(x, ~, ~, ~, ~, ~, ~)
    y = [x(1); x(2); x(3)]; % just the position
end

% default values for params (all zero)
% note, these variables (I believe) are completely ignored in the context
% of this file. These are purely for assignment in the "createParameterBus"
% func
[kP, body_mass, gravity_accel, lever_arm] = deal(1);
inertia_tensor = eye(3);

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
    [~, s_dot, rot, omega, n, phi, psi] = get_system_props(x, u);

    % quaternion object for the quaternion coefficients
    % order of inputs: z, y, x
    % q = eul2quat([rot(3), rot(2), rot(1)]);
    % body_quat = quaternion(q);

    % referenced from the vehicle gimbal sfunc
    body_thrust = get_vehicle_body_thrust_vec(kP, n, phi, psi);
    rot_matrix = rotz(rad2deg(rot(3))) * roty(rad2deg(rot(2))) * rotx(rad2deg(rot(1)));
    earth_thrust = rot_matrix * body_thrust;
    % earth_thrust = rotatepoint(body_quat, body_thrust')'; % need row vec input for TB
    % converted back to a column vec after operation

    % velocity, passed through
    % so, velocity = s_dot specified later in x_dot assignment

    % accel -> based only on the thrust of the gimbal
    accel = ((1/body_mass) * earth_thrust) - [0; 0; gravity_accel];

    % quaternion time derivative
    % from notes: q_dot = 0.5 q (x) omega
    % converts back to the 4 quaternion coefficients
    %omega_quat_conv = quaternion(0, omega(1), omega(2), omega(3));
    %[q_dot_1, q_dot_2, q_dot_3, q_dot_4] = parts(0.5 * mtimes(body_quat, omega_quat_conv));
    % TODO: remove quat time derivative after usage is not needed

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
    
    x_dot = zeros(12, 1);

    % TODO: find source for finding rotation change based on body rotation
    phi = rot(1); theta = rot(2);   % roll, pitch
    p = omega(1); q = omega(2); r = omega(3);
    
    x_dot(7) = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    x_dot(8) = q*cos(phi) - r*sin(phi);
    x_dot(9) = (q*sin(phi) + r*cos(phi)) / cos(theta);

    for i = 1:3
        x_dot(i) = s_dot(i);
        x_dot(i + 3) = accel(i);
        % % insert rotational velocity as change in angle
        % x_dot(i + 6) = omega(i);
        x_dot(i + 9) = rot_accel(i);
    end
end

% states (all scalars):
% 1-3: s(xyz)
% 4-6: v(xyz)
% 7-9: rot(xyz)
% 10-12: omega(xyz)
% TODO: fix NaN values potentially being thrown to other functions
% note, this may be caused by bad formatting of returned data
function [s, s_dot, rot, omega, n, phi, psi] = get_system_props(x, u)
    % set all the 4 element vectors to 3x1 zero vecs
    [s, s_dot, rot, omega] = deal(zeros(3, 1));

    for i = 1:3
        s(i) = x(i);
        s_dot(i) = x(i + 3);
        rot(i) = x(i + 6);
        omega(i) = x(i + 9);
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

x0 = zeros(1,12);
u0 = zeros(1,3);
validateFcns(mpc_obj,x0,u0, [], {kP, body_mass, gravity_accel, inertia_tensor, lever_arm});
