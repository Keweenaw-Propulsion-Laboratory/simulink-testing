% states: 6
% s: vehicle position
% s_dot: vehicle velocity
% q: body quaternion
% omega: vehicle rot. velocity
% w_d: translational force disturbance
% tau_d: rotational force disturbance
% however, with all the expanded components of the states
% (as is shown below in the fetching of sys params)
% there are 17 inputs

% outputs: 4 (13 components)
% same as the states (since these are our controls)
% not feeding out the disturbances, so the output func truncates

% inputs: 3
% n: RPS of the vehicle's propeller
% phi: propeller pitch
% psi: propeller yaw

mpc_obj = nlmpc(17, 13, 3);

mpc_obj.Model.NumberOfParameters = 5; % specified below
mpc_obj.Model.StateFcn = @vehicle_state_func;
mpc_obj.Model.OutputFcn = @vehicle_output_func;

% TESTING
% arbitrary weight values
% prioritize position and keep the others stable
mpc_obj.Weights.OutputVariables = [1, 1, 1, 0.1, 0.1, 0.1, 0, 0, 0, 0, 0.1, 0.1, 0.1];

% MV Weights: Don't penalize RPS too hard (or it won't fight gravity)
% penalize MV Rate to prevent the gimbal from vibrating (jitter)
mpc_obj.Weights.ManipulatedVariables = [0.1, 0.1, 0.1]; 
mpc_obj.Weights.ManipulatedVariablesRate = [0.1, 0.5, 0.5];

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

createParameterBus(mpc_obj, ['root_model' '/Nonlinear MPC Controller'], 'mpc_params_bus', {kP, body_mass, gravity_accel, inertia_tensor, lever_arm});

x0 = zeros(1,17);
x0(1, 10) = 1;
u0 = zeros(1,3); 
validateFcns(mpc_obj,x0,u0, [], {kP, body_mass, gravity_accel, inertia_tensor, lever_arm});
