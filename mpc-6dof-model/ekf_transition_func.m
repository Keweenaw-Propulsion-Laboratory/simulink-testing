
function x_hat = ekf_transition_func(x, extra_in)
    u = extra_in(1:3);
    kP = extra_in(4);
    body_mass = extra_in(5);
    gravity_accel = extra_in(6);
    inertia_tensor = reshape(extra_in(7:15), 3, 3);
    lever_arm = extra_in(16);
    Ts = 1; % same as the time horizon for MPC
    % TODO: make an input to the EKF block
    % So, this will effectively keep a ZOH on u
    
    k_1 = vehicle_state_func(x, u, kP, body_mass, gravity_accel, inertia_tensor, lever_arm);
    k_2 = vehicle_state_func(x + k_1 * Ts/2, u, kP, body_mass, gravity_accel, inertia_tensor, lever_arm);
    k_3 = vehicle_state_func(x + k_2 * Ts/2, u, kP, body_mass, gravity_accel, inertia_tensor, lever_arm);
    k_4 = vehicle_state_func(x + k_3 * Ts, u, kP, body_mass, gravity_accel, inertia_tensor, lever_arm);

    x_hat = zeros(17, 1);

    x_hat_determ = x + (Ts/6) * (k_1 + 2 * k_2 + 2 * k_3 + k_4);
    
    for i = 1:13
        if(i <= 13)
            x_hat(i) = x_hat_determ(i);
        end
    end
    % leave the disturbance variables to be determined by the KF

    % TODO: normalize quaternion states
end

