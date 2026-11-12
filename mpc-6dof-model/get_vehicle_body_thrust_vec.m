function body_thrust = get_vehicle_body_thrust_vec(kP, n, phi, psi)
    prop_force_mag = kP * (n^2);
    prop_force_direction = rotx(rad2deg(phi)) * roty(rad2deg(psi)) * [0; 0; 1];
    body_thrust = prop_force_mag * prop_force_direction; % keep as column
end