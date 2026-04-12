function gimbal_6dof_sfunc(block)
    setup(block)
end


function setup(block)
    % input ports:
    % - gimbal pitch (theta, around the x-axis)
    %   denoted phi in the original dynamics paper
    % - gimbal yaw (psi, around the y-axis)
    %   denoted psi in the original dynamics paper (yay)
    % - gimbal lever arm (L, m) -> TODO: change notation to "R"
    % - propeller constant (kP, N/(rev/s)^2)
    % - prop speed (n, rev/s)
    block.NumInputPorts = 5;

    % output ports:
    % - trans. force vector (fx, fy, fz)
    % - gimbal torque vector (tx, ty, tz)
    block.NumOutputPorts = 2;

    % Set up the port properties to be inherited or dynamic.
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    
    % Override input port properties (for ports 1-6)
    % even though port 1 will be a vector with 4 values, the vals
    % are still doubles

    for i = 1:5
        block.InputPort(i).Dimensions = 1;
        block.InputPort(i).DatatypeID  = 0;  % double
        block.InputPort(i).Complexity  = 'Real';
    end

    % Override output port properties
    for i = 1:2
        block.OutputPort(i).Dimensions = [1 3];
        block.OutputPort(i).DatatypeID  = 0; % double
        block.OutputPort(i).Complexity  = 'Real';
    end

    % Register continuous sample times [0 offset]
    block.SampleTimes = [0 0];

    % Specify if Accelerator should use TLC or call back into
    % MATLAB script
    % basically, just use the interpreter
    block.SetAccelRunOnTLC(false);

    %% -----------------------------------------------------------------
    %% Register methods called during update diagram/compilation
    %% -----------------------------------------------------------------
    block.RegBlockMethod('Outputs', @Outputs);
    % not implemented, but was mentioned in examples
    % block.RegBlockMethod('CheckParameters', @CheckPrms);
    block.RegBlockMethod('ProcessParameters', @ProcessPrms);
    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup)
    % not explicitely defined in ex.
    % block.RegBlockMethod('Terminate', @Terminate);
end


function ProcessPrms(block)
    %% Update run time parameters
    block.AutoUpdateRuntimePrms;
end

function DoPostPropSetup(block)
    %% Register all tunable parameters as runtime parameters.
    block.AutoRegRuntimePrms;
end


% source of eqs: https://www.mathworks.com/help/aeroblks/rotor.html
function Outputs(block)
    [theta, psi, L, kP, n] = get_block_ins(block);

    % if no theta -> set to zero
    % if no phi -> zero
    % if no L, kP, or n -> error

    % handle no inputs for the needed args
    if(isempty(L))
        error('Enter a value for the gimbal lever arm (L).')
    end
    if(isempty(kP))
        error('Enter a value for the propeller constant (kP).')
    end
    if(isempty(n))
        error('Enter a value for the propeller rev/s (n).')
    end
    
    % handle no inputs for theta or phi
    if(isempty(theta))
        theta = 0;
    end
    if(isempty(psi))
        psi = 0;
    end
    
    prop_force = get_vehicle_body_thrust_vec(kP, n, theta, psi);

    prop_torque = cross([0; 0; L], prop_force);

    disp(prop_torque);

    % convert both back to row vectors
    block.OutputPort(1).Data = prop_force';
    block.OutputPort(2).Data = prop_torque';
end

function [theta, psi, L, kP, n] = get_block_ins(block)
    theta = block.InputPort(1).Data;
    psi = block.InputPort(2).Data;
    L = block.InputPort(3).Data;
    kP = block.InputPort(4).Data;
    n = block.InputPort(5).Data;
end

function body_thrust = get_vehicle_body_thrust_vec(kP, n, phi, psi)
    prop_force_mag = kP * (n^2);
    prop_force_direction = rotx(rad2deg(phi)) * roty(rad2deg(psi)) * [0; 0; 1];
    body_thrust = prop_force_mag * prop_force_direction; % keep as column
end