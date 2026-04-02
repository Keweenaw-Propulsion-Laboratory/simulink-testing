function gimbal_6dof_sfunc(block)
    setup(block)
end


function setup(block)
    % input ports:
    % - body quaternion (q, 1x4 double vector of quat coeffs.)
    % - gimbal pitch (theta, around the x-axis)
    %   denoted phi in the original dynamics paper
    % - gimbal yaw (psi, around the y-axis)
    %   denoted psi in the original dynamics paper (yay)
    % - gimbal lever arm (L, m)
    % - propeller constant (kP, N/(rev/s)^2)
    % - prop speed (n, rev/s)
    block.NumInputPorts = 6;

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

    for i = 1:6
        input_dimension = [1 4];
        if(i ~= 1)
            input_dimension = 1;
        end

        block.InputPort(i).Dimensions = input_dimension;
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
    [body_q, theta, psi, L, kP, n] = get_block_ins(block);

    % if no theta -> set to zero
    % if no phi -> zero
    % if no body_q, L, kP, or n -> error

    % handle no inputs for the needed args
    if(isempty(body_q))
        error('Enter a value for the gimbal lever arm (L).')
    end
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


    prop_force_mag = kP * (n^2);
    prop_force_direction = quatrotate(body_q, [sin(theta), sin(psi), cos(theta)*cos(psi)]);
    prop_force = prop_force_mag * prop_force_direction;

    prop_torque = cross([0, 0, L], prop_force);

    disp(prop_torque);

    block.OutputPort(1).Data = prop_force;
    block.OutputPort(2).Data = prop_torque;
end

function [body_q, theta, psi, L, kP, n] = get_block_ins(block)
    body_q = block.InputPort(1).Data;
    theta = block.InputPort(2).Data;
    psi = block.InputPort(3).Data;
    L = block.InputPort(4).Data;
    kP = block.InputPort(5).Data;
    n = block.InputPort(6).Data;
end