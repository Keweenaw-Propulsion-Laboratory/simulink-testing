function gimbal_sfunc(block)
    setup(block)
end


function setup(block)
    % input ports:
    % - body orientation (theta)
    % - gimbal angle (phi)
    % - gimbal lever arm (L, m)
    % - propeller constant (kP, N/(rev/s)^2)
    % - prop speed (n, rev/s)
    block.NumInputPorts = 5;

    % output ports:
    % - (maybe?) gimbal force
    % - trans. force x
    % - trans. force z
    % - gimbal torque (Nm)
    block.NumOutputPorts = 3;

    % Set up the port properties to be inherited or dynamic.
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    
    % Override input port properties
    for i = 1:6
        block.InputPort(i).DatatypeID  = 0;  % double
        block.InputPort(i).Complexity  = 'Real';
    end

    % Override output port properties
    for i = 1:3
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
    block.RegBlockMethod('CheckParameters', @CheckPrms);
    block.RegBlockMethod('ProcessParameters', @ProcessPrms);
    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup)
    % not explicitely defined in ex.
    block.RegBlockMethod('Terminate', @Terminate);
end

function CheckPrms(block)
    % theta and phi not used
    [~, ~, L, kP, n] = get_block_ins(block);

    % if no theta -> set to zero
    % if no phi -> zero
    % if no L, kP, or n -> error

    if(isempty(L))
        error('Enter a value for the gimbal lever arm (L).')
    end
    if(isempty(kP))
        error('Enter a value for the propeller constant (kP).')
    end
    if(isempty(n))
        error('Enter a value for the propeller rev/s (n).')
    end
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
    [theta, phi, L, kP, n] = get_block_ins(block);
    
    % handle no inputs for theta or phi
    if(isempty(theta))
        theta = 0;
    end
    if(isempty(phi))
        phi = 0;
    end

    propForce = kP * (n^2);

    forceX = propForce * sin(theta + phi);
    forceZ = propForce * cos(theta + phi);
    torque = L * propForce * sin(phi);

    block.OutputPort(1).Data = forceX;
    block.OutputPort(2).Data = forceZ;
    block.OutputPort(3).Data = torque;
end

function block_ins = get_block_ins(block)
    theta = block.InputPort(1).Data;
    phi = block.InputPort(2).Data;
    L = block.InputPort(3).Data;
    kP = block.InputPort(4).Data;
    n = block.InputPort(5).Data;

    block_ins = [theta, phi, L, kP, n];
end