function gimbal_sfunc(block)
    setup(block)


function setup(block)
    % input ports:
    % - body orientation (theta)
    % - gimbal angle (phi)
    % - gimbal lever arm (L, m)
    % - propeller constant (kT)
    % - prop speed (n, rev/s)
    % - prop diameter (D, m)
    block.NumInputPorts = 6;

    % output ports:
    % - (maybe?) gimbal force
    % - trans. force x
    % - trans. force z
    % - gimbal torque (Nm)
    block.NumOutputPorts = 4;

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
    % not explicitely defined in ex.
    block.RegBlockMethod('Terminate', @Terminate);


% source of eqs: https://www.mathworks.com/help/aeroblks/rotor.html
function Outputs(block)
    theta = block.InputPort(1).Data;
    phi = block.InputPort(2).Data;
    L = block.InputPort(3).Data;
    kT = block.InputPort(4).Data;
    n = block.InputPort(5).Data;
    D = block.InputPort(6).Data;

    propForce = kT * 1.293 * (n^2) * (D^4);

    forceX = propForce * (cos(phi)*sin(theta) + sin(phi)*cos(theta));
    forceZ = propForce * (cos(phi)*cos(theta) - sin(phi)*sin(theta));
    torque = L * propForce * sin(phi);

    block.OutputPort(1).Data = forceX;
    block.OutputPort(2).Data = forceZ;
    block.OutputPort(3).Data = torque;