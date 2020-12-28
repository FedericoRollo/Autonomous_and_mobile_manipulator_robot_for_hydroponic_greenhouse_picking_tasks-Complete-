function simpleTest()
    disp('Program started');
    
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    
    clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5); 

    if (clientID>-1)
        disp('Connected to remote API server');
            
        %Handle
       [returnCode,Left_motor] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking)
       
       %other code
       [returnCode] = sim.simxSetJointTargetVelocity(clientID,Left_motor,0.1,sim.simx_opmode_blocking)

       pause(3)
       [returnCode] = sim.simxSetJointTargetVelocity(clientID,Left_motor,0,sim.simx_opmode_blocking)
       
    else
        disp('Failed connecting to remote API server');
    end
    sim.delete(); % call the destructor!
    
    disp('Program ended');
end
