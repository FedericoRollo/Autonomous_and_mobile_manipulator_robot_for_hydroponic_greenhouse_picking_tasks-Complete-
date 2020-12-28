% Make sure to have the server side running in CoppeliaSim: 
% in a child script of a CoppeliaSim scene, add following command
% to be executed just once, at simulation start:
%
% simRemoteApi.start(19999)
%
% then start simulation, and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

function simpleTest()
    clc;
    disp('Program started');
    % sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    
    
    if (clientID>-1)
        disp('Connected to remote API server');
            
        sim.simxSynchronous(clientID,true); % Enable the synchronous mode (Blocking function call)
        sim.simxStartSimulation(clientID,sim.simx_opmode_blocking);
        
        
        for i=0:2
            targ = sprintf('%s%d','target',i);
            [res,target] = sim.simxGetObjectHandle(clientID,targ,sim.simx_opmode_blocking);
            [res, target_position] = sim.simxGetObjectPosition(clientID,target,-1, sim.simx_opmode_blocking);
            moveArm(sim,clientID,target_position);
        end
        
        
        % stop the simulation:
%         sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);

        % Now close the connection to CoppeliaSim:    
        sim.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    sim.delete(); % call the destructor!
    
    disp('Program ended');
end

function moveArm(sim,clientID,target_position)
    sim.simxSynchronous(clientID,true);
    
    
    jointHandle = {-1,-1,-1,-1,-1,-1};
    current_q = [-1 -1 -1 -1 -1 -1];
    [k_q, j_q, j_p] = ArmKinematics();
    syms q1 q2 q3 q4 q5 q6

    for i = 1:6
        joint = sprintf('%s%d','joint',i);
        [res,jointHandle{i}] = sim.simxGetObjectHandle(clientID,joint,sim.simx_opmode_blocking);
        sim.simxSetObjectIntParameter(clientID,jointHandle{i},2001,0,sim.simx_opmode_blocking);
        current_q(i) = sim.simxGetJointPosition(clientID,jointHandle{i},sim.simx_opmode_blocking);
    end

    [res,ee] = sim.simxGetObjectHandle(clientID,'BaxterGripper_centerJoint',sim.simx_opmode_blocking);

    [res,ee_position] = sim.simxGetObjectPosition(clientID,ee,-1, sim.simx_opmode_blocking);

    error = ee_position - target_position;
    k=2*eye(3);

    while norm(error)>0.01

        if(norm(error)<0.1)
            k = 10*eye(3);
        end

        for i = 1:6
            [res,current_q(i)] = sim.simxGetJointPosition(clientID,jointHandle{i},sim.simx_opmode_oneshot);
        end

        j_p_curr = double(subs(j_p,[q1 q2 q3 q4 q5 q6], current_q));
        j_pinv = pinv(j_p_curr);
        q_dot = -j_pinv*k*error';

        %             for i =1:6
        %                 sim.simxPauseCommunication(clientID,1);
        res = sim.simxCallScriptFunction(clientID,'MyRoboticArm',sim.sim_scripttype_childscript,'setJointVel',[],[q_dot'],'',[],sim.simx_opmode_oneshot);
        %                 [res] = sim.simxSetJointTargetVelocity(clientID,jointHandle{i},q_dot(i),sim.simx_opmode_blocking)
        %                 sim.simxPauseCommunication(clientID,0);
        %             end
        sim.simxSynchronousTrigger(clientID);

        [res,ee_position] = sim.simxGetObjectPosition(clientID,ee,-1, sim.simx_opmode_blocking);

        error = ee_position - target_position;

    end

    q_end = [0 0 0 0 0 0];
    res = sim.simxCallScriptFunction(clientID,'MyRoboticArm',sim.sim_scripttype_childscript,'setJointVel',[],[q_end'],'',[],sim.simx_opmode_oneshot);
    sim.simxSynchronousTrigger(clientID);
    
  
    sim.simxSynchronous(clientID,false);
    
    
      for i = 1:6
         [res,pos] = sim.simxGetJointPosition(clientID,jointHandle{i},sim.simx_opmode_blocking);
         sim.simxSetJointTargetPosition(clientID,jointHandle{i},pos,sim.simx_opmode_blocking);
         sim.simxSetObjectIntParameter(clientID,jointHandle{i},2001,1,sim.simx_opmode_blocking);
      end
end
