clear all
clear
clc
sim=remApi('remoteApi');
sim.simxFinish(-1);
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
% variables
c=0;
Vrob=0.3;
kp=0.35;
tp=0;
sensor_h=[];
sensor_val=[];
sensor_loc=[-pi/2,-50/180.0*pi,-30/180.0*pi,-10/180.0*pi,10/180.0*pi,30/180.0*pi,50/180.0*pi,pi/2,pi/2,130/180.0*pi,150/180.0*pi,170/180.0*pi,-170/180.0*pi,-150/180.0*pi,-130/180.0*pi,-pi/2];

%check connection is ok
if (clientID>-1)
    disp('connected')
    %Handle
    [returnCode,left_Motor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking);
    [returnCode,right_Motor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking);
    %Set Sensor's
    for i=1:1:16
        conitos=num2str(i);
        senstring=strcat('Pioneer_p3dx_ultrasonicSensor',conitos);
        [returnCode,Sensor]=sim.simxGetObjectHandle(clientID,senstring,sim.simx_opmode_blocking);
        sensor_h(i)=Sensor;
        [returnCode,detectionState,detectedPoint,~,~]=sim.simxReadProximitySensor(clientID,Sensor,sim.simx_opmode_streaming);
        sensor_val(i)=norm(detectedPoint);
    end
    
    
    for t=1:300
        sensor_val=[];
        for j=1:1:16
            [returnCode,detectionState,detectedPoint,~,~]=sim.simxReadProximitySensor(clientID,Sensor,sim.simx_opmode_buffer);
%             disp('detect point is'); disp(detectedPoint);
            sensor_val(j)=norm(detectedPoint);
        end
        sensor_sq=sensor_val.*sensor_val;
        minsensor=min(sensor_sq);
        disp('minsensor='); disp(minsensor);
        loc=find(sensor_sq==minsensor);
        if minsensor<0.8
            steer=-1./sensor_loc(loc);
        else
            steer=0;
        end
        vl=Vrob+kp*steer;
        vr=Vrob-kp*steer;
%         disp('Vl is:'); disp(vl);
%         disp('Vr is:'); disp(vr);
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,right_Motor,vr,sim.simx_opmode_blocking);
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor,vl,sim.simx_opmode_blocking);
        pause(0.2);
    end
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,right_Motor,0,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor,0,sim.simx_opmode_blocking);
        
    end
    %add code here
    
    sim.simxFinish(-1);

sim.delete();