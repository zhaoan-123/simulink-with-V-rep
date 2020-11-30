function mobile2(block)


setup(block);
function setup(block)
  
  % Register number of input and output ports
  block.NumInputPorts  = 2;%输入端口数
  block.NumOutputPorts  = 2;%输入端口数
  % Setup functional port properties to dynamically
  % inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
block.InputPort(1).Dimensions  = 1;%端口数据维数
block.InputPort(1).DirectFeedthrough = false;%端口数据是否直接反馈入

block.InputPort(2).Dimensions  = 1;%端口数据维数
block.InputPort(2).DirectFeedthrough = false;%端口数据是否直接反馈入
     
block.OutputPort(1).Dimensions   = [1,3]; 
block.OutputPort(2).Dimensions   = [1,3];%端口数据维数 
%  block.OutputPort(3).Dimensions   = [1,2];%端口数据维数 


  block.SampleTimes = [-1 0]; 
  % Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';
  
block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Update', @Update);
block.RegBlockMethod('Terminate', @Terminate); % Required
block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);

%endfunction
function DoPostPropSetup(block)
block.NumDworks = 2;
  
  block.Dwork(1).Name            = 'handles';
  block.Dwork(1).Dimensions      = 3;
  block.Dwork(1).DatatypeID      = 6;      % int32
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'clientID';
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;  

function Start(block)
global vrep;
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
block.Dwork(2).Data=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);%%clientID
tstep = 0.005;  % 5ms per simulation pass
vrep.simxSetFloatingParameter(block.Dwork(2).Data,vrep.sim_floatparam_simulation_time_step,tstep,vrep.simx_opmode_oneshot);

vrep.simxSynchronous(block.Dwork(2).Data,true);%开启v-rep同步仿真设置

[~,MotorHandle_Left] = vrep.simxGetObjectHandle(block.Dwork(2).Data,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);%对象访问
[~,MotorHandle_Right] = vrep.simxGetObjectHandle(block.Dwork(2).Data,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
[~,robot]=vrep.simxGetObjectHandle(block.Dwork(2).Data,'Pioneer_p3dx',vrep.simx_opmode_blocking);
vrep.simxSynchronousTrigger(block.Dwork(2).Data);
disp('Handles available!');

block.Dwork(1).Data = [MotorHandle_Left, MotorHandle_Right, robot]';

vrep.simxStartSimulation(block.Dwork(2).Data,vrep.simx_opmode_blocking);
%     vrep.simxSetJointTargetVelocity(block.Dwork(2).Data,block.Dwork(1).Data(1),0.1,vrep.simx_opmode_oneshot);
%     vrep.simxSetJointForce(block.Dwork(2).Data,block.Dwork(1).Data(1),0,vrep.simx_opmode_oneshot);
%     vrep.simxSetJointTargetVelocity(block.Dwork(2).Data,block.Dwork(1).Data(2),0.1,vrep.simx_opmode_oneshot);
%     vrep.simxSetJointForce(block.Dwork(2).Data,block.Dwork(1).Data(2),0,vrep.simx_opmode_oneshot);
    
vrep.simxSynchronousTrigger(block.Dwork(2).Data);         % every calls this function, verp is triggered, 50ms by default

%end Start
function Outputs(block)
global vrep;
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
 [~,position] = vrep.simxGetObjectPosition(block.Dwork(2).Data,block.Dwork(1).Data(3),-1,vrep.simx_opmode_streaming);
 [~,eulerAngles] = vrep.simxGetObjectOrientation(block.Dwork(2).Data,block.Dwork(1).Data(3),-1,vrep.simx_opmode_streaming);
 [~,linearVelocity,angularVelocity]= vrep.simxGetObjectVelocity(block.Dwork(2).Data,block.Dwork(1).Data(3),vrep.simx_opmode_streaming);

 x=position(1,1);y=position(1,2);g=eulerAngles(1,3);
 P=[x,y,g];
 vx=linearVelocity(1,1);vy=linearVelocity(1,2);wz=angularVelocity(1,3);
 U=[vx,vy,wz];
 block.OutputPort(1).Data = double(P);%[x,y,z]
block.OutputPort(2).Data = double(U);%[vx,vy,wz]
% block.OutputPort(3).Data = double([]);%[a,b,g]
% block.OutputPort(1).Data = double(position);%[x,y,z]
% block.OutputPort(2).Data = double(eulerAngles);%[a,b,g]

%end Outputs


function Update(block)
global vrep;
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxSynchronousTrigger(block.Dwork(2).Data);
           wl=(2*block.InputPort(1).Data-0.331*block.InputPort(2).Data)/0.195;
           wr=(2*block.InputPort(1).Data+0.331*block.InputPort(2).Data)/0.195;
    vrep.simxSetJointTargetVelocity(block.Dwork(2).Data,block.Dwork(1).Data(1),wl,vrep.simx_opmode_oneshot);
    vrep.simxSetJointForce(block.Dwork(2).Data,block.Dwork(1).Data(1),10000,vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(block.Dwork(2).Data,block.Dwork(1).Data(2),wr,vrep.simx_opmode_oneshot);%%设置电机角速度，rad/s
    vrep.simxSetJointForce(block.Dwork(2).Data,block.Dwork(1).Data(2),10000,vrep.simx_opmode_oneshot);

%end Update
function Terminate(block)
global vrep;
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxStopSimulation(block.Dwork(2).Data,vrep.simx_opmode_oneshot);
pause(0.1);
vrep.simxFinish(block.Dwork(2).Data);
vrep.delete(); % call the destructor!

%end Terminate

function SetInpPortFrameData(block, idx, fd)
block.InputPort(idx).SamplingMode = fd;
block.OutputPort(idx).SamplingMode = fd;
















