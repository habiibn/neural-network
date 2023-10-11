close all;
clc; clear;

addpath('./lib');

%% Define

R2D = 180/pi; % Conversion from Radian to Degree
D2R = pi/180; % Conversion from Degree to Radian

%% Initialize Parameter of Drone
drone_params = containers.Map( ...
    {'Mass', 'armLength', 'Ixx', 'Iyy', 'Izz'}, ...
    {0.08, 0.06, 0.679e-2, 0.679e-2, 1.313e-2}); % Parameters of DJI Tello Quadcopter

drone_initStates = [2.0, 3.0, 1.5, ...   % X, Y, Z in Inertial Frame
                    0, 0, 0, ...         % dX,dY,dZ Velocity of Drone
                    0, 0, 0, ...         % varphi, theta, psi| Orientation of Drone
                    0, 0, 0]';            % p,q r. Angular Rates of Orientation Euler Angle

drone_initControlInput = [0, 0, 0, 0]';   % U1,U2,U3,U4 (T, tx, ty, tz) Control Input four propeller

% DRONE PLUS-SHAPE
% drone_body = [ 0.06,     0,    0, 1;...  % Motor 1 Position
%                   0, -0.06,    0, 1;...  % Motor 2 Position     
%               -0.06,     0,    0, 1;...  % Motor 3 Position
%                   0,  0.06,    0, 1;...  % Motor 4 Position
%                   0,     0,    0, 1;...  % Adding Payload
%                   0,     0, 0.01, 1]';   % Addding another payload like batteries, FC, etc

% TELLO BODY FRAME
drone_body = [ 0.042,  0.042,    0, 1;...  % Motor 1 Position
              -0.042,  0.042,    0, 1;...  % Motor 2 Position     
              -0.042, -0.042,    0, 1;...  % Motor 3 Position
               0.042, -0.042,    0, 1;...  % Motor 4 Position
                   0,      0,    0, 1;...  % Adding Payload
                   0,      0, 0.01, 1]';   % Addding another payload like batteries, FC, etc

% Its for the altitude controller, delete P_xpos if you dont need
% trajectory tracking
drone_gains = containers.Map( ...
    {'P_varphi','I_varphi', 'D_varphi',...
    'P_theta','I_theta', 'D_theta',...
    'P_psi','I_psi', 'D_psi',...
    'P_zdot','I_zdot', 'D_zdot'},...
    {0.3, 0.02, 0.3,...
     0.4, 0.03, 0.3,...
     0.2, 0.01, 0.5,...
     0.3, 0.01, 0.005});

drone_posGains = containers.Map( ...
    {'P_xpos','I_xpos', 'D_xpos',...
    'P_ypos','I_ypos', 'D_ypos',...
    'P_zpos','I_zpos', 'D_zpos'},...
    {0.0, 0.0, 0.0,...
     0.0, 0.0, 0.0,...
     0.0, 0.0, 0.0});

simulationTime = 2;
drone = Drone(drone_params, drone_initStates, drone_initControlInput, drone_gains, drone_posGains, simulationTime);

% Waypoint
Ref(1) = 1.0; % x
Ref(2) = 2.0; % y
Ref(3) = 1.5; % z

%% INIT. 3D Figure
fig = figure('Position',[0 200 800 800]);
h = gca;
view(3); % sets the default 3-D view
%fig.CurrentAxes.ZDir = 'Reverse';
%fig.CurrentAxes.YDir = 'Reverse';

axis equal;
grid on;

xlim([0 5]);
ylim([0 5]);
zlim([0 3.5]);
xlabel('X[m]')
ylabel('Y[m]')
zlabel('Z[m]')

hold(gca,'on');
drone_state = drone.getState();

    %display(RPY2Rot(drone_state(7:9))) % This is the euler angles, we getting the rotation matrix
    %display(drone_state(1:3)) % This is the position X,Y,Z
wHb = [RPY2Rot(drone_state(7:9))' drone_state(1:3);
       0 0 0 1]; % This is idk for what --> The answer lies in this reference --> This is Transformation Matrix, but why input the translation component equal to its own position?
       % it says 0 0 0 1, with 0 0 0 is it perpsective and 1 to global
       % scale
      
drone_world = wHb * drone_body; % 4x4 and 4x6, Result is matrix in dimension of 4x6

%{
https://www.youtube.com/watch?v=4srS0s1d9Yw&list=PLe_-6Y6QrDd9xzH1pvvT6CMyH64satvWu&index=26
Transformation matrix represent about translation and rotation, while
rotation spesifically represent about rotation only, so the reason
transformation matrix multiply to body is so that we can see in graph the
translation of drone. It also because we define the propeller in matlab as
point, thats why!
%}

% Suppose its the information about the body, but I still don't know how to
% read the drone_body
drone_altitude = drone_world(1:3, :);

% Still dont know how this works, but this is the first initializing plot
% before looping
fig_ARM13 = plot3(gca, drone_altitude(1,[1 3]), drone_altitude(2,[1 3]), drone_altitude(3,[1 3]), '-ro', 'MarkerSize', 5);
fig_ARM24 = plot3(gca, drone_altitude(1,[2 4]), drone_altitude(2,[2 4]), drone_altitude(3,[2 4]), '-bo', 'MarkerSize', 5);
fig_PAYLOAD = plot3(gca, drone_altitude(1,[5 6]), drone_altitude(2,[5 6]), drone_altitude(3,[5 6]), '-k', 'LineWidth', 3);
fig_shadow = plot3(gca, drone_initStates(1), drone_initStates(2), 0, 'xk' , 'LineWidth',2);
fig_target = plot3(gca, Ref(1), Ref(2), Ref(3), 'xr' , 'LineWidth',2);

hold(gca, 'off');

%% INIT DATA LOGGING FIGURE
fig2 = figure('pos', [800 600 800 400]);
subplot(2,3,1);
title('phi[deg]');
grid on;
hold on;

subplot(2,3,2);
title('theta[deg]');
grid on;
hold on;

subplot(2,3,3);
title('psi[deg]');
grid on;
hold on;

subplot(2,3,4);
title('x[m]')
grid on;
hold on;

subplot(2,3,5);
title('y[m]');
grid on;
hold on;

subplot(2,3,6);
title('z[m/s]');
grid on;
hold on;

%% LOOP
% Control Input
ReferenceSig(1) = 1.848;     % Z Thrust, + for Upward | - for downward
ReferenceSig(2) = 20.0 * D2R;  % Phi
ReferenceSig(3) = 0.0 * D2R;  % Theta
ReferenceSig(4) = 0.0 * D2R;  % Psi

for i=1:simulationTime/0.01
    % Trying position controller [NOT COMPLETED], Uncomment code below to
    % see its works!
    %drone.landing(Ref);
    
    % First, we try to send some control to our block control (U1,U2,U3,U4)
    drone.attitudeCtrl(ReferenceSig);
    % Next, updating the state of the drone, its include updating position,
    % and orientation of drone, from euler integration
    drone.updateState();
    
    % Get state of all DoF so we can plot it
    drone_state = drone.getState();
    %% 3D Plot again
    figure(1);
    % Drone_state(7:9)
    wHb = [RPY2Rot(drone_state(7:9))' drone_state(1:3); 0 0 0 1];
    %{
        I think its making conect, ok here we go, So basically transformation 
        matrix can represent combined transformations such as translation 
        and rotation. So, In wHb basically I tried to transform from body
        Frame (which is the input drone_state position and orientation of
        drone it self) to the Earth Frame I refer to, that is I had in
        RPY2Rot
    %}
    drone_world = wHb * drone_body;
    drone_altitude = drone_world(1:3, :);
    title(['Iteration ',num2str(i), ', Time ', num2str(i/100), 's'])

    set(fig_ARM13,...
        'xData', drone_altitude(1,[1 3]),...
        'yData', drone_altitude(2,[1 3]),...
        'zData', drone_altitude(3,[1 3]));

    set(fig_ARM24,...
        'xData', drone_altitude(1,[2 4]),...
        'yData', drone_altitude(2,[2 4]),...
        'zData', drone_altitude(3,[2 4]));

    set(fig_PAYLOAD,...
        'xData', drone_altitude(1,[5 6]),...
        'yData', drone_altitude(2,[5 6]),...
        'zData', drone_altitude(3,[5 6]));

    set(fig_shadow,...
        'xData', drone_state(1),...
        'yData', drone_state(2),...
        'zData', 0);
    %pause(0.005);
    
%     figure(2)
%     subplot(2,3,1);
%     plot(i/100, drone_state(7)*R2D, '.');
%     yline(ReferenceSig(2) * R2D,'-.b')
%     
%     subplot(2,3,2);
%     plot(i/100, drone_state(8)*R2D, '.');
%     yline(ReferenceSig(3) * R2D,'-.b')
%     
%     subplot(2,3,3);
%     plot(i/100, drone_state(9)*R2D, '.');
%     yline(ReferenceSig(4) * R2D,'-.b')
% 
%     subplot(2,3,4);
%     plot(i/100, drone_state(1), '.');
% 
%     subplot(2,3,5);
%     plot(i/100, drone_state(2), '.');
%     
%     subplot(2,3,6);
%     plot(i/100, drone_state(6), '.');
%     %yline(ReferenceSig(1) * R2D,'-.b')

     
    if(drone_state(3) <= 0.05)
        msgbox('Crashed!!','Error','error');
        display((drone_state(1:3)))
        break;
    end
    drawnow;
    % Pause for some sec, because I want to see the simulation slowly
    %pause(0.05);

end