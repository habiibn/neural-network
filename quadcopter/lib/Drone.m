classdef Drone < handle
%% MEMBERS // Meaning properties inside the drone to be functional
    % Dynamic Model Parameter
    properties
        g               % Earth Acceleration
        t               % Initial Time of Universe
        dt              % sampling time
        tf              % simulation Time

        m               % drone mass
        l               % arm length
        I               % Inertia Matrix
        
        x               % [X,Y,Z, dX,dY,dZ, varphi,theta,psi, p,q,r]'
        r               % [X,Y,Z]'
        dr              % [dX,dY,dZ]'
        euler           % [varphi,theta,psi]'
        w               % [p,q,r]'

        dx              % Derivative of all DoF, [dX,dY,dZ, ddX,ddY,ddZ, p,q,r, dp,dq,dr]'

        u               % [T_sum,tx,ty,tz]'
        T               % T_sum // Thrust Drone
        M               % [tx,ty,tz]' // Inertia Moment on each axis
    end

    properties
        x_des
        x_err
        x_err_prev
        x_err_sum

        y_des
        y_err
        y_err_prev
        y_err_sum

        z_des
        z_err
        z_err_prev
        z_err_sum

        p_err
        p_err_prev
        p_err_sum

        q_err
        q_err_prev
        q_err_sum

        r_err
        r_err_prev
        r_err_sum

        varphi_des
        varphi_err
        varphi_err_prev
        varphi_err_sum

        theta_des
        theta_err
        theta_err_prev
        theta_err_sum

        psi_des
        psi_err
        psi_err_prev
        psi_err_sum

        zdot_des
        zdot_err
        zdot_err_prev
        zdot_err_sum

        KP_varphi
        KI_varphi
        KD_varphi

        KP_theta
        KI_theta
        KD_theta

        KP_psi
        KI_psi
        KD_psi

        KP_zdot
        KI_zdot
        KD_zdot

        log_des
    end

%% METHOD
    methods
        %% CONSTRUCTOR \\First Thing First
        function obj = Drone(params, initStates, initControlInputs, gains, gainsPos, simTime)
            % Drone Dynamic
            obj.g = 9.81;
            obj.t = 0.0;
            obj.dt = 0.01;
            obj.tf = simTime;

            obj.m = params('Mass');
            obj.l = params('armLength');
            obj.I = [params('Ixx'),             0,             0;...
                                 0, params('Iyy'),             0;...
                                 0,             0, params("Izz")];
            
            obj.x     = initStates;
            obj.r     = obj.x(1:3);
            obj.dr    = obj.x(4:6);
            obj.euler = obj.x(7:9);
            obj.w     = obj.x(10:12);
            
            obj.dx = zeros(12,1);

            obj.u = initControlInputs;
            obj.T = obj.u(1);
            obj.M = obj.u(2:4);

            obj.x_des = 0.0;
            obj.x_err = 0.0;
            obj.x_err_prev = 0.0;
            obj.x_err_sum = 0.0;

            obj.y_des = 0.0;
            obj.y_err = 0.0;
            obj.y_err_prev = 0.0;
            obj.y_err_sum = 0.0;

            obj.z_des = 0.0;
            obj.z_err = 0.0;
            obj.z_err_prev = 0.0;
            obj.z_err_sum = 0.0;

            obj.p_err = 0.0;
            obj.p_err_prev = 0.0;
            obj.p_err_sum = 0.0;
            
            obj.q_err = 0.0;
            obj.q_err_prev = 0.0;
            obj.q_err_sum = 0.0;

            obj.r_err = 0.0;
            obj.r_err_prev = 0.0;
            obj.r_err_sum = 0.0;

            obj.varphi_des = 0.0;
            obj.varphi_err = 0.0;
            obj.varphi_err_prev = 0.0;
            obj.varphi_err_sum = 0.0;
    
            obj.theta_des = 0.0;
            obj.theta_err = 0.0;
            obj.theta_err_prev = 0.0;
            obj.theta_err_sum = 0.0;
    
            obj.psi_des = 0.0;
            obj.psi_err = 0.0;
            obj.psi_err_prev = 0.0;
            obj.psi_err_sum = 0.0;
    
            obj.zdot_des = 0.0;
            obj.zdot_err = 0.0;
            obj.zdot_err_prev = 0.0;
            obj.zdot_err_sum = 0.0;
    
            obj.KP_varphi = gains('P_varphi');
            obj.KI_varphi = gains('I_varphi');
            obj.KD_varphi = gains('D_varphi');
    
            obj.KP_theta = gains('P_theta');
            obj.KI_theta = gains('I_theta');
            obj.KD_theta = gains('D_theta');
    
            obj.KP_psi = gains('P_psi');
            obj.KI_psi = gains('I_psi');
            obj.KD_psi = gains('D_psi');
    
            obj.KP_zdot = gains('P_zdot');
            obj.KI_zdot = gains('I_zdot');
            obj.KD_zdot = gains('D_zdot');

            obj.log_des = [];
        end
        
        % Getting Indirectly my state of DoF Value from this method
        function state = getState(obj)
            state = obj.x;
        end

        function obj = EvalEOM(obj)
            bRe = RPY2Rot(obj.euler); % Transformation of body frame to Inertial frame or Earth Frame{EF}, Roll Pitch Yaw (Euler Angles) to Rotation Matrix
            R = bRe'; % So, basically R is on the other side of Body Frame, which is now is Acting to Body Frame to Inertial Frame
            
            % Implementing position dynamic Model Controller
                
            % Velocity of Translation    
            obj.dx(1:3) = obj.dr; % dx is basically derivative of all 12 variables of DoF, so its true dx(1:3) == dr, but its x(4:6) == dr
            % Acceleration of Translation
            %obj.dx(4:6) = (1 / obj.m) * ([0; 0; obj.m * - obj.g]) + (1 / obj.m)* R * obj.T * [0; 0; 1];
            obj.dx(4:6) = (1 / obj.m) * ([0; 0; obj.m * -obj.g]) + (1 / obj.m)* R * obj.T * [0; 0; 1] - (1 / obj.m) * [3.365e-2*(obj.dx(1)); 3.365e-2*(obj.dx(2)); 3.365e-2*(obj.dx(3))]; % Not Finished, yet but better follow the code first
            
            % Updating Euler Angles, reference from this
            % https://rotations.berkeley.edu/the-euler-angle-parameterization/  || The 3-2-1 set of Euler angles
            % Kalo gitu bener, bahwa di obj.euler(1) ini ddidalam 
            varphi = obj.euler(1);
            theta  = obj.euler(2);
            psi    = obj.euler(3);
            
            % Kalo gini inijadinya yang dx itu punyanya si inertial wkwk,
            % ini namanya special transfer matrix
            obj.dx(7:9) = [1 sin(varphi)*tan(theta) cos(varphi)*tan(theta);...
                           0            cos(varphi)           -sin(varphi);...
                           0 sin(varphi)*sec(theta) cos(varphi)*sec(theta)] * obj.w;
            
            % Hopefully this is correct :'), Eecause this array value is 
            % for drone orientation dynamic models [ddotvarphi, ddottheta, ddotpsi]
            
            % Acceleration of Orientation, this is dp,dq,dr for
            % Acceleration Rates in Body Frame [EDIT] --> Harusnya ini
            % Acceleration Rates di Inertial Frame nggak sih?
            %display(obj.M)
            obj.dx(10:12) = (obj.I) \ (obj.M - cross(obj.w, obj.I * obj.w));
            %display(obj.dx(10:12))
            %display('------------')
            %obj.dx(10:12) = (obj.I) \ (obj.M - cross(obj.w, obj.I * obj.w) + [4.609e-3*obj.w(1).^2; 4.609e-3*obj.w(2).^2 ; 4.609e-3*obj.w(3).^2] - (4.95e-5 * (obj.T/obj.m) * obj.w));        
        end

        function obj = updateState(obj)
            % Updating simulation time
            obj.t = obj.t + obj.dt;

            % Calling evaluation function, this is for model dynamics
            obj.EvalEOM();

            % Still dont understand this, but it purpose to integrate it?,
            % where the impact of dx  from evalEOM is place in here obj.xx
            obj.x = obj.x + obj.dx * obj.dt;
            % EDIT --> Finally understand it, explain here
            %{
                So, obj.dx is basically derivative of obj.x right? And up
                there in obj.EvalEOM we basically update our value all in
                the derivative form, in the dynamic model of quadcopter, so
                in order to have it back again to position we want, we
                integrate our obj.dx to get obj.x. Its make sense! we get
                obj.dx by derivate obj.x, so if we want obj.x we integrate
                obj.dx! I AM FUCKING GENIUS
            %}

            % Updating drone position, is this in Inertial Frame or What?
            obj.r = obj.x(1:3); % Position
            obj.dr = obj.x(4:6); % Velocity

            % Updating orientation drone
            obj.euler = obj.x(7:9); % Angular
            obj.w = obj.x(10:12); % Angular Velocity
        end

        %% CONTROLLER
%         function obj = angularCtrl(obj, RefSig)
%             %Angular Control, convert p,q,r desired to U Vector
%             p_des = RefSig(2);
%             q_des = RefSig(3);
%             r_des = RefSig(4);
% 
%             % Measured Angular Velocity
%             p_mes = obj.dx(7);
%             q_mes = obj.dx(8);
%             r_mes = obj.dx(9);
% 
%             %Error
%             obj.p_err = p_des - p_mes;
%             obj.q_err = q_des - q_mes;
%             obj.r_err = r_des - r_mes;
%             
%             % Our Implementation Controller for Position Control through
%             obj.u(1) = RefSig(1);
%             obj.u(2) = (0.2)*(obj.p_err) + (0.05)*(obj.p_err_sum) + (0.001)*(obj.p_err - obj.p_err_prev)/obj.dt;
%             obj.u(3) = (0.2)*(obj.q_err) + (0.05)*(obj.q_err_sum) + (0.001)*(obj.q_err - obj.q_err_prev)/obj.dt;
%             obj.u(4) = (0.2)*(obj.r_err) + (0.05)*(obj.r_err_sum) + (0.001)*(obj.r_err - obj.r_err_prev)/obj.dt;
% 
%             obj.p_err_prev = obj.p_err;
%             obj.q_err_prev = obj.q_err;
%             obj.r_err_prev = obj.r_err;
% 
%             obj.p_err_sum = obj.p_err_sum + obj.p_err;
%             obj.q_err_sum = obj.q_err_sum + obj.q_err;
%             obj.r_err_sum = obj.r_err_sum + obj.r_err;
%             
% 
%             %
%             % Update Control Inputs
%             obj.T = obj.u(1);
%             obj.M = obj.u(2:4);
% 
%         end
        
        function obj = attitudeCtrl(obj, refSig)
            % Defining the reference we want first
            obj.varphi_des = refSig(2); % Radian from PID Controller
            obj.theta_des  = refSig(3); % Radian from PID Controller
            obj.psi_des    = refSig(4); % Radian from PID Controller
            
            
            % Defining the error, Desired - Measurement , this is the data
            varphi_mes = obj.euler(1);
            theta_mes = obj.euler(2);
            psi_mes = obj.euler(3);
            
            % from the sensor will be inputted
            obj.varphi_err = obj.varphi_des - varphi_mes;   
            obj.theta_err  = obj.theta_des - theta_mes;
            obj.psi_err    = obj.psi_des - psi_mes;
            
            obj.u(1) = refSig(1);

            % PID Controller for phi angle
            obj.u(2) = (obj.KP_varphi * obj.varphi_err + ... % Proportional
                        obj.KI_varphi * obj.varphi_err_sum + ... % Integral
                        obj.KD_varphi * (obj.varphi_err - obj.varphi_err_prev)/obj.dt); % Derivative
            
            % PID Controller for theta angle
            obj.u(3) = (obj.KP_theta * obj.theta_err + ... % Proportional
                        obj.KI_theta * obj.theta_err_sum + ... % Integral
                        obj.KD_theta * (obj.theta_err - obj.theta_err_prev)/obj.dt); % Derivative

            % PID Controller for psi angle
            obj.u(4) = (obj.KP_psi * obj.psi_err + ... % Proportional
                        obj.KI_psi * obj.psi_err_sum + ... % Integral
                        obj.KD_psi * (obj.psi_err - obj.psi_err_prev)/obj.dt); % Derivative

            % Millecenoiyus
            obj.varphi_err_prev = obj.varphi_err;
            obj.theta_err_prev = obj.theta_err;
            obj.psi_err_prev = obj.psi_err;

            obj.varphi_err_sum = obj.varphi_err_sum + obj.varphi_err;
            obj.theta_err_sum = obj.theta_err_sum + obj.theta_err;
            obj.psi_err_sum = obj.psi_err_sum + obj.psi_err;
            
            % If you activate this, it will hover. The reason even if we
            % reference u(1) equal below here, the PID controller will
            % multiply our error even for a bit. the solution is set the
            % gain to zero
            display(obj.M)
            obj.u(1) = (obj.m * obj.g)*1;
            obj.T = obj.u(1);
            obj.M = obj.u(2:4);

            % Angular Control Subsystem
            %obj.angularCtrl(refS)

        end
        
        % Controller for Position Controller
        function obj = landing(obj, reference)
            % Defining the reference we want first
            obj.x_des   = reference(1); % This is x
            obj.y_des   = reference(2); % This is y
            obj.z_des   = reference(3); % This is z
            
            % Defining the error, Desired - Measurement , this is the data
            % from the sensor will be inputted
            obj.x_err   = obj.x_des - obj.r(1);
            obj.y_err   = obj.y_des - obj.r(2);
            obj.z_err   = obj.z_des - obj.r(3);

            % Taking the euler value
            phi   = obj.euler(1);
            theta = obj.euler(2);
            psi   = obj.euler(3);
            
            
            % Our Implementation Controller for Position Control through
            % Here from the error of the positon we want, how much
            % acceleration does it needed?
            xddot = (0.5)*(obj.x_err) + (0.05)*(obj.x_err_sum) + (0.01)*(obj.x_err - obj.x_err_prev)/obj.dt;  % Speed x Desired // ex_ddot
            yddot = (0.4)*(obj.y_err) + (0.02)*(obj.y_err_sum) + (0.01)*(obj.y_err - obj.y_err_prev)/obj.dt;  % Speed y Desired // ey_ddot
            zddot = (0.4)*(obj.z_err) + (0.02)*(obj.z_err_sum) + (0.01)*(obj.z_err - obj.z_err_prev)/obj.dt;  % Speed z Desired // ez_ddot

            % Reference Value
            % U1
            %ref(1) = (obj.g+(3.365e-2/obj.m)*obj.x(6)+ ...
            %      zddot)/(cos(obj.euler(1))*cos(obj.euler(2)));
            %ref(1) = (obj.g * obj.m); 
            ref(1) = (obj.m*(0-zddot-obj.g))/(cos(phi)*cos(theta));
    
            % Phi Desired
            ref(2) = asin(...
                          (((xddot+(3.365e-2/obj.m)*obj.dx(1))*sin(obj.euler(3)))...
                         -(( yddot+(3.365e-2/obj.m)*obj.dx(2))*cos(obj.euler(3))))/ref(1)...
                          );
            
            % Theta Desired
            ref(3) = atan(( ((0-xddot)*cos(psi))/(0-zddot+obj.g) )+ ( ((0-yddot)*sin(psi))/(0-zddot + obj.g)));
            % ref(3) = asin((((xddot+ 3.365e-2*obj.dx(1)/obj.m)/ref(1))-(sin(obj.euler(1))*sin(obj.euler(2))))/(cos(obj.euler(1))*cos(obj.euler(2))));
            % Yaw Desired
            ref(4) = 0;
    
            obj.log_des = [obj.log_des;ref*(180/pi),obj.r',obj.u'];

            obj.x_err_prev = obj.x_err;
            obj.y_err_prev = obj.x_err;
            obj.z_err_prev = obj.z_err;

            obj.x_err_sum = obj.x_err_sum + obj.x_err;
            obj.y_err_sum = obj.y_err_sum + obj.y_err;
            obj.z_err_sum = obj.z_err_sum + obj.z_err;
            
            % Update attitude controller
            obj.attitudeCtrl(ref);
            return
        end

    end
end