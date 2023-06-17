%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos
%%%%  Code modified by Akshet Patel (Q3A)
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef Drone < handle
    properties (Constant)
        %width, length, height offset between centre and rotors
        body = [2 2 0.0];

        % size of floating window that follows drone
        axis_size = 2.;

        %colours of each component of drone model
        colours = [[.8 .3 .1];[.2 .2 .5];[.8 .1 .3];[.9 .6 .8];[.9 .2 .4]];

        %Follows the drone within the figure
        %Don't use if you are simulating more than one drone!
        %Switch to false to see the overall world view
        drone_follow = false;
    end
    properties
        % %time interval for simulation (seconds)
        time_interval

        %axis to draw on
        axis

        %length of one side of the flight arena
        spaceDim

        %limits of flight arena
        spaceLimits

        %drone position
        pos
        pos_dot

        %drone rotation
        R
        omega
        th
        thd

        %Simulation time
        time

        %parameter to start drone in random position
        pos_offset

        %number of drones
        num_drones

        %mass
        m

        %gravity
        g

        %frictional coefficient
        kd

        %propeller constant
        k
        b

        %Full state feedback variables
        eigenvalues
        K
        reference_coordinates_list %reference values for drone to follow
        checkpoints %boolean vector to see if checkpoints are reached

        %length from each propeler to the centre
        L

        %quadcopter rotatinal inertia matrix
        I

        % propeller inputs
        inputs
        equ_inputs
        pre_inputs

        % LTI parameters
        Ad
        Bd

        % state
        state
        delta_x

        % keeping track of position, orientation and time for plots
        xyzpos
        orientation
        times

        % time stamp for the hovering task
        time_stamp

        % stop sim when mission completes
        operations_completed_successfully

        %counter for intermediate points
        count
    end
    methods

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %INSTANTIATION OF CLASS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drone(axis, spaceDim, num_drones, time_interval)
            if nargin > 1
                obj.time_interval = time_interval;

                obj.axis = axis;

                obj.spaceDim = spaceDim;

                obj.spaceLimits = [(-spaceDim/2)+10 (spaceDim/2)-10 ...
                    (-spaceDim/2)+10 ...
                    (spaceDim/2)-10 ...
                    10 ...
                    spaceDim-10];

                obj.pos = [0;0;0]; %inital pos

                obj.pos_dot = zeros(3,1);

                obj.R = eye(3);

                obj.th =  zeros(3,1);

                obj.thd =  zeros(3,1);

                obj.omega = [1, 0, -sin(obj.th(2));
                    0, cos(obj.th(1)), cos(obj.th(2)) * sin(obj.th(1));
                    0, -sin(obj.th(1)), cos(obj.th(2)) * cos(obj.th(1))
                    ] * obj.thd;

                obj.time = 0;

                obj.num_drones = num_drones;

                %constants provided

                %mass
                obj.m = 0.2;

                %gravitational acceleration
                obj.g = 9.8;

                %friction constant
                obj.kd = 0.1;

                % propeller constants
                obj.k = 1;

                obj.L = 0.2;

                obj.b = 0.1;

                obj.count = 1;

                % Rotational Inertia Matrix
                obj.I = [1 , 0, 0;
                    0, 1, 0;
                    0, 0, 0.5];

                % inputs
                obj.equ_inputs = solveInputs(zeros(1,3), obj);
                obj.inputs = solveInputs(zeros(1,3), obj);
                obj.pre_inputs = obj.equ_inputs;

                % state space
                obj.state = [obj.pos; obj.pos_dot; obj.th; obj.omega];
                obj.delta_x = zeros(12,1);


                %%
                % Circular trajectory
                % Define the center and radius of the circle
                center = [2.5 5 5];
                radius = 2.5;
                z = 5;
                % Define the start and end angles of the circular trajectory
                startAngle = 0;
                endAngle = 2*pi;
                
                % Use linspace to generate a set of evenly spaced points along the circumference of the circle
                angles = linspace(startAngle, endAngle, 19);
                
                % Compute the x and y coordinates of the points on the circumference
                x = flip(center(1) + radius*cos(angles));
                y = flip(center(2) + radius*sin(angles));
                z = z*ones(size(x));

                % concatenate the coodinates to form a 3x19 list of reference coordinates 
                coordinates = vertcat(x, y, z);

                % last landing cooridnate
                newPoint = [5; 5; 0];

                %concatenate the landing coorindate to form a list of 3x20 reference coordinates
                coordinates = cat(2, coordinates, newPoint);

                %storing the list in the reference_coordinates_list object
                obj.reference_coordinates_list = coordinates;
                
                %%
                %Full state feedback variables
                %Pick arbitrary values less than one
                obj.eigenvalues = [0.9    0.91    0.983    0.994    0.988   0.806    0.988   0.991    0.972    0.876  0.790   0.824];
                obj.K = zeros(4,12); %Initialise K matrix to be found when matrices Ad and Bd are calculated later

                % add zero padding to make sure each column is of length 12
                % to be consistent with 12x1 x vector
                obj.reference_coordinates_list = [obj.reference_coordinates_list;zeros(9,20)];
                %checkpoints (false until conditions are met)
                obj.checkpoints = zeros(20,1);

                % Keeping track of position and orientation
                obj.xyzpos = [];
                obj.orientation = [];
                obj.times = [];

                % LTI parameters
                [obj.Ad, obj.Bd] = discrete_quadcopter_model(obj);

                obj.time_stamp = obj.time;

                obj.operations_completed_successfully = false;

            else
                error('Drone not initialised correctly')
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %DRAWING OF DRONE TO FIGURE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw(obj)
            %how big should the moving window be
            cL = obj.axis_size;

            %set to false if you want to see world view
            if(obj.drone_follow)
                axis([obj.pos(1)-cL obj.pos(1)+cL obj.pos(2)-cL obj.pos(2)+cL obj.pos(3)-cL obj.pos(3)+cL]);
            end

            %create middle sphere
            [X, Y, Z] = sphere(8);
            X = (obj.body(1)/5.).*X + obj.pos(1);
            Y = (obj.body(1)/5.).*Y + obj.pos(2);
            Z = (obj.body(1)/5.).*Z + obj.pos(3);
            s = surf(obj.axis,X,Y,Z);
            set(s,'edgecolor','none','facecolor',obj.colours(1,:));

            %create side spheres
            %front, right, back, left
            hOff = obj.body(3)/2;
            Lx = obj.body(1)/2;
            Ly = obj.body(2)/2;
            rotorsPosBody = [...
                0    Ly    0    -Ly;
                Lx    0    -Lx   0;
                hOff hOff hOff hOff];
            rotorsPosInertial = zeros(3,4);
            for i = 1:4
                rotorPosBody = rotorsPosBody(:,i);
                rotorsPosInertial(:,i) = bodyToInertial(obj,rotorPosBody);
                [X, Y, Z] = sphere(8);
                X = (obj.body(1)/8.).*X + obj.pos(1) + rotorsPosInertial(1,i);
                Y = (obj.body(1)/8.).*Y + obj.pos(2) + rotorsPosInertial(2,i);
                Z = (obj.body(1)/8.).*Z + obj.pos(3) + rotorsPosInertial(3,i);
                s = surf(obj.axis,X,Y,Z);
                set(s,'edgecolor','none','facecolor',obj.colours(i+1,:));
            end
            obj.axis.Title.String = ['Sim Time = ',num2str(obj.time,'%f'),' seconds'];
        end

        function vectorInertial = bodyToInertial(obj, vectorBody)
            vectorInertial = obj.R*vectorBody;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% SIMULATION FUNCTIONS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function simulation_dynamics(obj)
            dt = obj.time_interval;

            %Compute new pose
            obj.omega = obj.omega +dt * angular_acceleration(obj);
            obj.thd = omega2thetadot(obj);
            obj.th = obj.th +dt * obj.thd;
            obj.pos_dot = obj.pos_dot + dt * acceleration(obj);
            obj.pos = obj.pos + dt * obj.pos_dot;
            obj.omega = thetadot2omega(obj);
        end

        %%
        function inputs = solveInputs(a_target, obj)
            %Inputs; a_target: The target z acceleration of the drone
            %obj: The global variables specified at beginning of file

            %Outputs; Inputs: the gamma_1 to gamma_4 squared angular
            %velocity of each propeller

            gravity=[0; 0; -obj.g];
            Fd = -obj.kd * obj.pos_dot;
            obj.R = rotation(obj);
            T = Fd+ obj.m * (a_target-gravity) / (cos(obj.th(2))*cos(obj.th(1)));

            inputs = zeros(4, 1);
            inputs(:) = T(3)/4;
        end
        
        %% discretization function
        function [Ad, Bd] = discrete_quadcopter_model(obj)

            % Define symbolic variables for state and inputs
            syms x [12,1]
            syms xd [12,1]
            syms u [4,1]
        
            % define parameters
            gravity = [0; 0; -obj.g];
            Tb = [0; 0; obj.k * sum(u)];
            Fd = -obj.kd * [x(4); x(5); x(6)] ;
        
            % compute dynamics
            phi = x(7);
            theta = x(8);
            psi = x(9);
        
            % Compute rotation matrices
            Rx = [1 0 0;
                0 cos(phi) -sin(phi); ... 
                0 sin(phi) cos(phi)];
            Ry = [ cos(theta) 0 sin(theta);
                0 1 0; ...
                -sin(theta) 0 cos(theta)];
            Rz = [cos(psi) -sin(psi) 0; ... 
                sin(psi) cos(psi) 0; ... 
                0 0 1];
            R = Rz*Ry*Rx;
        
            % Compute torques
            tau = [
                obj.L * obj.k * (u(1) - u(3))
                obj.L * obj.k * (u(2) - u(4))
                obj.b * (u(1) - u(2) + u(3) - u(4))];
        
            % Compute time derivatives
            xd(1:3) = x(4:6);
            xd(4:6) = gravity +(1/obj.m) * R *(Tb) + (1/obj.m) *Fd;
            xd(7:9) = [1, 0, -sin(x(8));
                0, cos(x(7)), cos(x(8)) * sin(x(7));
                0, -sin(x(7)), cos(x(8)) * cos(x(7))] \ x(10:12);
            xd(10:12) = (obj.I) \ (tau - cross(x(10:12), obj.I * x(10:12)));
        
            % Compute Jacobians 
            Aj = jacobian(xd,x);
            Bj = jacobian(xd,u);
        
            % Evaluate Jacobians
            A =  subs(Aj, [x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12], obj.state.');
            A = double (subs(A, u, obj.inputs));
            B = double (subs(Bj, [x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12], obj.state.'));
            C = eye(size(A));
            D = zeros(size(B));
            
            cont_sys = ss(A,B,C,D);
            disc_sys = c2d(cont_sys,obj.time_interval,'zoh');
        
            Ad = disc_sys.A;
            Bd = disc_sys.B;
            
        end
        %% Acceleration
        function a = acceleration(obj)
            % Inputs; obj: Global varibales at beginning of file
            % g: acceleration due to gravity (-9.8m/s/s)
            % kd: Drag coefficient
            % pos_dot: velocity of the drone
            %Outputs; a: Acceleration of the drone

            gravity=[0; 0; -obj.g];
            obj.R = rotation(obj);
            T = obj.R * thrust(obj.inputs, obj);
            Fd = -obj.kd * obj.pos_dot;
            a = gravity + 1 / obj.m * T + Fd;
        end

        %% Thrust
        function T = thrust(input, obj)
            %Inputs are for Wi^2
            %Outputs T Thrust Vector
            T= [0; 0; obj.k * sum(input)];
        end
        %% Torques
        function tau = torques(inputs, obj)
            %Inputs are values for wi^2
            % obj global variables
            % Outputs tau Torque Vector

            tau = [
                obj.L * obj.k * (inputs(1) - inputs(3))
                obj.L * obj.k * (inputs(2) - inputs(4))
                obj.b * (inputs(1) - inputs(2) + inputs(3) - inputs(4))];
        end

        %% Rotation
        function R = rotation(obj)
            % Inputs phi(roll), theta (pitch), psi (yaw)
            % Outputs Rotation Matrix R
            phi = obj.th(1);
            theta = obj.th(2);
            psi = obj.th(3);

            Rx = [1 0 0;
                0 cos(phi) -sin(phi); ...
                0 sin(phi) cos(phi)];
            Ry = [ cos(theta) 0 sin(theta);
                0 1 0; ...
                -sin(theta) 0 cos(theta)];
            Rz = [cos(psi) -sin(psi) 0; ...
                sin(psi) cos(psi) 0; ...
                0 0 1];
            R = Rz*Ry*Rx;
        end

        %%
        function omega = thetadot2omega(obj)
            % Inputs: thetadot, first derivative of roll, pitch, yaw
            % Outputs R rotation Matrix
            omega = [1, 0, -sin(obj.th(2));
                0, cos(obj.th(1)), cos(obj.th(2)) * sin(obj.th(1));
                0, -sin(obj.th(1)), cos(obj.th(2)) * cos(obj.th(1))] * obj.thd;
        end

        %%
        function omegadot = angular_acceleration(obj)
            % Inputs: Drone squared angular velocity inputs
            % I quadcopter rotational inertia matrix
            % Outputs: omegadot
            tau = torques(obj.inputs, obj);
            omegadot = (obj.I) \ (tau - cross(obj.omega, obj.I * obj.omega));
        end

        %%
        function thd = omega2thetadot(obj)
            thd = [1, 0, -sin(obj.th(2));
                0, cos(obj.th(1)), cos(obj.th(2)) * sin(obj.th(1));
                0, -sin(obj.th(1)), cos(obj.th(2)) * cos(obj.th(1))] \ obj.omega;
        end

        %% Full State Feedback Controller 
        function fsfInput(ref_pos, obj)
            %Calculates inputs, u, using full-state feedback when given the
            %reference position and the object
            %calculate K matrix
            obj.K = place(obj.Ad,obj.Bd,obj.eigenvalues);
            %find error (i.e. difference between x and reference position)
            e = obj.state - ref_pos;
            %adjust input relative to the equilibrium inputs
            obj.inputs = obj.equ_inputs - obj.K * e;
            obj.inputs = obj.inputs(:);
        end
        %%
        function update(obj)
            tol = 0.1; %tolerance for controller
            %update simulation time
            obj.time = obj.time + obj.time_interval;
            if obj.checkpoints(1) == 0
                ref_pos = obj.reference_coordinates_list(:,1);
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(1) = 1;
                    obj.time_stamp = obj.time;
                end
            elseif obj.checkpoints(1) == 1 && obj.checkpoints(2) == 0
                if obj.time < obj.time_stamp+5
                    disp ('Staying at [5 5 5] for 5 seconds');
                    ref_pos = obj.reference_coordinates_list(:,1);
                    fsfInput(ref_pos, obj);
                    simulation_dynamics(obj);
                else    
                    ref_pos = obj.reference_coordinates_list(:,2);
                    fsfInput(ref_pos, obj);
                    simulation_dynamics(obj);
                    if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                        obj.checkpoints(2) = 1;
                    end
                end
            elseif obj.checkpoints(2) == 1 && obj.checkpoints(3) == 0 
                ref_pos = obj.reference_coordinates_list(:,3);
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(3) = 1;
                end
            elseif obj.checkpoints(3) == 1 && obj.checkpoints(4) == 0 
                ref_pos = obj.reference_coordinates_list(:,4);
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(4) = 1;
                end
            elseif obj.checkpoints(4) == 1 && obj.checkpoints(5) == 0 
                ref_pos = obj.reference_coordinates_list(:,5);
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(5) = 1;
                end
            elseif obj.checkpoints(5) == 1 && obj.checkpoints(6) == 0 
                ref_pos = obj.reference_coordinates_list(:,6);
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(6) = 1;
                end
            elseif obj.checkpoints(6) == 1 && obj.checkpoints(7) == 0 
                ref_pos = obj.reference_coordinates_list(:,7);
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(7) = 1;
                end
            elseif obj.checkpoints(7) == 1 && obj.checkpoints(8) == 0 
                ref_pos = obj.reference_coordinates_list(:,8);
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(8) = 1;
                end
            elseif obj.checkpoints(8) == 1 && obj.checkpoints(9) == 0 
                ref_pos = obj.reference_coordinates_list(:,9);
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(9) = 1;
                end
            elseif obj.checkpoints(9) == 1 && obj.checkpoints(10) == 0 
                ref_pos = obj.reference_coordinates_list(:,10);
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(10) = 1;
                end
            elseif obj.checkpoints(10) == 1 && obj.checkpoints(11) == 0 
                ref_pos = obj.reference_coordinates_list(:,11);
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(11) = 1;
                end
            elseif obj.checkpoints(11) == 1 && obj.checkpoints(12) == 0 
                ref_pos = obj.reference_coordinates_list(:,12);
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(12) = 1;
                end
            elseif obj.checkpoints(12) == 1 && obj.checkpoints(13) == 0 
                ref_pos = obj.reference_coordinates_list(:,13);
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(13) = 1;
                end
            elseif obj.checkpoints(13) == 1 && obj.checkpoints(14) == 0 
                ref_pos = obj.reference_coordinates_list(:,14);
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(14) = 1;
                end
            elseif obj.checkpoints(14) == 1 && obj.checkpoints(15) == 0 
                ref_pos = obj.reference_coordinates_list(:,15);
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(15) = 1;
                end
            elseif obj.checkpoints(15) == 1 && obj.checkpoints(16) == 0 
                ref_pos = obj.reference_coordinates_list(:,16);
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(16) = 1;
                end
            elseif obj.checkpoints(16) == 1 && obj.checkpoints(17) == 0 
                ref_pos = obj.reference_coordinates_list(:,17);
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(17) = 1;
                end
            elseif obj.checkpoints(17) == 1 && obj.checkpoints(18) == 0 
                ref_pos = obj.reference_coordinates_list(:,18);
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(18) = 1;
                end
            elseif obj.checkpoints(18) == 1 && obj.checkpoints(19) == 0 
                ref_pos = obj.reference_coordinates_list(:,19);
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(19) = 1;
                end                
            elseif obj.checkpoints(19) == 1 && obj.checkpoints(20) == 0 
                ref_pos = obj.reference_coordinates_list(:,20);
                disp ('Landing at [5 5 0]')
                fsfInput(ref_pos, obj);
                simulation_dynamics(obj);
                
                % velocity restriction for landing
                velocity_linear = sqrt( sum(obj.pos_dot.^2));
                if velocity_linear > 0.1
                    obj.pos_dot = 0.1 * obj.pos_dot / velocity_linear;
                end
                
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(20) = 1;
                    obj.inputs = zeros(4,1);
                    disp ('Operations Completed successfully!');
                    obj.time_stamp = obj.time;
                end

            elseif obj.checkpoints(20) == 1 && obj.time > obj.time_stamp + 3
                disp('Waiting for three seconds to shut down the systems.')
                simulation_dynamics(obj);
                obj.operations_completed_successfully = true;   
            end

            % altitude restrcition
            if obj.pos(3)<0 
                obj.pos(3) = 0;
            end
            disp('current position:')
            disp(obj.pos)
            
            % update state
            obj.state = [obj.pos;obj.pos_dot;obj.th;obj.omega];

            % draw drone to figure
            draw(obj);

            % keep track of current and previous position, orientation and
            % time
            obj.xyzpos = [obj.xyzpos,obj.pos];
            obj.orientation = [obj.orientation, obj.th];
            obj.times = [obj.times, obj.time];
        end
    end
end
