classdef NullspaceController < handle
    properties
        % Constants with default values
        Kp = 5;
        q_dot_max = [0.4; 0.4; 0.8; 0.8];
        elevation_min = deg2rad(50);
        weight_z = 10;
        delta_q_numeric_diff = 0.001;
        exponential_factor_elevation_cost = 20;
        use_nakamura = true;
        q_min = [-pi/4; -pi/4; -pi; -(5/6) * pi];
        q_max = [pi/4; pi/4; pi; (5/6) * pi];
    end
    
    properties (Access=private)
        delta_matrix
    end
    
    methods
        function obj = NullspaceController(varargin)
            % Handle optional input arguments
            p = inputParser;
            addParameter(p, 'Kp', obj.Kp);
            addParameter(p, 'q_dot_max', obj.q_dot_max);
            addParameter(p, 'elevation_min', obj.elevation_min);
            addParameter(p, 'weight_z', obj.weight_z);
            addParameter(p, 'delta_q_numeric_diff', obj.delta_q_numeric_diff);
            addParameter(p, 'exponential_factor_elevation_cost', obj.exponential_factor_elevation_cost);
            addParameter(p, 'use_nakamura', obj.use_nakamura);
            addParameter(p, 'q_min', obj.q_min);
            addParameter(p, 'q_max', obj.q_max);

            parse(p, varargin{:});
            
            % Assign properties from the parsed inputs
            obj.Kp = p.Results.Kp;
            obj.q_dot_max = p.Results.q_dot_max;
            obj.elevation_min = p.Results.elevation_min;
            obj.weight_z = p.Results.weight_z;
            obj.delta_q_numeric_diff = p.Results.delta_q_numeric_diff;
            obj.exponential_factor_elevation_cost = p.Results.exponential_factor_elevation_cost;
            obj.use_nakamura = p.Results.use_nakamura;
            obj.q_min = p.Results.q_min;
            obj.q_max = p.Results.q_max;

            % Pre-compute and store the perturbation matrix
            num_joints = length(obj.q_dot_max);
            obj.delta_matrix = obj.delta_q_numeric_diff * eye(num_joints);
        end
        
        function q_dot = computeDesiredJointVelocity(obj, sr, x_desired, z_desired, v_desired)
            % Main controller method for velocity control

            % Get current end-effector position and robot configuration once
            q = sr.getQ;
            x_current = SimulatedRobot.forwardKinematicsNumeric(q);
            
            % Compute desired effective workspace velocity
            v_d_eff = obj.computeEffectiveVelocity(x_desired, x_current, v_desired);
            
            % Compute gradients of cost functions H(q) for nullspace tasks
            dHdQ_z = obj.numericDiff(@obj.H_z_desired, q, z_desired);
            dHdQ_elevation = obj.numericDiff(@obj.H_min_shoulder_elevation, q);

            % Combine both gradients to do both tasks 
            dHdQ = obj.combineGoals(dHdQ_z, dHdQ_elevation);
            
            % Compute Jacobian
            J = SimulatedRobot.getJacobianNumeric(q);

            % Compute q_dot
            if obj.use_nakamura
                q_dot = obj.computeQdotNakamura(J, v_d_eff, dHdQ);
            else
                q_dot = obj.computeQdotPseudoinverse(J, v_d_eff, dHdQ);
            end
            
            % Limit joint speeds
            q_dot = obj.limitQdot(q_dot);


            % q_next = q + q_dot * 0.01; % Predicted next joint configuration

            % Check if the predicted next configuration violates the joint limits
            % if ~obj.isWithinJointLimits(q_next)
            %     % Modify q_dot to prevent limit violation
            %     violating_indices = (q_next < obj.q_min) | (q_next > obj.q_max);
            %     q_dot(violating_indices) = 0; % Set the velocities of violating joints to 0. Can be enhanced further.
            % 
            %     % Display a warning for the violating joints
            %     for idx = find(violating_indices)'
            %         fprintf('Warning: Joint %d has reached its limit.\n', idx);
            %     end
            % end

        end
    end

    methods (Access=private)

        % Nakamura method for computing Qdot (curr not working)
        function q_dot = computeQdotNakamura(~, J, u, dHdQ)
                z = -dHdQ';
                B = J * J';
                p = u - J*z;
                lambda = linsolve(B,p);
                q_dot = J'*lambda + z;
        end

        % Pseudoinverse, Nullspaceprojector method for computing Qdot
        function q_dot = computeQdotPseudoinverse(~, J, u, dHdQ)
            % Compute pseudo-inverse
            pinvJ = pinv(J);
            N = (eye(4) - pinvJ * J);
            % Compute joint-space velocity q_dot
            q_dot = pinvJ * u - N * dHdQ';
        end
        
        function v_d_eff = computeEffectiveVelocity(obj, x_desired, x_current, v_desired)
            % Calculate v_d_eff (Driftkompensation)
            % P - Controller, could be expanded to PID
            
            % Compute error
            error = x_desired - x_current;
            % Compute effective workspace velocity
            v_d_eff = error * obj.Kp + v_desired;
            
        end
        
        function H = H_z_desired(~, q, z_desired)
            if isnan(z_desired)
                H = 0;
                return;
            end
            
            z_desired_normalized = z_desired/norm(z_desired);
            g_A_tcp = SimulatedRobot.roty(q(1)) * SimulatedRobot.rotx(q(2)) * SimulatedRobot.rotz(q(3)) * SimulatedRobot.rotx(q(4));
            z_q = g_A_tcp * [0;0;1];
            z_q_normalized = z_q/norm(z_q);
            % H = 1/2 * (z(q) - z_desired)^2 
            H = 0.5 * norm(z_q_normalized - z_desired_normalized)^2;
        end
        
        function H = H_min_shoulder_elevation(obj, q)
            elevation = SimulatedRobot.getShoulderElevation(q);
            distance_to_min_elevation = elevation - obj.elevation_min;
            H = exp(-obj.exponential_factor_elevation_cost * distance_to_min_elevation);
        end

        function dHdQ = combineGoals(obj, dHdQ_z, dHdQ_elevation)
            dHdQ = obj.weight_z * dHdQ_z + dHdQ_elevation;
        end

        function dHdQ = numericDiff(obj, H_method, q, varargin)
            num_joints = length(q);
            
            % Extract delta_q from the delta_matrix
            delta_q = obj.delta_matrix(1,1); % because it's diagonal and all values are the same
            
            % Perturb joint configurations using broadcasting
            q_plus = q + obj.delta_matrix;
            q_minus = q - obj.delta_matrix;
            
            % Modify the H function call depending on varargin
            if isempty(varargin)
                H_modified = @(q) H_method(q);
            else
                H_modified = @(q) H_method(q, varargin{:});
            end

            % Evaluate H for all perturbed configurations
            H_plus = arrayfun(@(col) H_modified(q_plus(:, col)), 1:num_joints);
            H_minus = arrayfun(@(col) H_modified(q_minus(:, col)), 1:num_joints);
            
            % Compute gradient
            dHdQ = (H_plus - H_minus) / (2 * delta_q);
        end

        function q_dot_limited = limitQdot(obj, q_dot)
            % Limits q_dot without changing the rations between entries in q_dot
            max_ratio = max(abs(q_dot) ./ obj.q_dot_max);
            q_dot_limited = q_dot / max(1, max_ratio);
        end

        function is_valid = isWithinJointLimits(obj, q_next)
            is_valid = all(q_next >= obj.q_min) && all(q_next <= obj.q_max);
        end
    end
end
