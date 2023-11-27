classdef NullspaceController < handle
    properties (Access=private)
        % Constants with default values
        Kp = 1;
        % Conifugre maximum absolut joint velocities % RAD/s
        q_dot_max = [0.1;0.1;0.2;0.2];

        weight_z = 0;
        weight_preffered_config = 0.1;
        exponential_factor_joint_limit = 0;
        weight_joint_limit = 0;

        delta_q_numeric_diff = 0.001;

        use_nakamura = true;
        q_min;
        q_max;

    end
    
    properties (Access=private)
        delta_matrix
    end
    
    methods
        function obj = NullspaceController(robot, varargin)
            % Handle optional input arguments
            p = inputParser;
            addParameter(p, 'Kp', obj.Kp);
            addParameter(p, 'q_dot_max', obj.q_dot_max);
            addParameter(p, 'weight_z', obj.weight_z);
            addParameter(p, 'delta_q_numeric_diff', obj.delta_q_numeric_diff);
            addParameter(p, 'use_nakamura', obj.use_nakamura);
            addParameter(p, 'weight_preffered_config', obj.weight_preffered_config);
            addParameter(p, 'exponential_factor_joint_limit', obj.exponential_factor_joint_limit);
            addParameter(p, 'weight_joint_limit', obj.weight_joint_limit);

            parse(p, varargin{:});
            
            % Assign properties from the parsed inputs
            obj.Kp = p.Results.Kp;
            obj.q_dot_max = p.Results.q_dot_max;
            obj.weight_z = p.Results.weight_z;
            obj.delta_q_numeric_diff = p.Results.delta_q_numeric_diff;
            obj.use_nakamura = p.Results.use_nakamura;
            obj.weight_preffered_config = p.Results.weight_preffered_config;
            obj.exponential_factor_joint_limit = p.Results.exponential_factor_joint_limit;
            obj.weight_joint_limit = p.Results.weight_joint_limit;

            % Pre-compute and store the perturbation matrix
            num_joints = length(obj.q_dot_max);
            obj.delta_matrix = obj.delta_q_numeric_diff * eye(num_joints);

            % Extract q_max, q_min from passed SimulatedRobot object
            obj.q_max = robot.joint_limits(:,2);
            obj.q_min = robot.joint_limits(:,1);

        end
        
        function q_dot = computeDesiredJointVelocity(obj, sr, x_desired, z_desired, v_desired)
            % Main controller method for velocity control

            % Get current end-effector position and robot configuration once
            q = sr.getQ;
            x_current = SimulatedRobot.forwardKinematicsNumeric(q);
            
            % Compute desired effective workspace velocity
            v_d_eff = obj.computeEffectiveVelocity(x_desired, x_current, v_desired);
            
            % Compute gradients of cost functions H(q) for nullspace tasks
            dHdQ_joint_limit = obj.numericDiff(@obj.H_joint_limit, q);

            % If a desired z-Axis is provided, then the preffered config is
            % deactivated.
            if isnan(z_desired)
                dHdQ_preferred_config = obj.numericDiff(@obj.H_prefered_joint_configuration, q);
                dHdQ_z = [0,0,0,0];
            else
                dHdQ_z = obj.numericDiff(@obj.H_z_desired, q, z_desired);
                dHdQ_preferred_config = [0,0,0,0];
            end

            % Combine both gradients to do both tasks 
            dHdQ = obj.weight_z * dHdQ_z + obj.weight_preffered_config*dHdQ_preferred_config + obj.weight_joint_limit * dHdQ_joint_limit;

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

            % Ensure compliance with joint angle limits
            q_dot = obj.ensureJointLimitCompliance(q, q_dot);
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

            z_desired_normalized = z_desired/norm(z_desired);
            g_A_tcp = SimulatedRobot.roty(q(1)) * SimulatedRobot.rotx(q(2)) * SimulatedRobot.rotz(q(3)) * SimulatedRobot.rotx(q(4));
            z_q = g_A_tcp * [0;0;1];
            z_q_normalized = z_q/norm(z_q);
            % H = 1/2 * (z(q) - z_desired)^2 
            H = 0.5 * norm(z_q_normalized - z_desired_normalized)^2;

            % fprintf("H-z-desired: %.2f\n", H);

        end

        function H = H_prefered_joint_configuration(obj,q)
            
            for i = 1:length(q)
                if q(i) >= 0
                    q(i) = q(i)/obj.q_max(i);
                else
                    q(i) = q(i)/obj.q_min(i);
                end
            end
                
            H = (q' * q);  % -18 < H < H

            % fprintf("H-preferred-config: %.2f\n", H);
        end

        function H = H_joint_limit(obj, q)

            d_upper = obj.q_max - q;
            d_lower = q - obj.q_min;

            joint_range = obj.q_max - obj.q_min;
        
            norm_d_upper = d_upper ./ joint_range;
            norm_d_lower = d_lower ./ joint_range;
            
            P_upper = exp(-obj.exponential_factor_joint_limit * norm_d_upper);
            P_lower = exp(-obj.exponential_factor_joint_limit * norm_d_lower);
            
            H = sum(P_upper + P_lower);
           
            % fprintf("H-joint-limit: %.2f\n", H);
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

        function q_dot = ensureJointLimitCompliance(obj, q, q_dot)
            % Predicted next joint configuration
            q_next = q + q_dot * 0.01; % Ensure this timestep matches your control loop
        
            % Check if the predicted next configuration violates the joint limits
            for idx = 1:length(q)
                if q_next(idx) < obj.q_min(idx)
                    % If joint is moving towards lower limit and is too close, only allow motion away from the limit
                    if q_dot(idx) < 0
                        q_dot(idx) = 0;
                    end
                    fprintf('Warning: Joint %d is approaching lower limit. Adjusted to: %.2f °.\n', idx, rad2deg(q_next(idx)));
                elseif q_next(idx) > obj.q_max(idx)
                    % If joint is moving towards upper limit and is too close, only allow motion away from the limit
                    if q_dot(idx) > 0
                        q_dot(idx) = 0;
                    end
                    fprintf('Warning: Joint %d is approaching upper limit. Adjusted to: %.2f °.\n', idx, rad2deg(q_next(idx)));
                end
            end
        end

    end
end
