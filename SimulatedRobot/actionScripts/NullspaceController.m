classdef NullspaceController < handle
    properties
        % Constants
        Kp = 5;
        epsilon = 5;
        u_max = 100;
        q_dot_max = [0.4; 0.4; 0.8; 0.8];
        elevation_min = deg2rad(50);
        weight_z = 10;
    end
    
    methods
        function obj = NullspaceController()
            % Empty constructor if needed
        end
        
        function q_dot = computeDesiredJointVelocity(obj, sr, x_desired, z_desired, v_desired)
            % Get current end-effector position, Jacobian
            x_current = SimulatedRobot.forwardKinematicsNumeric(sr.getQ);
            
            % Compute the Jacobian for the current robot configuration
            J = SimulatedRobot.getJacobianNumeric(sr.getQ);
            
            % Compute pseudo-inverse
            pinvJ = pinv(J);
            
            % Compute error
            error = x_desired - x_current;
            
            % Compute effective workspace velocity
            v_d_eff = error * obj.Kp + v_desired;
            
            % Compute control input (P)
            u = v_d_eff;
            
            % Limit u [mm/s]
            % u = obj.u_max * u/norm(u);
            
            % Calculate Nullspace Operator
            N = (eye(4) - pinvJ * J);
            q = sr.getQ;

            dHdQ_z = obj.numericDiff(@(q, param) obj.H_z_desired(q, param), q, z_desired);
            dHdQ_elevation = obj.numericDiff(@(q, param) obj.H_min_shoulder_elevation(q), q, []);

                
            % Combine both goals
            dHdQ = obj.weight_z * dHdQ_z + dHdQ_elevation;
            
            % Compute joint-space velocity q_dot
            q_dot = pinvJ * u - N * dHdQ';
            
            % Limit joint speeds [rad/s]
            q_dot = obj.limitQdot(q_dot);
        end
        
        function H = H_z_desired(obj, q, z_desired)
            if isnan(z_desired)
                H = 0;
                return;
            end
            
            z_desired_normalized = z_desired/norm(z_desired);
            g_A_tcp = SimulatedRobot.roty(q(1)) * SimulatedRobot.rotx(q(2)) * SimulatedRobot.rotz(q(3)) * SimulatedRobot.rotx(q(4));
            z_q = g_A_tcp * [0;0;1];
            z_q_normalized = z_q/norm(z_q);
            H = 0.5 * norm(z_q_normalized - z_desired_normalized)^2;
        end
        
        function H = H_min_shoulder_elevation(obj, q, ~)
            elevation = SimulatedRobot.getShoulderElevation(q);
            distance_to_min_elevation = elevation - obj.elevation_min;
            a = 20;
            H = exp(-a * distance_to_min_elevation);
        end
        
        function dHdQ = numericDiff(obj, H, q, param)
            % Define a small change in joint angles
            delta_q = 0.001;
        
            % Initialize dHdQ
            dHdQ = zeros(1,4); % Column vector initialization
        
            % For each joint angle
            for i = 1:4
                % Create a copy of q for perturbation
                q_plus = q;
                q_minus = q;
            
                % Add and subtract small change to joint i
                q_plus(i) = q_plus(i) + delta_q;
                q_minus(i) = q_minus(i) - delta_q;
            
                % Compute H(q) with the provided param
                H_plus = H(q_plus, param);
                H_minus = H(q_minus, param);

            
                % Compute derivative
                dHdQ(1,i) = (H_plus - H_minus) / (2 * delta_q);
            end
        end

        
        function q_dot_limited = limitQdot(obj, q_dot)
            ratio = abs(q_dot) ./ obj.q_dot_max;
            max_ratio = max(ratio);
            if max_ratio > 1
                q_dot_limited = q_dot / max_ratio;
            else
                q_dot_limited = q_dot;
            end
        end
    end
end
