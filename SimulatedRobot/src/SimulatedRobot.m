classdef SimulatedRobot < handle
    % SimulatedRobot is a MATLAB class that represents a robot manipulator 
    % in a 3D environment. It provides methods for computing forward 
    % kinematics, Jacobian matrices and displaying the robot configuration 
    % in a 3D plot. 
    %
    % Each instance of the class has an array of Joint objects representing 
    % the robot's joints, an array of Link objects representing the physical 
    % connections between joints, and an array of Frame objects representing 
    % additional frames of the robot.

    properties
        joints  % An array of Joint objects, defining the joints of the robot
        links   % An array of Link objects, defining the physical connections between joints
        frames % An array of Frame objects, defining additional frames of the robot
        fig % The figure in which everything is visualized

        workspaceResolution = 0.13;
        workspaceTolerance = 0.02;
        joint_limits = [-pi/6, pi/6;  % Currently severly limited by shoulder joint stability
                        -pi/6, pi/6; 
                        -pi, pi; 
                        -(4/6)*pi, (4/6)*pi];

        % Workspace
        boundaryK % A property to store the computed 3D boundary of the workspace
        boundaryVertices % A property to store the vertices of the boundary
    end

    methods
        function obj = SimulatedRobot()
            %% Setup Frames and Joints of the simulated robot
            orig_frame = CustomFrame([0; 0; 0], [], 'Origin');
            joint1 = CustomJoint([0; 0; 83.51], orig_frame, 'Joint 1', 'y');
            joint2 = CustomJoint([0;0;0], joint1, 'Joint 2', 'x');
            joint3 = CustomJoint([0;0;119.35], joint2, 'Joint 3', 'z');
            joint4 = CustomJoint([0;0;163.99], joint3, 'Joint 4', 'x');
            endeffector_frame = CustomFrame([0;0;218.86], joint4, 'Endeffector');
            
            %% Setup Links of the simulated robot
            numLinks = 5; % Number of links
            grayLevels = linspace(0.2, 0.8, numLinks);  % Define a range of grayscale values. Start from 0.2 (dark) to 0.8 (lighter)
            
            link1 = CustomLink(orig_frame, joint1, repmat(grayLevels(1), 1, 3));  % Link 1 with first grayscale value
            link2 = CustomLink(joint1, joint2, repmat(grayLevels(2), 1, 3));  % Link 2 with second grayscale value
            link3 = CustomLink(joint2, joint3, repmat(grayLevels(3), 1, 3));  % and so on...
            link4 = CustomLink(joint3, joint4, repmat(grayLevels(4), 1, 3));
            link5 = CustomLink(joint4, endeffector_frame, repmat(grayLevels(5), 1, 3));

            obj.joints =  [joint1, joint2, joint3, joint4];
            obj.links = [link1, link2, link3, link4, link5];
            obj.frames = [orig_frame, endeffector_frame];

            % calculate workspace
            obj.calculateWorkspace;
        end

        function q = getQ(obj)
            q = vertcat(obj.joints.angle);
        end

        function setQ(obj, q)
            arrayfun(@(joint, angle) joint.setAngle(angle), obj.joints, q');
        end

        function draw(obj, draw_frames)
            % The display method updates and displays all joints and links
            % Activates hold on, no hold off
            obj.ensureFigureExists();

            if draw_frames
                arrayfun(@(x) x.draw, obj.joints);
                arrayfun(@(x) x.draw, obj.frames);
            end
            arrayfun(@(x) x.draw, obj.links); 
        end

        function visualizeWorkspace(obj)
            % Visualize the 3D boundary of the robot's workspace

            obj.ensureFigureExists(); % Ensure figure exists before plotting
        
            % Plot the boundary
            trisurf(obj.boundaryK, obj.boundaryVertices(:, 1), obj.boundaryVertices(:, 2), obj.boundaryVertices(:, 3), 'Facecolor', 'cyan', 'Edgecolor', 'none');
            light('Position', [1 3 2]);
            lighting gouraud
            alpha 0.1  % Make it slightly transparent
        end
   
        function isInside = isPointInWorkspace(obj, point)
            % This method checks if a given point is within the robot's workspace.
        
            % Use the boundaryVertices
            vertices = obj.boundaryVertices;
            isInside = inhull(point', vertices);

        end

        function calculateWorkspace(obj, varargin)

            tic

            %% Get all workspace points

            joint_limits_local = obj.joint_limits;
            resolution_local = obj.workspaceResolution;

            if ~isempty(obj.boundaryK) && nargin == 1
                 disp("Workspace already calculated. Pass additional argument to update.")
                 return
            else
                disp("Calculating Workspace...")
            end
    
            % Generate multi-dimensional grids
            [Q1, Q2, Q3, Q4] = ndgrid(joint_limits_local(1,1):resolution_local:joint_limits_local(1,2), ...
                                      joint_limits_local(2,1):resolution_local:joint_limits_local(2,2), ...
                                      joint_limits_local(3,1):resolution_local:joint_limits_local(3,2), ...
                                      joint_limits_local(4,1):resolution_local:joint_limits_local(4,2));
                                  
            % Reshape the grids to vectors
            Q1 = Q1(:); Q2 = Q2(:); Q3 = Q3(:); Q4 = Q4(:);
        
            % Preallocate workspace based on the number of samples we'll generate
            numSamples = numel(Q1);
            workspace = zeros(3, numSamples);
        
            % Compute positions for each sample
            parfor i = 1:numSamples
                q = [Q1(i); Q2(i); Q3(i); Q4(i)];
                workspace(:, i) = SimulatedRobot.forwardKinematicsNumeric(q);
            end


           %% Reduce the number of workspace points using uniquetol
            uniqueWorkspace = uniquetol(workspace', obj.workspaceTolerance, 'ByRows', true)';
            
            % Extract the x, y, and z coordinates from the unique points
            x_positions = uniqueWorkspace(1, :);
            y_positions = uniqueWorkspace(2, :);
            z_positions = uniqueWorkspace(3, :);
            
            %% Compute a 3D boundary (convex hull) of the robot's reduced workspace.
            K = boundary(x_positions', y_positions', z_positions');
            v = uniqueWorkspace'; % The vertices of the boundary
            
            % Store the boundary
            obj.boundaryK = K;
            obj.boundaryVertices = v;

            disp(toc)
        end

    end
    
    methods (Access = private)

        function closeFigureCallback(obj, src)
            % This callback function will be executed when the figure is being closed.
            % It clears the fig property of the object and then closes the figure.
            obj.fig = [];
            delete(src);
        end

        function ensureFigureExists(obj)
            if isempty(obj.fig) || ~isvalid(obj.fig)
                obj.fig = figure;

                % Set the CloseRequestFcn of the figure
                set(obj.fig, 'CloseRequestFcn', @(src, event) obj.closeFigureCallback(src));

                hold on
                view(-290, 25);
                axis equal;
                xlabel('X');
                ylabel('Y');
                zlabel('Z');
                title('Simulated Robot');
                grid on;

                % Set fixed axis limits
                xlim([-500, 500]);
                ylim([-500, 500]);
                zlim([0, 600]);
            end
        end
    end
    
    
    methods (Static)

        function singularityBool = checkSingularity(q)
            % Check for singularity
            J = SimulatedRobot.getJacobianNumeric(q);
            pinvJ = pinv(J);

            if norm(J)*norm(pinvJ) > 25
                disp('Warning: Close to singularity');
                singularityBool = true;
            else
                singularityBool = false;
            end
        end

        function [elevation] = getShoulderElevation(q)
            % Calculate the elevation of the shoulder joint in spherical
            % coordinates in RAD from q
            elevation = pi/2 - acos(cos(q(2))*cos(q(1)));           
        end

        function [oxE] = forwardKinematicsNumeric(q)
            R = {SimulatedRobot.roty(q(1)), SimulatedRobot.rotx(q(2)), SimulatedRobot.rotz(q(3)), SimulatedRobot.rotx(q(4))};
            x = [    0         0         0         0         0
                     0         0         0         0         0
                83.5100         0  119.3500  163.9900  218.8600];
            oxE = R{1} * (R{2} * (R{3} * (R{4} * x(:,5) + x(:,4)) + x(:,3)) + x(:,2)) + x(:,1);
        end

        function J = getJacobianNumeric(q)
            % Computes the Jacobian matrix numerically, relating joint velocities to end-effector velocities.
            % Uses finite differences on forward kinematics by perturbing joint angles with 'delta_q'. 
            % Numerical methods may offer speed advantages over symbolic ones, but precision can vary.
            
            % Small change in joint angles
            delta_q = 1e-6;
            
            % Initialize Jacobian matrix
            J = zeros(3, 4);
            
            % For each joint angle
            parfor i = 1:4
                % Perturb joint angle i
                q_plus = q;
                q_plus(i) = q_plus(i) + delta_q;
                q_minus = q;
                q_minus(i) = q_minus(i) - delta_q;
                
                % Compute forward kinematics for q_plus
                oxE_plus = SimulatedRobot.forwardKinematicsNumeric(q_plus);
                
                % Compute forward kinematics for q_minus
                oxE_minus = SimulatedRobot.forwardKinematicsNumeric(q_minus);
                
                % Compute derivative
                J(:, i) = (oxE_plus - oxE_minus) / (2 * delta_q);
            end
        end

        %% Definition of the standard rotational matrices
        function rotx = rotx(alpha)
            rotx = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
        end
        
        function roty = roty(beta)
            roty = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
        end
        
        function rotz = rotz(gamma)
            rotz = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];
        end


        %% In Hull John D'Errico
        function in = inhull(testpts,xyz,tess,tol)
            % inhull: tests if a set of points are inside a convex hull
            % usage: in = inhull(testpts,xyz)
            % usage: in = inhull(testpts,xyz,tess)
            % usage: in = inhull(testpts,xyz,tess,tol)
            %
            % arguments: (input)
            %  testpts - nxp array to test, n data points, in p dimensions
            %       If you have many points to test, it is most efficient to
            %       call this function once with the entire set.
            %
            %  xyz - mxp array of vertices of the convex hull, as used by
            %       convhulln.
            %
            %  tess - tessellation (or triangulation) generated by convhulln
            %       If tess is left empty or not supplied, then it will be
            %       generated.
            %
            %  tol - (OPTIONAL) tolerance on the tests for inclusion in the
            %       convex hull. You can think of tol as the distance a point
            %       may possibly lie outside the hull, and still be perceived
            %       as on the surface of the hull. Because of numerical slop
            %       nothing can ever be done exactly here. I might guess a
            %       semi-intelligent value of tol to be
            %
            %         tol = 1.e-13*mean(abs(xyz(:)))
            %
            %       In higher dimensions, the numerical issues of floating
            %       point arithmetic will probably suggest a larger value
            %       of tol.
            %
            %       DEFAULT: tol = 0
            %
            % arguments: (output)
            %  in  - nx1 logical vector
            %        in(i) == 1 --> the i'th point was inside the convex hull.
            %  
            % Example usage: The first point should be inside, the second out
            %
            %  xy = randn(20,2);
            %  tess = convhulln(xy);
            %  testpoints = [ 0 0; 10 10];
            %  in = inhull(testpoints,xy,tess)
            %
            % in = 
            %      1
            %      0
            %
            % A non-zero count of the number of degenerate simplexes in the hull
            % will generate a warning (in 4 or more dimensions.) This warning
            % may be disabled off with the command:
            %
            %   warning('off','inhull:degeneracy')
            %
            % See also: convhull, convhulln, delaunay, delaunayn, tsearch, tsearchn
            %
            % Author: John D'Errico
            % e-mail: woodchips@rochester.rr.com
            % Release: 3.0
            % Release date: 10/26/06
            
            % get array sizes
            % m points, p dimensions
            p = size(xyz,2);
            [n,c] = size(testpts);
            if p ~= c
              error 'testpts and xyz must have the same number of columns'
            end
            if p < 2
              error 'Points must lie in at least a 2-d space.'
            end
            
            % was the convex hull supplied?
            if (nargin<3) || isempty(tess)
              tess = convhulln(xyz);
            end
            [nt,c] = size(tess);
            if c ~= p
              error 'tess array is incompatible with a dimension p space'
            end
            
            % was tol supplied?
            if (nargin<4) || isempty(tol)
              tol = 0;
            end
            
            % build normal vectors
            switch p
              case 2
                % really simple for 2-d
                nrmls = (xyz(tess(:,1),:) - xyz(tess(:,2),:)) * [0 1;-1 0];
                
                % Any degenerate edges?
                del = sqrt(sum(nrmls.^2,2));
                degenflag = (del<(max(del)*10*eps));
                if sum(degenflag)>0
                  warning('inhull:degeneracy',[num2str(sum(degenflag)), ...
                    ' degenerate edges identified in the convex hull'])
                  
                  % we need to delete those degenerate normal vectors
                  nrmls(degenflag,:) = [];
                  nt = size(nrmls,1);
                end
              case 3
                % use vectorized cross product for 3-d
                ab = xyz(tess(:,1),:) - xyz(tess(:,2),:);
                ac = xyz(tess(:,1),:) - xyz(tess(:,3),:);
                nrmls = cross(ab,ac,2);
                degenflag = false(nt,1);
              otherwise
                % slightly more work in higher dimensions, 
                nrmls = zeros(nt,p);
                degenflag = false(nt,1);
                for i = 1:nt
                  % just in case of a degeneracy
                  % Note that bsxfun COULD be used in this line, but I have chosen to
                  % not do so to maintain compatibility. This code is still used by
                  % users of older releases.
                  %  nullsp = null(bsxfun(@minus,xyz(tess(i,2:end),:),xyz(tess(i,1),:)))';
                  nullsp = null(xyz(tess(i,2:end),:) - repmat(xyz(tess(i,1),:),p-1,1))';
                  if size(nullsp,1)>1
                    degenflag(i) = true;
                    nrmls(i,:) = NaN;
                  else
                    nrmls(i,:) = nullsp;
                  end
                end
                if sum(degenflag)>0
                  warning('inhull:degeneracy',[num2str(sum(degenflag)), ...
                    ' degenerate simplexes identified in the convex hull'])
                  
                  % we need to delete those degenerate normal vectors
                  nrmls(degenflag,:) = [];
                  nt = size(nrmls,1);
                end
            end
            
            % scale normal vectors to unit length
            nrmllen = sqrt(sum(nrmls.^2,2));
            % again, bsxfun COULD be employed here...
            %  nrmls = bsxfun(@times,nrmls,1./nrmllen);
            nrmls = nrmls.*repmat(1./nrmllen,1,p);
            
            % center point in the hull
            center = mean(xyz,1);
            
            % any point in the plane of each simplex in the convex hull
            a = xyz(tess(~degenflag,1),:);
            
            % ensure the normals are pointing inwards
            % this line too could employ bsxfun...
            %  dp = sum(bsxfun(@minus,center,a).*nrmls,2);
            dp = sum((repmat(center,nt,1) - a).*nrmls,2);
            k = dp<0;
            nrmls(k,:) = -nrmls(k,:);
            
            % We want to test if:  dot((x - a),N) >= 0
            % If so for all faces of the hull, then x is inside
            % the hull. Change this to dot(x,N) >= dot(a,N)
            aN = sum(nrmls.*a,2);
            
            % test, be careful in case there are many points
            in = false(n,1);
            
            % if n is too large, we need to worry about the
            % dot product grabbing huge chunks of memory.
            memblock = 1e6;
            blocks = max(1,floor(n/(memblock/nt)));
            aNr = repmat(aN,1,length(1:blocks:n));
            for i = 1:blocks
               j = i:blocks:n;
               if size(aNr,2) ~= length(j)
                  aNr = repmat(aN,1,length(j));
               end
               in(j) = all((nrmls*testpts(j,:)' - aNr) >= -tol,1)';
            end


        end
    end
end


    
