classdef Rotation
    %ROTATION Summary of this class goes here
    %   Detailed explanation goes here
    properties
        tx, ty, tz, Rx, Ry, Rz, Rzyx, Rxyz, Txyz, tolerance
        sym_eu_ang_v  % Symbolic Euler Angle Vector composed from tx, ty, tz
    end
    
    methods
        function obj = Rotation()
            obj.tx = sym('tx');
            assume(obj.tx,'real')
            obj.ty  = sym('ty');
            assume(obj.ty,'real')
            obj.tz  = sym('tz');
            assume(obj.tz,'real')
            obj.sym_eu_ang_v = [obj.tx; obj.ty; obj.tz];
            tx_ = obj.tx;
            ty_ = obj.ty;
            tz_ = obj.tz;
            obj.Rx = [1 0 0; 0 cos(tx_) -sin(tx_); 0 sin(tx_) cos(tx_)];
            obj.Ry = [cos(ty_) 0 sin(ty_); 0 1 0; -sin(ty_) 0 cos(ty_)];
            obj.Rz = [cos(tz_) -sin(tz_) 0; sin(tz_) cos(tz_) 0; 0 0 1];
            obj.Rzyx = obj.Rz*obj.Ry*obj.Rx;
            obj.Rxyz = obj.Rx*obj.Ry*obj.Rz;
            obj.Txyz = [cos(tx_) 0 -cos(ty_)*sin(tx_); ...
                0 1 sin(ty_); sin(tx_) 0 cos(ty_)*cos(tx_)]
            obj.tolerance = .0001;
        end
        function res = AisalmostB(obj, A, B)
            res = isalmost(A,B, obj.tolerance, true);
        end         
        function res = assertProperRotationMatrix(obj, X)
            assert (size(X, 1) == size(X, 2));                         % square
            sz = size(X, 1);
            assert(obj.AisalmostB(det(X),1));                       % proper
            assert(obj.AisalmostB(X*X', eye(sz)));               % orthogonal
            assert(obj.AisalmostB(trace(X), sum(eig(X)))); % Derived property
            res = 1; % True
        end
        %%%%
        %   getRotationXMinusY(~, X, Y)
        %   
        %   Returns the rotation matrix that multiplied into Y, gives X.
        %   i.e. adding this rotation to Y gives X. X is usually the
        %   rotation you want to acheive, and Y is the starting rotation.
        %   Both X and Y must be in the same frame of reference for
        %   all this to make any sense.
        %%%%
        function matrix = getRotationXMinusY(obj, X, Y)
            obj.assertProperRotationMatrix(X);
            obj.assertProperRotationMatrix(Y);
            matrix = Y'*X;
        end
         %%%%
        %   getDesiredRotation3D(~, type, t, v, tz_desired)
        %   
        %   t is the vector you want to align with.
        %   tz_desired is the desired yaw angle
        %   v is the vector you want to to align against t
        %   
        %   Returns the rotation matrix 
        %%%%
        function matrix = getDesiredRotation3D(obj, type, t, v, tz_desired)
            if strcmp(type,  'ZYX') == 0
                error('Unsupported type')
            end
            % Normalize ‘t’. This is the lhs of the equation
            rhs = t/ norm(t);
            % Let Rdes = the generic Rzyx rotation matrix. I think this can 
            % be any of the symbolic matrices, such as Ryzx etc, but not
            % sure that's ok theory-wise.
            Rdes = obj.Rzyx;
            % Simplify Rdes using the value of one of the symbols: the rotation about z axis, or the ‘psi’
            Rdes = subs(Rdes, obj.tz, tz_desired);
            % Find LHS of equation (b3 rotated by Rdes)
            lhs = Rdes * v/norm(v);
            % solve using lhs and rhs
            solution = solve(lhs == rhs);
            % You may get multiple solutions. Say ty = pi/6, tx = 0 is one solution. Replace in generic rotation matrix
            Rdes = obj.getVectorRotation('ZYX', solution.tx(1), solution.ty(1), tz_desired);
            % Convert the symbolic form into real matrix
            matrix = vpa(Rdes);
        end
        function matrix = getOmegaRotation(obj, type,tx, ty, tz)
            if strcmp(type,  'ZYX') == 0
                error('Unsupported type')
            end
            matrix =  subs(obj.Txyz, [obj.tx, obj.ty, obj.tz], [tx, ty, tz]);
        end
        function matrix = getVectorRotation(obj, type,tx, ty, tz)
            matrix = [];
            switch type 
                case 'ZYX'
                    matrix = subs(obj.Rzyx, [obj.tx, obj.ty, obj.tz], [tx, ty, tz]);
                case 'XYZ'
                    matrix = subs(obj.Rxyz, [obj.tx, obj.ty, obj.tz], [tx, ty, tz]);
                otherwise
                    error('Unsupported Operation')
            end
        end
        %%%%
        %   getAngularBodyVelFromEulerRates(obj, type, euler_rate_vec)
        %   
        %   euler_rate_vec The vector of change rates of euler angles in
        %   fixed frame
        %   eu_ang_v          The vector of euler angles.
        %   
        %   Returns the angular velocity in body frame
        %%%%
        function ang_vel_body_vec = getAngularBodyVelFromEulerRates(obj, type, euler_rate_vec, eu_ang_v)
            if strcmp(type,  'ZYX') == 0
                error('Unsupported type')
            end
            %Define a1, a2, a3 (unit vectors in inertial frame)
            a1 = [1; 0; 0];         a2 = [0; 1; 0];         a3 = [0; 0;1];
            translation_matrix = [obj.Ry'*a1, a2, obj.Ry'*obj.Rx'*a3];
            ang_vel_body_vec = translation_matrix * euler_rate_vec;
            ang_vel_body_vec = subs(ang_vel_body_vec, obj.sym_eu_ang_v, eu_ang_v);
        end
         %%%%
        %   getAngularFixedVelFromEulerRates(obj, type, euler_rate_vec)
        %   
        %   euler_rate_vec is the vector of change rates of euler angles in
        %   fixed frame
        %   
        %   Returns the angular velocity in fixed frame
        %%%%
        function ang_vel_body_vec = getAngularFixedVelFromEulerRates(obj, type, euler_rate_vec, eu_ang_v)
            if strcmp(type,  'ZYX') == 0
                error('Unsupported type')
            end
            %Define a1, a2, a3 (unit vectors in inertial frame)
            a1 = [1; 0; 0];         a2 = [0; 1; 0];         a3 = [0; 0;1];
            translation_matrix = obj.Rzyx' * [obj.Ry'*a1, a2, obj.Ry'*obj.Rx'*a3];
            ang_vel_body_vec = translation_matrix * euler_rate_vec;
            ang_vel_body_vec = subs(ang_vel_body_vec, obj.sym_eu_ang_v, eu_ang_v);
        end
    end
end

