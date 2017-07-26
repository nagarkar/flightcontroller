classdef Coefficients < handle
    %ROTATION Summary of this class goes here
    %   Detailed explanation goes here
    properties
        m_T, m_S, m_waypoints, m_ncoeff, m_subscripts, m_velocity;
        m_lhs_v, m_rhs_v, eqn_index;
        m_all_coeffs;
        m_all_paths;
        t;
    end

%     methods
%         function obj = Coeff()
%         end
%     end
    methods
        function obj = Coefficients(waypoints, ncoeff, subscripts, velocity_fn)
            first_wp = waypoints(:,1);                            % First column
            if nnz(first_wp) == length(subscripts)
                obj.m_waypoints = waypoints;                % All columns onwards
            else
                obj.m_waypoints = waypoints(:,2:end); % 2nd columns onwards                
            end
            %waypoints = obj.m_waypoints;                    % Convenience
            obj.m_ncoeff = ncoeff;
            obj.m_subscripts = subscripts;
            obj.m_velocity = 2;        % m/s
            d = waypoints(:,2:end) - waypoints(:,1:end-1);
            wp_diff_norms =  sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
            obj.m_T = wp_diff_norms./velocity_fn(waypoints);
            obj.m_S = [cumsum(obj.m_T)];
            
            obj.m_all_coeffs = sym(zeros(length(subscripts), ncoeff));
            % Rows: path index, columns: derivative, 3rd dim: coefficients.
            % To access some row of coefficients: m_all_paths(1, 1, :)
            obj.m_all_paths = sym(zeros(1,1,8));            
            
            obj.t = sym('t');
        end
        function wp = getWaypoint(obj, index)
            wp = obj.m_waypoints(:,index);
        end
        function [T, S] = getTVector(obj)
            T = obj.m_T;
            S = obj.m_S;
        end
        function coeff_m= getCoeffMatrix(obj,i)
            subscripts = obj.m_subscripts;
            ncoeff = obj.m_ncoeff;
            if size(obj.m_all_coeffs, 3) >= i && nnz(obj.m_all_coeffs(:,:,i)) ~= 0
                coeff_m = obj.m_all_coeffs(:,:,i);
                return;
            end
            coeff_m = sym(zeros(length(subscripts), ncoeff));
            %m = 7;
            assert(isvector(subscripts));
            nsubs = length(subscripts);
            
            %subscripts = ['x', 'y', 'z'];
            %subscripts(1)
            for k = 1:nsubs
                for j = 1:ncoeff
                    coeff_m(k, j) = sym(strcat('C', '_', int2str(i), '_', int2str(j-1), subscripts(k)));
                end
            end
            obj.m_all_coeffs(:,:,i) = coeff_m;
        end
        function path = getPath(obj, i, time, deriv)
      %      path2 = 0;
            ncoeff = obj.m_ncoeff;
            coefficients = obj.getCoeffMatrix(i);
%             if size(obj.m_all_paths, 1) >= i ...                                % First two array indexes are not out of bounds
%                     && size(obj.m_all_paths, 2) >= deriv + 1 ...
%                     && nnz(obj.m_all_paths(i, deriv+1,:)) > 0           % Coefficients all zero would indicate this is not initialized.
%                 path2 = coefficients * double(subs(reshape(obj.m_all_paths(i, deriv+1,:), ncoeff, 1), obj.t, time));
%                 path = path2;
%                 return;
%             end            
            T = obj.m_T;
            S = obj.m_S;
%            path_coeff_v2 = sym(zeros(ncoeff,1));
%             if deriv == 0
%                 path_coeff_v2(1) = 1;
%             else 
%                 path_coeff_v2(1) = 0;
%             end
%             for k= 2: ncoeff
%                 if i == 1                    
%                     path_coeff_v2(k) = (1/T(i))^(k-1)*diff (obj.t^(k-1), obj.t, deriv);                    
%                 else 
%                     path_coeff_v2(k) = (1/T(i))^(k-1)*diff ((obj.t - S(i-1))^(k-1), obj.t, deriv);
%                 end
%             end
            path_coeff_v = zeros(ncoeff,1);
            pkn = @(k, n) prod([k-n+1:k]);
            traj = @Coefficients.getTrajectory;
            for k = deriv:ncoeff-1
                rem = k-deriv;                
                path_coeff_v(k+1) = pkn(k, deriv)...
                    * power(traj(i, time, T, S), rem)...
                    * power(1/T(i), deriv);
            end
   %         obj.m_all_paths(i, deriv+1,:) = path_coeff_v2;
            %path2 = coefficients * double(subs(reshape(obj.m_all_paths(i, deriv+1,:), ncoeff, 1), obj.t, time));
            %path_coeff_v2 = double(subs(path_coeff_v2, obj.t, time));
            %path_coeff_v = double(path_coeff_v);
%             if ~isalmost(path_coeff_v2, path_coeff_v, .001, 1)
%                 error('Error: Index %s, deriv %s, t %s', [i, deriv, time]);
%             end
            %path = path2;
            path = coefficients*path_coeff_v;
            %assert(isequaln(path,path2))
        end
        function path = getPathAtTime(obj, time)
            if (time >= obj.m_S(end))
                path = length(obj.m_S);
                return;
            end
            path = find(obj.m_S >= time, 1);
            if isempty(path) || path == 0
                error('No path found');
            end
        end
        function initializeEquations(obj)
            sz = obj.m_ncoeff*size(obj.m_waypoints, 2)*length(obj.m_subscripts);
            obj.m_lhs_v = vpa(sym(zeros(sz, 1)), 4);
            obj.m_rhs_v =vpa(sym(zeros(sz, 1)), 4);
            obj.eqn_index = 1;
        end
        function addEquation(obj, lhs, rhs)
            len = length(lhs);
            assert(len == length(obj.m_subscripts));
            for i = 1:len
                obj.m_lhs_v(obj.eqn_index) = lhs(i);
                obj.m_rhs_v(obj.eqn_index) = rhs(i);
                obj.eqn_index = obj.eqn_index + 1;
            end
        end
        function n = getNumPaths(obj)
            n = size(obj.m_waypoints, 2);
        end
        function solution = solve(obj, precision_not_used)
            %d1 = digits(precision);
            solution = orderfields(solve(obj.m_lhs_v - obj.m_rhs_v));
            fields = fieldnames(solution);
            for index = 1:numel(fields)
               solution.(fields{index}) = double(solution.(fields{index}));
            end
            %% Move solution (a structure to matrix)
            index = 1;
            for path_index = 1: obj.getNumPaths()
                for coeff = 1: obj.m_ncoeff
                    for subscript = 1:  length(obj.m_subscripts)
                        obj.m_all_coeffs(subscript, coeff, path_index) = ...
                            solution.(fields{index});
                        index = index + 1;
                    end                    
                end
            end
            obj.m_all_coeffs = double(obj.m_all_coeffs);
            %digits(d1);
        end
    end
    methods (Static)
        function fti = getTrajectory(i, t, T, S)
            % i = path_index;
            if (i == 1)
                fti = t/T(i);
            else
                fti = (t - S(i-1))/T(i);
            end
        end       
    end %methods (static)
end % class def