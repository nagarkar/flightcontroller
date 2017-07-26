classdef SnapTrajectoryGenerator < handle
    %SNAPTRAJECTORYGENERATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Static)
        %%%%
        % Inputs:
        %   waypoints: kxN matrix of waypoints. k = length(subscripts)
        %   subscripts: The subscripts (e.g. x, y, z, theta) associated with 
        %      degrees of freedom.
        %   m               : Order of the derivative you want to optimize
        %       in the functional. E.g. for snap trajectory, you want to
        %       minimize fourth derivative, so m = 4.
        %   velocity_fn: A function that takes waypoints as iput, and
        %       produces velocity expected to travel between waypoints
        function [instance, solution] = GenerateTrajectorySolution(waypoints, subscripts, m, velocity_fn)
            ncoeff = m*2;                  % Order 7 polynomial
            system_order = ncoeff - 1;

            coeff_inst = Coefficients(waypoints, ncoeff, subscripts, velocity_fn);
            n = size(coeff_inst.m_waypoints, 2);
            % Get T and S vectors
            [~, S] = coeff_inst.getTVector();
             % Precompute coefficient matrices (not necessary)
             for i = 1:2
                coeff_inst.getCoeffMatrix(i);
             end

            %  coeff_m_expected = [C_2_0x, C_2_1x, C_2_2x, C_2_3x, C_2_4x, C_2_5x, C_2_6x, C_2_7x;
            %     C_2_0y, C_2_1y, C_2_2y, C_2_3y, C_2_4y, C_2_5y, C_2_6y, C_2_7y;
            %     C_2_0z, C_2_1z, C_2_2z, C_2_3z, C_2_4z, C_2_5z, C_2_6z, C_2_7z;];
            %  assert(isequal(coeff_m,coeff_m_expected))

            coeff_inst.initializeEquations();
            % Add Constraints on starting position for paths (special case for first path)
            path = 1;
            t = 0;
            deriv = 0;
            coeff_inst.addEquation(coeff_inst.getPath(path, t, deriv), [0;0;0]);
            for path = 2:n
                t = S(path - 1);
                coeff_inst.addEquation(coeff_inst.getPath(path, t, deriv),...
                    coeff_inst.getWaypoint(path - 1));
            end

            % Add Constraints on ending position for paths
            deriv = 0;
            for path = 1:n
                t = S(path);
               coeff_inst.addEquation(coeff_inst.getPath(path, t, deriv),...
                    coeff_inst.getWaypoint(path));
            end

            for deriv = 1:(m-1)
                path = 1;
                t = 0;
                coeff_inst.addEquation(coeff_inst.getPath(path, t, deriv),...
                    [0; 0; 0]);
                path = n;
                t = S(path);
                coeff_inst.addEquation(coeff_inst.getPath(path, t, deriv),...
                    [0; 0; 0]);
            end

            for deriv = 1:(system_order -1)
                for path = 1:(n-1)
                    coeff_inst.addEquation(coeff_inst.getPath(path, S(path), deriv),...
                        coeff_inst.getPath(path + 1, S(path), deriv));
                end
            end
            solution = coeff_inst.solve();
            instance = coeff_inst;
        end
    end
    
end

