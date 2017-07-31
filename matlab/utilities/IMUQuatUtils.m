classdef IMUQuatUtils
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Static)
        % V is a Mx3 matrix. Q is all quaternions that map the vecotors in
        % V with a vector [0 0 norm(vector)]. For acceleration in
        % stationary state, this may be a [0 0 g].
        % quatrotate(Q, V) will yield a series of vectors of the form [0 0 c]
        function [Q] = GAlignment(V)
            assert(size(V, 2) == 3);
            M = size(V, 1);
            Q = zeros(M, 3);            
            for i=1:M
                v1 = V(i,:);
                v2 = [0, 0, norm(v1)];
                Q(i,1) = norm(v1)*norm(v2) + dot(v1,v2);
                Q(i, 2:4) = cross(v2, v1);
                Q(i,:) = Q(i,:)/norm(Q(i,:));
            end            
        end
    end
    
end

