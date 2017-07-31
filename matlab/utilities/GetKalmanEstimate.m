% Zs are measurements M x 3
% P is the known P matrix from running EstimateKalmanFilter
% u is either zero or known value.
function [X] = GetKalmanEstimate(kf_sys, P, Z, U)
    M = size(Z, 1);
    state = [0,0,0, 0,0,0, Z(1,:)]';
    X = zeros(M, 9);
    X(1,:) = state;
    sign = 1;
    for i=2:M
        sign = -1 * sign;
        [state, P]  = KF(kf_sys, Z(i,:)', U(i,:)', state, P);
        X(i,:) = state;
    end
    DEAnalysis.plotState(X);
end

