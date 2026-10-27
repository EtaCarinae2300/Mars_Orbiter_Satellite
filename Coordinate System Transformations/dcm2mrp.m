function sigma = dcm2mrp(C)
% dcm2mrp - Converts a Direction Cosine Matrix (DCM) to Modified Rodrigues Parameters (MRP)
% using Sheppard's method for quaternion extraction
%
% Input:
%   C - 3x3 Direction Cosine Matrix
%
% Output:
%   sigma - 3x1 Modified Rodrigues Parameters vector

    % Validate input
    if ~isequal(size(C), [3, 3])
        error('Input must be a 3x3 matrix.');
    end

    % Sheppard’s method to extract quaternion from DCM
    q = zeros(4, 1);  % q = [q0; q1; q2; q3]

    q0_sq = 0.25 * (1 + trace(C));
    q1_sq = 0.25 * (1 + 2*C(1,1) - trace(C));
    q2_sq = 0.25 * (1 + 2*C(2,2) - trace(C));
    q3_sq = 0.25 * (1 + 2*C(3,3) - trace(C));

    [max_q_sq, idx] = max([q0_sq, q1_sq, q2_sq, q3_sq]);

    switch idx
        case 1
            q(1) = sqrt(q0_sq);
            q(2) = (C(2,3) - C(3,2)) / (4*q(1));
            q(3) = (C(3,1) - C(1,3)) / (4*q(1));
            q(4) = (C(1,2) - C(2,1)) / (4*q(1));
        case 2
            q(2) = sqrt(q1_sq);
            q(1) = (C(2,3) - C(3,2)) / (4*q(2));
            q(3) = (C(1,2) + C(2,1)) / (4*q(2));
            q(4) = (C(3,1) + C(1,3)) / (4*q(2));
        case 3
            q(3) = sqrt(q2_sq);
            q(1) = (C(3,1) - C(1,3)) / (4*q(3));
            q(2) = (C(1,2) + C(2,1)) / (4*q(3));
            q(4) = (C(2,3) + C(3,2)) / (4*q(3));
        case 4
            q(4) = sqrt(q3_sq);
            q(1) = (C(1,2) - C(2,1)) / (4*q(4));
            q(2) = (C(3,1) + C(1,3)) / (4*q(4));
            q(3) = (C(2,3) + C(3,2)) / (4*q(4));
    end

    q = q / norm(q);  % Normalize quaternion

    % Quaternion to MRP
    q0 = q(1);          % scalar part
    qv = q(2:4);        % vector part

    if abs(1 + q0) < 1e-8
        warning('Quaternion scalar close to -1; MRP may be large or undefined.');
    end

    sigma = qv / (1 + q0);
end
