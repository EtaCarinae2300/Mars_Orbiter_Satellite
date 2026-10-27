function BN = MRP2dcm(sigma)

    s2 = dot(sigma, sigma);  % sigma squared

    S = vec2skew(sigma);         % Skew-symmetric matrix
    I = eye(3);

    BN = I + (8 * S*S - 4 * (1 - s2) * S) / (1 + s2)^2;
end
