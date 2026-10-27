function [MRP_BR, w_BR] = Calc_AttitudeError(BN, w_BN, RN, w_RN)
% Function enables attitude feedback control and computes the tracking errors in both attitude and angular velocity 
% Inputs: Spacecraft Attitude (BN) & Angular Rate (w_BN) 
%     AND Reference Attitude (RN) & Angular Rate(w_RN)
% Outputs: Attitude Error MRP (MRP_BR) & Angular Rate error (w_BR)

BR = BN*(RN');
q_BR = dcm2quat(BR);
if q_BR(1) < 0
    q_BR = -dcm2quat(BR);
end

MRP_BR = [q_BR(2)/(1+q_BR(1)); q_BR(3)/(1+q_BR(1)); q_BR(4)/(1+q_BR(1))];
w_BR = w_BN - [BN]*w_RN;

end
