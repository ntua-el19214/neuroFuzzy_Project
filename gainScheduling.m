function gainMat = gainScheduling(currentSA, ssVectorSteerA, ExpandedMatrices)
%Define an algorithmic method to switch between gains, when operating
%points change

gainMat = struct('Kr', zeros([size(ExpandedMatrices(1).Kr)]), 'Ki', zeros([size(ExpandedMatrices(1).Ki)]), 'Kp', zeros([size(ExpandedMatrices(1).Kp)]));
% Loop to calculate degreeOfMembership
if isscalar(ssVectorSteerA)
    gainMat.Kr = ExpandedMatrices.Kr;
    gainMat.Ki = ExpandedMatrices.Ki;
    gainMat.Kp = ExpandedMatrices.Kp;
else
    % Multi-point gain scheduling: blends gains from several linearization operating
    % points via triangular membership functions over the steering angle. Currently
    % unreachable — helpSetupTheProblem.m only ever computes a single design point
    % (i = 3:3), so ssVectorSteerA is always scalar and the branch above runs instead.
    % Kept in place (not deleted) in case the solver loop is restored to sweep all
    % 5 operating points and true gain scheduling is re-enabled.
    for iOP = 1:length(ssVectorSteerA)
        if iOP == 1
            % Special case for the first element
            degreeOfMembership = triangularPulse(2*ssVectorSteerA(1), ssVectorSteerA(1), ssVectorSteerA(2), currentSA);
            gainMat.Kr = gainMat.Kr + degreeOfMembership.*ExpandedMatrices(iOP).Kr;
            gainMat.Ki = gainMat.Ki + degreeOfMembership.*ExpandedMatrices(iOP).Ki;
            gainMat.Kp = gainMat.Kp + degreeOfMembership.*ExpandedMatrices(iOP).Kp;
        elseif iOP == 5
            % Special case for the last element
            degreeOfMembership = triangularPulse(2*ssVectorSteerA(4), ssVectorSteerA(5), 2*ssVectorSteerA(5), currentSA);
            gainMat.Kr = gainMat.Kr + degreeOfMembership.*ExpandedMatrices(iOP).Kr;
            gainMat.Ki = gainMat.Ki + degreeOfMembership.*ExpandedMatrices(iOP).Ki;
            gainMat.Kp = gainMat.Kp + degreeOfMembership.*ExpandedMatrices(iOP).Kp;
        else
            % General case for elements 2 to 4
            degreeOfMembership = triangularPulse(ssVectorSteerA(iOP-1), ssVectorSteerA(iOP), ssVectorSteerA(iOP+1), currentSA);
            gainMat.Kr = gainMat.Kr + degreeOfMembership.*ExpandedMatrices(iOP).Kr;
            gainMat.Ki = gainMat.Ki + degreeOfMembership.*ExpandedMatrices(iOP).Ki;
            gainMat.Kp = gainMat.Kp + degreeOfMembership.*ExpandedMatrices(iOP).Kp;
        end
    end
end

end