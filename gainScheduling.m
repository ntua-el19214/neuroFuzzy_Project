function gainMat = gainScheduling(curretSA, ssVectorSA, ExpandedMatrices)
%Define an algorithmic method to switch between gains, when operating
%points change

gainMat = struct('Kr', zeros([size(ExpandedMatrices(1).Kr)]), 'Ki', zeros([size(ExpandedMatrices(1).Ki)]), 'Kp', zeros([size(ExpandedMatrices(1).Kp)]));
% Loop to calculate degreeOfMembership
for iOP = 1:5
    if iOP == 1
        % Special case for the first element
        degreeOfMembership = triangularPulse(2*ssVectorSA(1), ssVectorSA(1), ssVectorSA(2), curretSA);
        gainMat.Kr = gainMat.Kr + degreeOfMembership.*ExpandedMatrices(iOP).Kr;
        gainMat.Ki = gainMat.Ki + degreeOfMembership.*ExpandedMatrices(iOP).Ki;
        gainMat.Kp = gainMat.Kp + degreeOfMembership.*ExpandedMatrices(iOP).Kp;
    elseif iOP == 5
        % Special case for the last element
        degreeOfMembership = triangularPulse(2*ssVectorSA(4), ssVectorSA(5), 2*ssVectorSA(5), curretSA);
        gainMat.Kr = gainMat.Kr + degreeOfMembership.*ExpandedMatrices(iOP).Kr;
        gainMat.Ki = gainMat.Ki + degreeOfMembership.*ExpandedMatrices(iOP).Ki;
        gainMat.Kp = gainMat.Kp + degreeOfMembership.*ExpandedMatrices(iOP).Kp;
    else
        % General case for elements 2 to 4
        degreeOfMembership = triangularPulse(ssVectorSA(iOP-1), ssVectorSA(iOP), ssVectorSA(iOP+1), curretSA);
        gainMat.Kr = gainMat.Kr + degreeOfMembership.*ExpandedMatrices(iOP).Kr;
        gainMat.Ki = gainMat.Ki + degreeOfMembership.*ExpandedMatrices(iOP).Ki;
        gainMat.Kp = gainMat.Kp + degreeOfMembership.*ExpandedMatrices(iOP).Kp;
    end
end


end