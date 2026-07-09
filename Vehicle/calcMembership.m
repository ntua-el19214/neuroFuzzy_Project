function membership = calcMembership(input, centers, width)
%CALCMEMBERSHIP Gaussian fuzzification with normalization.
% Returns the (normalized) degree of membership of INPUT in each fuzzy
% set defined by CENTERS, with common spread WIDTH.
membership = exp(-((input - centers) / width).^2);
membership = membership / sum(membership); % Normalize
end
