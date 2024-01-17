function [lag,r] = gpscor(x1,x2,numlag)
%GPSCOR      Limited circular cross-correlation function for C/A-codes
%            (1023 length sequences)
%
%	[LAG,R] = GPSCOR(X1,X2,NUMLAG)
%
%   INPUTS
%	x1, x2 = sequences to be correlated (i.e., the same or two
%                different C/A-codes or G1 or G2 codes)
%
%   OUTPUTS
%       R = cross-correlation values
%       LAG = vector of lags corresponding to the values in R
%       NUMLAG = total number of lags to be processed.  
%                LAG ranges from 0 to NUMLAG.

%	M. & S. Braasch 10-96
%	Copyright (c) 1996 by GPSoft
%	All Rights Reserved.
%

if nargin<3,error('insufficient number of input arguments'),end
if round(numlag)~=numlag,error('NUMLAG must be an integer'),end

for tau = 0:numlag,
    lag(tau+1) = tau;
    r(tau+1) = 0;
    for i = 1:1023,
        k = i+tau;
        if k > 1023,
           k = k - 1023;
        end,
        r(tau+1) = r(tau+1) + x1(i)*x2(k);
    end,
end,
