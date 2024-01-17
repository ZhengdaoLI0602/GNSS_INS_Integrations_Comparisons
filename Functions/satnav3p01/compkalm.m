function [prsm,Pvec,prevadr]=compkalm(pr,adr,svid,P,Q,R,Pvec,prevsm,prevadr)
%COMPKALM	Perform carrier-smoothing of the pseudorange measurements
%               via the complementary Kalman filter
%
%	[prsm,Pvec,prevadr]=COMPKALM(pr,adr,svid,P,Q,R,Pvec,prevsm,prevadr)
%
%   INPUTS
%       pr = vector of measured pseudoranges for the current epoch
%	adr = vector of measured integrated Doppler for the current epoch
%	svid = vector of satellite id's corresponding to the measurements
%              in PR and ADR
%       P = initial value for estimation error variance
%       Q = Process noise variance (meters-squared)
%       R = measured Pseudorange thermal noise variance (meters-squared)
%       Pvec = vector of prediction error variances for each measurement.
%              When COMPKALM is first called, PVEC is set to [].
%       prevsm = vector of previous smoothed pseudoranges for all satellites.
%                When COMPKALM is first called, PREVSM is set to [].
%       prevadr = vector of previous integrated Doppler measurements.
%                 When COMPKALM is first called, PREVADR is set to [].
%
%   OUTPUTS
%	prsm = vector of carrier-smoothed pseudoranges
%       Pvec = updated vector of prediction error variances
%       prevadr = updated vector of previous integrated Doppler measurements.

%
%	M. & S. Braasch 12-96
%	Copyright (c) 1996 by GPSoft
%	All Rights Reserved.
%

if nargin<9,error('insufficient number of input arguments'),end
[m,n]=size(pr); 
if m<n,trans=0;else,trans=1;end
if isempty(prevsm),
   prsm = pr;
   Pvec = -99999*ones(1,100);
   for i=1:max(size(svid)),
       Pvec(1,svid(i))=P;
       prevadr(1,svid(i))=adr(i);
   end
else,
   for i=1:max(size(svid)),
       if Pvec(1,svid(i)) > 0,
          predpr = prevsm(i) + adr(i) - prevadr(1,svid(i));
          Pvec(1,svid(i)) = Pvec(1,svid(i)) + Q;
          K = Pvec(1,svid(i)) / ( Pvec(1,svid(i)) + R );
          prsm(i) = predpr + K * ( pr(i) - predpr );
          Pvec(1,svid(i)) = (1 - K) * Pvec(1,svid(i));
       else,
          Pvec(1,svid(i)) = P;
          prsm(i) = pr(i);
       end,
       prevadr(1,svid(i)) = adr(i);
   end
   if trans,prsm=prsm';end
end
