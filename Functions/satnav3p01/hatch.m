function [prsm,prmat,adrmat]=hatch(pr,adr,svid,smint,prmat,adrmat)
%HATCH		Perform carrier-smoothing of the pseudorange measurements
%               via the Hatch filter
%
%	[prsm,prmat,adrmat]=HATCH(pr,adr,svid,smint,prmat,adrmat)
%
%   INPUTS
%       pr = vector of measured pseudoranges for the current epoch
%	adr = vector of measured integrated Doppler for the current epoch
%	svid = vector of satellite id's corresponding to the measurements
%              in PR and ADR
%       smint = smoothing interval in seconds (must be an integer)
%       prmat = matrix of previous pseudorange values for all satellites.
%               When HATCH is first called, PRMAT and ADRMAT are set to [].
%               These matrices are updated by HATCH and output to the user
%               who must input them at the next epoch
%       adrmat = matrix of previous integrated Doppler values for
%                all satellites
%
%   OUTPUTS
%	prsm = vector of carrier-smoothed pseudoranges
%       prmat = updated matrix of previous pseudoranges
%       adrmat = updated matrix of previous integrated Doppler

%   Reference:  "The Synergism of GPS Code and Carrier Measurements,"
%               by Ron Hatch, Proceedings of the Third International
%               Geodetic Symposium on Satellite Doppler Positioning,
%               Las Cruces, NM, February 1982.
%
%	M. & S. Braasch 12-96
%	Copyright (c) 1996 by GPSoft
%	All Rights Reserved.
%

if nargin<6,error('insufficient number of input arguments'),end
[m,n]=size(pr); 
if m<n,trans=0;else,trans=1;end
if isempty(prmat),
   prsm = pr;
   prmat = -99999*ones(smint,100); adrmat = -99999*ones(smint,100);
   for i=1:max(size(svid)),
       prmat(1,svid(i))=pr(i); adrmat(1,svid(i))=adr(i);
   end
else,
   prmat(2:smint,:)=prmat(1:smint-1,:);
   adrmat(2:smint,:)=adrmat(1:smint-1,:);
   prmat(1,:)=-99999*ones(1,100); adrmat(1,:)=-99999*ones(1,100);
   for i=1:max(size(svid)),
       prmat(1,svid(i))=pr(i); adrmat(1,svid(i))=adr(i);
   end
   for i=1:max(size(svid)),
       rngbias=mean(prmat(find(prmat(:,svid(i))~=-99999),svid(i))-...
                   adrmat(find(prmat(:,svid(i))~=-99999),svid(i)));
       prsm(i)=rngbias+adr(i);
   end
   if trans,prsm=prsm';end
end
