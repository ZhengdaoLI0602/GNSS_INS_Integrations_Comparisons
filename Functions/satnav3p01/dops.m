function dopvec = dops(svxyzmat,usrxyz)
%DOPS	Compute Dilution's-of-Precision
%
%	dopvec = dops(svxyzmat,usrxyz)
%
%   INPUTS
%	svxyzmat = matrix of satellite positions in cartesian coordinates
%	svxyzmat(i,1:3) = x,y,z coordinates for satellite i
%	usrxyz = user position in cartesian coordinates
%                (consistent with svxyzmat)
%
%   OUTPUTS
%	dopvec(1) = X-axis DOP
%	dopvec(2) = Y-axis DOP
%	dopvec(3) = Z-axis DOP (VDOP if ENU relative to user)
%	dopvec(4) = X & Y DOP (HDOP if ENU relative to user)
%	dopvec(5) = TDOP
%	dopvec(6) = PDOP
%	dopvec(7) = GDOP
%
%       Note: dopvec consists of -1's if less than four satellite
%	      positions are provided

%	Reference:  Understanding GPS: Principles and Applications,
%	            Elliott D. Kaplan, Editor, Artech House Publishers,
%	            Boston, 1996.
%
%	M. & S. Braasch 10-99
%	Copyright (c) 1996-99 by GPSoft LLC
%	All Rights Reserved.
%
	N = max(size(svxyzmat));
	if N < 4,
	   dopvec = -ones(1,7);
	else,
	H = hmat(svxyzmat,usrxyz);
	dm = inv(H'*H);
	dopvec(1) = sqrt(dm(1,1));
	dopvec(2) = sqrt(dm(2,2));
	dopvec(3) = sqrt(dm(3,3));	
	dopvec(4) = sqrt(trace(dm(1:2,1:2)));
	dopvec(5) = sqrt(dm(4,4));
	dopvec(6) = sqrt(trace(dm(1:3,1:3)));
	dopvec(7) = sqrt(trace(dm));
	end,

       	