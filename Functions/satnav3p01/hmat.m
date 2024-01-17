function h = hmat(svmat,usrpos)
%HMAT	Compute Direction Cosine Matrix
%
%	h = hmat(svmat,usrpos)
%
%   INPUTS
%	svmat = matrix of satellite positions in
%	        user defined cartesian coordinates
%		   svmat(i,1:3) = x,y,z coordinates for satellite i
%	usrpos = estimated user position in user defined 
%	         cartesian coordinates
%
%   OUTPUTS
%	h = direction cosine matrix for GPS positioning

%	Reference:
%                   Understanding GPS: Principles and Applications,
%	            Elliott D. Kaplan, Editor, Artech House Publishers,
%	            Boston, 1996.
%
%	Copyright (c) 1996 by GPSoft
%
	N = max(size(svmat));
	if N < 4,
	   error('insufficient number of satellites')
	else,
	   tmppos = usrpos;
	   [m,n] = size(tmppos);
	   if m > n, tmppos = tmppos'; end,
	   h = ones(N,4);
	   for i = 1:N,
	       tmpvec = tmppos - svmat(i,:);
	       h(i,1:3) = tmpvec./norm(tmpvec);
	   end,
	end,

   	