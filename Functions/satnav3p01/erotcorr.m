function svxyzr = erotcorr(svxyz,pr)
%EROTCORR	Apply earth rotation correction
%           to the satellite position
%
%	svxyzr = erotcorr(svxyz,pr)
%
%   INPUTS
%	svxyz = satellite position at time-of-transmission
%           expressed in the ECEF cartesian coordinate
%           frame at the time of transmission
%	pr = measured pseudorange for the given satellite
%        corrected for satellite clock offset (meters)
%
%   OUTPUTS
%	svxyzr = satellite position at time-of-transmission
%            expressed in the ECEF cartesian coordinate
%            frame at time-of-reception (returned as
%            a column vector)

%	Copyright (c) 2002 by GPSoft
%
[r,c] = size(svxyz);
if r == 1,
    satxyz = svxyz';
else
    satxyz = svxyz;
end
%
omega = 7.2921151467e-5;   % WGS-84 value of the Earth's
%                          % rotation rate (rad/s)
deltat = pr/299792458;
theta = omega*deltat;
rotmat = [cos(theta) sin(theta) 0;   % rotation matrix
         -sin(theta) cos(theta) 0;
             0          0       1];
         
svxyzr = rotmat*satxyz;
