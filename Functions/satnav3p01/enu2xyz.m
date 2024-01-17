function xyz = enu2xyz(enu,orgxyz)
%ENU2XYZ	Convert from rectangular local-level-tangent 
%               ('East'-'North'-Up) coordinates to WGS-84 
%               ECEF cartesian coordinates.
%
%	xyz = ENU2XYZ(enu,orgxyz)	
%
%	enu(1) = 'East'-coordinate relative to local origin (meters)
%	enu(2) = 'North'-coordinate relative to local origin (meters)
%	enu(3) = Up-coordinate relative to local origin (meters)
%
%	orgxyz(1) = ECEF x-coordinate of local origin in meters
%	orgxyz(2) = ECEF y-coordinate of local origin in meters
%	orgxyz(3) = ECEF z-coordinate of local origin in meters
%
%	xyz(1,1) = ECEF x-coordinate in meters
%	xyz(2,1) = ECEF y-coordinate in meters
%	xyz(3,1) = ECEF z-coordinate in meters

%	Reference: Alfred Leick, GPS Satellite Surveying, 2nd ed.,
%	           Wiley-Interscience, John Wiley & Sons, 
%	           New York, 1995.
%
%	M. & S. Braasch 10-96
%	Copyright (c) 1996 by GPSoft
%	All Rights Reserved.

if nargin<2,error('insufficient number of input arguments'),end
[m,n]=size(enu);if m<n,tmpenu=enu';else,tmpenu=enu;end
[m,n]=size(orgxyz);if m<n,tmpxyz=orgxyz';else,tmpxyz=orgxyz;end
orgllh = xyz2llh(tmpxyz);
phi = orgllh(1);
lam = orgllh(2);
sinphi = sin(phi);
cosphi = cos(phi);
sinlam = sin(lam);
coslam = cos(lam);
R = [ -sinlam          coslam         0     ; ...
      -sinphi*coslam  -sinphi*sinlam  cosphi; ...
       cosphi*coslam   cosphi*sinlam  sinphi];
difxyz = inv(R)*tmpenu;
xyz = tmpxyz + difxyz;
