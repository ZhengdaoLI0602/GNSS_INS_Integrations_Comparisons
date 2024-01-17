function delta=ionogen(usrxyz,svxyz,time,freqc,TECmax,TECmin)
%IONOGEN        Generate ionospheric delay in meters
%
%       delta = ionogen(usrxyz,svxyz,time,freqc,TECmax,TECmin)
%
%   INPUTS
%       usrxyz(1:3) = true user position in ECEF cartesian coordinates.
%       svxyz(1:3) = position of satellite in ECEF cartesian coordinates.
%  time = GPS time of week in seconds.
%  freqc = optional argument equal to carrier frequency in MHz 
%          (default is 1575.42)
%  TECmax = optional argument equal to maximum daytime value
%           of total election content (default is 1.6*1e18)
%  TECmin = optional argument equal to minimum daytime value
%           of total election content (default is 4*1e17)
%
%   OUTPUTS
%       delta = ionospheric delay in meters corresponding to user and
%               satellite positions specified by USRXYZ and SVXYZ and 
%               GPS time given by TIME

%       References: 
%                   Global Positioning System - Theory and Applications, 
%                   Vol. I, Parkinson, B. and J. Spilker, Jr., editors,
%                   American Institute of Aeronautics and Astronautics,
%                   Washington, D.C., 1996.
%
%                   Understanding GPS: Principles and Applications,
%                   Elliott D. Kaplan, Editor, Artech House Publishers,
%                   Boston, 1996.
%
%       M. & S. Braasch 7-99
%       Copyright (c) 1999 by GPSoft LLC
%       All Rights Reserved.
%

if nargin<6, TECmin=4e17; end
if nargin<5, TECmax=1.6e18; end
if nargin<4, freqc=1575.42; end
if nargin<3,error('insufficient number of input arguments'),end

svenu=xyz2enu(svxyz,usrxyz);
E=atan2(svenu(3),norm(svenu(1:2)));
A=atan2(svenu(1),svenu(2));
usrllh=xyz2llh(usrxyz);
f=freqc*1e6;
c1=40.3*TECmin/f^2;
c2=40.3*TECmax/f^2;

phi=0.0137/(E/pi+0.11)-0.022;
phii=usrllh(1)/pi + phi*cos(A);
if phii > 0.416, phii=0.416; end
if phii < -0.416, phii=-0.416; end
lambdai=usrllh(2)/pi + (phi*sin(A))/cos(phii*pi);
numday=fix(time/86400);
todsec=time-numday*86400;
localt=4.32e4*lambdai + todsec;
if localt > 86400, localt = localt - 86400; end
if localt < 0, localt = localt + 86400; end

phase=((localt/3600-14)*2*pi)/28;
if abs(phase) < pi/2,
   Tg = c1 + c2*cos(phase);
else,
   Tg = c1;
end,

Re=mean([6378137 6356752]);
hi=400000;
F=1/sqrt( 1 - ( (Re*cos(E))/(Re+hi) )^2 );
delta=F*Tg;