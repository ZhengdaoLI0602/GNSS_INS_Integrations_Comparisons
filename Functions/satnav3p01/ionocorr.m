function ionodel = ionocorr(systime,svxyz,usrxyz)
%IONOCORR	Compute ionospheric correction from the
%           broadcast model parameters
%
%	ionodel = ionocorr(systime,svxyz,usrxyz)
%
%   INPUTS
%   systime = time at which iono delay is to be
%             computed; note this is expressed as
%             system time (i.e., GPS time of week in seconds)
%	svxyz = satellite position expressed in ECEF cartesian coordinates
%	usrxyz = user position expressed in ECEF cartesian coordinates
%   NOTE: This function assumes the iono broadcast model 
%         parameters (ALPHA and BETA vectors) have been loaded
%         into global memory
%
%   OUTPUTS
%	ionodel = ionospheric delay correction (meters)

%	Copyright (c) 2002 by GPSoft
%
global ALPHA BETA

svllh = xyz2llh(svxyz);
svenu = xyz2enu(svxyz,usrxyz);
el = atan2(svenu(3),norm(svenu(1:2)));
az = atan2(svenu(1),svenu(2));
E = el/pi;  % E is elevation angle expressed in semi-circles
A = az/pi;  % A is azimuth angle expressed in semi-circles

F = 1 + 16*(0.53 - E)^3;
psi = 0.00137/(E + 0.11) - 0.022;

phiu = svllh(1)/pi;  % user geodetic latitude expressed in semi-circles
phii = phiu + psi*cos(az);
if phii > 0.416, phii = 0.416; end
if phii < -0.416, phii = -0.416; end

lambdau = svllh(2)/pi;  % user geodetic longitude expressed in semi-circles
lambdai = lambdau + psi*sin(az)/cos(phii*pi);

phim = phii + 0.064*cos((lambdai-1.616)*pi);

t = 4.32e4*lambdai + systime;
while ( (t < 0) | (t >= 86400) ),
    if t >= 86400, t = t - 86400; end
    if t < 0, t = t + 86400; end
end

PER = BETA(1) + BETA(2)*phim + BETA(3)*phim^2 + BETA(4)*phim^3;
if PER < 72000, PER = 72000; end

x = 2*pi*(t - 50400)/PER;

AMP = ALPHA(1) + ALPHA(2)*phim + ALPHA(3)*phim^2 + ALPHA(4)*phim^3;
if AMP < 0, AMP = 0; end

if abs(x) < 1.57,
    Tiono = F*(5e-9 + AMP*(1 - x^2/2 + x^4/24));
else
    Tiono = F*5e-9;
end
ionodel = Tiono*299792458;