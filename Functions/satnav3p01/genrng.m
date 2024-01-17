function [prvec,adrvec]=genrng(rxid,usrxyz,svmat,svid,time,esf,...
   samat,mpmat,freqc,TECmax,TECmin)
%GENRNG Generate vector of 'measured' pseudoranges and accumulated
%        delta ranges (i.e., integrated Doppler) to all
%        visible satellites.
%
%   [prvec,adrvec] = genrng(rxid,usrxyz,svmat,svidvec,time,esf,...
%                    samat,mpmat,freqc,TECmax,TECmin,randstate)
%
%   INPUTS
%       rxid = receiver identification number (must be a positive integer)
%              For single receiver (stand-alone) simulations, RXID can be 
%              set to 1.
%     usrxyz(1:3) = true user position in ECEF cartesian coordinates.
%     svmat(i,1:3) = position of satellite i in ECEF 
%                    cartesian coordinates.
%       svid = vector of identification numbers for the visible satellites
%              corresponding to svmat
%       time = time of week in seconds
%             esf = error flags and scaling.  If set to zero, error-free 
%                pseudoranges are computed.  If ESF is any other scalar,
%                all error sources are added (with default scaling) to the 
%                true ranges to produce
%                'measured' pseudoranges.  If ESF is a vector,
%                specific error sources can be turned on or off:
%                  esf(1):  Thermal Noise scaling factor
%                  esf(2):  Tropospheric error scaling factor
%                  esf(3):  SA scaling factor
%                  esf(4):  Multipath error scaling factor
%                  esf(5):  Ionospheric error scaling factor
%                Example: esf=[1 0 0 0 0] would cause only
%                           thermal noise to be added
%       samat = Optional matrix of Selective Availability error.
%               SAMAT(t,i) is the SA error for satellite i at t seconds
%               after the GPS hour.  
%       mpmat = Optional matrix of multipath error.  MPMAT(t,i) is the
%               zero-elevation angle equivalent pseudorange multipath 
%               error for 
%               satellite i at t seconds after the GPS hour.  The 
%               zero-angle error is scaled by the cosine of the true
%               satellite elevation angle before it is applied to the
%               range measurement.  The carrier-phase multipath error
%               is given by multiplying the pseudorange multipath
%               error by (lambda*0.005).  Since lambda (the carrier
%               wavelength in meters) is approximately 0.2 (depending
%               upon the carrier frequency), the carrier-phase
%               multipath error is approximately a factor of 1000
%               less than the pseudorange multipath error.  This is
%               typical with the pseudorange multipath error being
%               on the meter level and the carrier-phase multipath
%               error being on the millimeter-level.
%               For DGPS applications, a different MPMAT
%               should be used for each receiver.  Note also that
%               a different MPMAT should be used for each simulated
%               code (i.e., C/A and P) and each carrier frequency 
%               (i.e., the multipath on
%               L1 is not the same as the multipath on L2)
%       freqc = GPS, Galileo or GEO carrier frequency in MHz; default
%               is 1575.42; if Glonass is being simulated, user should
%               input the corresponding GPS frequency.  For example,
%               input 1227.6 if you are trying to simulate Glonass L2.
%               Note that Glonass satellites are identified by satellite id
%               numbers which are 50 or above.  Although any carrier
%               frequency for GPS can be modeled; simulation of
%               Glonass is limited to its L1 or L2 bands.
%      TECmax = optional argument equal to maximum daytime value
%               of total election content (default is 1.6*1e18)
%      TECmin = optional argument equal to minimum daytime value
%               of total election content (default is 4*1e17)
%
%   OUTPUTS
%            prvec = vector of 'measured' pseudoranges for satellites
%               specified in svmat
%      adrvec = vector of 'measured' accumulated delta ranges for
%               satellites specified in svmat
%
%    NOTE: If the input parameters FREQC, TECmax and TECmin are to
%          be used, the matrices SAMAT and MPMAT must be input as
%          well.  These matrices may be set equal to the empty set,
%          [], if the user does not want these errors to be included.
%          Similarly, if the input parameter RANDSTATE is to be used
%          and it is desired to let FREQC, TECmax and TECmin have their
%          default values, then these parameters should be set equal
%          to the empty set, [].
%
%          The scaling factors normally have a valid range between 
%          0 and 1 and the
%          default value is 1.  For thermal noise, the default one sigma
%          ranging error is 1 meter for pseudorange and (0.05*lambda) meters
%          for the integrated Doppler (where lambda is the carrier 
%          wavelength in meters).  For tropospheric delay, the default
%          is determined by the function TROPGEN and results in (roughly) a
%          delay of 3 meters for a satellite at zenith and increases to 
%          a value of 25 meters for a satellite at 5 degrees elevation.
%          For selective availability, the
%          ranging error level is set by the function SAGEN which is
%          used to create SAMAT.
%          For multipath, the ranging error level is set by the function
%          MPGEN which is used to create MPMAT.  Ionospheric delay is set 
%          by the function IONOGEN.
%
%          The following should be noted regarding simulation of P-code,
%          dual-frequency, third civil frequency, et cetera.  The primary
%          differences between C/A-code pseudoranges measured on L1 and 
%          C/A-code pseudoranges on, say, the third civil frequency are
%          ionospheric delay and multipath error.  In addition, the noise
%          and multipath on the respective carrier-phase measurements 
%          will be different.
%
%          The function IONOGEN handles the differences in ionospheric
%          delay as a function of carrier frequency.  Note also that the
%          ionospheric delay on the C/A-code on L1 is the same as the
%          ionospheric delay on the P-code on L1.  However, the P-code
%          (on L1) noise is significantly less than the noise on the
%          C/A-code (on L1).  In order to simulate both C/A and P-code
%          measurements, the user must call GENRNG once for each code.
%          When simulating C/A-code, the noise scaling flag, esf(1),
%          might be set equal to 1.  When calling GENRNG again to
%          simulate simultaneous P-code measurements, esf(1) should
%          then be set equal to 0.32 to account for the 1/sqrt(10)
%          noise reduction in P-code over the C/A-code.  If simulating
%          both P-code on L1 and P-code on L2, GENRNG must be called
%          separately for each to ensure that the noise is independent
%          for each one and to account for the different carrier
%          frequencies.  The situation is the same for multiple
%          civilian frequencies.
%
%          Regarding multipath, consider the typical case where the
%          reflecting obstacles are relatively close to the receiver.
%          The pseudorange multipath error level is independent of
%          whether P-code or C/A-code is used.  In addition, the
%          multipath level for the P-code on L1 is the same as the
%          multipath level for the P-code on L2.  At any instant
%          in time, however, the two codes will not have equal
%          multipath error.  Thus, MPGEN must be used separately
%          to generate the multipath error for each code.
%
%          As is implied by the input parameter set, multipath and selective
%          availability are created 'in batch.'  This is done to speed runtime
%          and, in the case of selective availability, is required in order to
%          achieve the proper correlation properties (vital for DGPS applications).
%          GENRNG only recognizes the first hour (3600 seconds) of MPMAT and SAMAT.
%          Simulations which surpass an hour will result in error discontinuties
%          resulting from the recycling of MPMAT and SAMAT.  If the user desires
%          to run longer simulations, this can be accomplished by setting multipath
%          and SA to zero in GENRNG and then adding it to the measurements
%          externally (see MPGEN and SAGEN).
%
%          The core GPS satellites (i.e., PRN's 1 - 32) are the only
%          satellites capable of Selective Availability.  However, the SA
%          level was set to zero on 01 May 2000.  This toolbox allows one
%          to simulate the presence of SA, however, if one so desires.
%          In order to accommodate simultaneous operation of GPS with Glonass,
%          GEO's and or Galileo, GENRNG
%          only adds SA if the satellite identification number is less than 33.
%          Note that LOADGPS results in satellite orbital parameters loaded
%          for GPS (SV ID's 1 - 24); LOADGALILEO loads SV ID's 201 - 230 for
%          the Galileo satellites and LOADGEO loads SV ID's in the
%          range of 120 - 138.  LOADGLO loads Glonass (range of 51 - 74).


%       M. & S. Braasch Revised: 07-2003
%       Copyright (c) 1996-2003 by GPSoft LLC
%       All Rights Reserved.
%

if nargin<11, TECmin=4e17; end
if nargin<10, TECmax=1.6e18; end
if nargin<9, freqc=1575.42; end
if nargin<8,
   mpflg=0;
elseif isempty(mpmat),
   mpflg=0;
else,
   mpflg=1;
end
if nargin<7,
   saflg=0;
elseif isempty(samat),
   saflg=0;
else,
   saflg=1;
end
if nargin<6,error('insufficient number of input arguments'),end
[m,n] = size(usrxyz);
if m>n, usrpos=usrxyz';else,usrpos=usrxyz;end
if max(size(esf))~=5,
   if esf==0,esf=[0 0 0 0 0];else,esf=[1 1 1 1 1];end
end
if esf(3)==0, saflg=0; end
if esf(4)==0, mpflg=0; end
numvis = max(size(svmat));
SAerr=0;prnois=0;adrnois=0;troperr=0;mperrpr=0;mperradr=0;ionoerr=0;
for N = 1:numvis,
   pr = norm(svmat(N,:)-usrpos);
   if ((mpflg~=0)|(saflg~=0)),
    tgpssec=1+time-3600*(fix(time/3600));t1=fix(tgpssec);
    K=svid(N);
    if ( (saflg~=0) & (K<33) ),
     SAerr=esf(3)*(samat(t1,K)+(tgpssec-t1)*(samat(t1+1,K)-samat(t1,K)));
    end,
    if mpflg~=0,
     svenu=xyz2enu(svmat(N,:),usrxyz);
     beta=atan2(svenu(3),norm(svenu(1:2)));
     sf=cos(beta);
     mperrpr=...
        sf*esf(4)*(mpmat(t1,K)+(tgpssec-t1)*(mpmat(t1+1,K)-mpmat(t1,K)));
    end,
   end,
   if esf(1)~=0,prnois=esf(1)*randn;end
   if esf(2)~=0,troperr=esf(2)*tropgen(usrxyz,svmat(N,:));end
   if esf(5)~=0,ionoerr=...
         esf(5)*ionogen(usrxyz,svmat(N,:),time,freqc,TECmax,TECmin);end
   prvec(N) = pr+SAerr+prnois+troperr+mperrpr+ionoerr;
   if svid(N) < 50,
      lambda = 299792458/(freqc*1e6);
   elseif svid(N) < 100,
      if freqc==1575.42,
         Z=9;
      else
         Z=7;
      end
      lambda = 299792458/((178+(svid(N)-50)/16)*Z*1e6);
   else
      lambda = 299792458/(freqc*1e6);
   end
   if mpflg~=0, mperradr=mperrpr*0.005*lambda;end
   if esf(1)~=0,adrnois=0.05*lambda*esf(1)*randn;end
   adrvec(N) = pr-10*rxid*rxid*svid(N)*lambda+SAerr+adrnois+...
               troperr+mperradr-ionoerr;
end,
