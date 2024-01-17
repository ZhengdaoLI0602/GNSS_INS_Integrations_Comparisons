function [ca,g1,g2]=cacode(g2shift)
%CACODE		Generate C/A-code sequence
%
%	[CA,G1,G2] = CACODE(G2SHIFT)
%
% INPUTS
%     g2shift = circular shift of G2 maximal length code relative to the
%            G1 maximal length code (must be an integer in the range 0:1023)
%
% OUTPUTS
%   ca = C/A-code sequence realized with +1's and -1's (digital 0's and 1's).
%        CA is the product of G1 and G2 (after G2 has been shifted)
%     g1, g2 = maximal length sequences with g2 shifted according to g2shift

%	References: 
%                   ICD-GPS-200, NAVSTAR GPS Space Segment/Navigation User
%                   Interfaces (Public Release Version), ARINC Research
%                   Corporation, 11770 Warner Ave., Suite 210, Foutain
%                   Valley, CA 92708, July 3, 1991.
%
%                   Understanding GPS: Principles and Applications,
%	            Elliott D. Kaplan, Editor, Artech House Publishers,
%	            Boston, 1996.
%
%	M. & S. Braasch 12-96;  Revised 10-99
%	Copyright (c) 1996-99 by GPSoft LLC
%	All Rights Reserved.
%

if nargin<1,error('insufficient number of input arguments'),end
if ((g2shift<0)|(g2shift>1023)),error('invalid g2shift.  Must be in the range 0:1023'),end
if round(g2shift)~=g2shift,error('invalide g2shift.  Must be an integer'),end

R1 = -1*ones(1,10);
R2 = R1;

for k=1:1023,
   G1(k)=R1(10);
   temp1 =R1(3)*R1(10);
   R1(2:10)=R1(1:9);
   R1(1)=temp1;
   
   G2(k)=R2(10);
   temp2 =R2(2)*R2(3)*R2(6)*R2(8)*R2(9)*R2(10);
   R2(2:10)=R2(1:9);
   R2(1)=temp2;   
end,

shift=g2shift;
shiftG2(shift+1:1023)=G2(1:1023-shift);
shiftG2(1:shift)=G2(1023-shift+1:1023);

ca=G1.*shiftG2;
g1 = G1;
g2 = shiftG2;
