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
%	M. & S. Braasch 12-96
%	Copyright (c) 1996 by GPSoft
%	All Rights Reserved.
%

if nargin<1,error('insufficient number of input arguments'),end
if ((g2shift<0)|(g2shift>1023)),error('invalid g2shift.  Must be in the range 0:1023'),end
if round(g2shift)~=g2shift,error('invalide g2shift.  Must be an integer'),end

% Generate G1 code
%
%  load shift register
for i = 1:10,
    reg(i) = -1;
end,
%
%  generate code
for i = 1:1023,
    g1(i) = reg(10);
    save1 = reg(3)*reg(10);
    for j = 9:-1:1,
        reg(j+1) = reg(j);
    end,
    reg(1) = save1;
end,
%
% Generate G2 code
%
% load shift register
for i = 1:10,
    reg(i) = -1;
end,
%
% generate code
for i = 1:1023,
    g2(i) = reg(10);
    save2 = reg(2)*reg(3)*reg(6)*reg(8)*reg(9)*reg(10);
    for j = 9:-1:1,
        reg(j+1) = reg(j);
    end,
    reg(1) = save2;
end,
%
%  Shift G2 code
for i = 1:1023,
    k = i + g2shift;
    if k > 1023,
       k = k - 1023;
    end,
    g2tmp(k) = g2(i);
end,
%
g2 = g2tmp;
%
%  Form C/A code by multiplying G1 and G2
%
ca = g1.*g2;
%
