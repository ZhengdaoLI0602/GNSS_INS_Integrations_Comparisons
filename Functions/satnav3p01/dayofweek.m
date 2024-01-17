function daywknum = dayofweek(year,monthnum,dayofmonth)
%DAYOFWEEK	Calculate the day of the week given the
%           year, month and day of the month
%
%	daynum = dayofweek(year,monthnum,dayofmonth)
%
%   INPUTS
%	year = year number; only valid for the Gregorian
%          calendar (the calendar the entire world
%          now uses);  this function is not valid for
%          the Julian calendar which was in use until
%          the 1500's (even later in some countries)
%	monthnum = month number (1=Jan, 2=Feb, etc)
%   dayofmonth = day of the month (i.e., 1 through 31)
%
%   OUTPUTS
%	daywknum = day-of-the-week number (0 = Sunday,
%            1 = Monday, ..., 6 = Saturday)
%
%   Notes:  algorithm is only good for the years
%           1700 through 2299 of the Gregorian calendar

%   Reference:  http://www.whichday.com/
%	Copyright (c) 2002 by GPSoft
%
mc = [6 2 2 5 0 3 5 1 4 6 2 4];
c = floor(year/100);
d = year - c*100;
e = d + floor(d/4);
s = e + mc(monthnum) + dayofmonth;
r = s - 7*floor(s/7);

% check for leap year
if rem(year,4) == 0,
    if rem(year,400) == 0,
        leapyear = 1;
    elseif d == 0,
        leapyear = 0;
    else
        leapyear = 1;
    end
else
    leapyear = 0;
end
%
if leapyear == 1,
    if monthnum < 3,
        r = r - 1;
    end
end
%
% deal with different centuries
if c == 17, r = r + 5; end
if c == 18, r = r + 3; end
if c == 19, r = r + 1; end
if c == 21, r = r - 2; end
if c == 22, r = r - 4; end

while r < 0,
    r = r + 7;
end
daywknum = r;
