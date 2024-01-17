function loadgalileo
%LOADGALILEO	Galileo orbital parameters
%		Load the Kepler parameters for ideal circular orbits
%               from the matrix in galileokep.mat and maintain as global variables
%
%	loadgalileo
%
%   GLOBAL VARIABLES
%	SVIDV =	vector of satellite identification numbers
%	MV = 	vector of Mean anomalies for the satellites in svid
%		(at reference time) in degrees
%	RV =	vector orbit radii for the satellites in svid 
%		(semi-major axis) in meters
%	TOEV =	vector of reference times for the Kepler parameters 
%		for the satellites in svid (time of ephemeris) in seconds
%	OMGV = 	vector of longitudes of the ascending nodes for the 
%		satellites in svid (at weekly epoch) in degrees
%	INCLV = vector of inclination angles of orbital planes of
%		the satellites in svid (in degrees)

%	Copyright (c) 2003 by GPSoft
%
	global SVIDV MV OMGV RV INCLV TOEV

	load galileokep

	SVIDV = [SVIDV; galileokep(:,1)];
	MV = [MV; galileokep(:,2)];
	OMGV = [OMGV; galileokep(:,3)];
	RV = [RV; galileokep(:,4)];
	INCLV = [INCLV; galileokep(:,5)];
	TOEV = [TOEV; galileokep(:,6)];  
     	
