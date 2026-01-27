function [zenith] = sunphysics(longitude,latitude,hr,min,sec,yr,mo,day)
% function to determine the zenith vector of the sun relative to an input
% time and place. Based on 2022 article in renewable energy journal by
% Finley R. Shapiro
arguments (Input)
    % get the position of interest in longitude and latitude (E/N)
    longitude
    latitude
    % get the time of interest in UTC
    hr
    min
    sec
    % get the day of interest in ISO form
    yr
    mo
    day
end

arguments (Output)
    zenith
end

% get the reference values needed
alpha = 23.44; % degrees
phi_s = 134.25; % degrees
s = ((2*pi)/24)*(366.2425/365.2425); % rad/hr
day_length = 23+(56/60)+(4.09/3600); % hr
solstice = datetime(2025,12,21,0,0,0);

% store all of the passed time information in date-time format
date_of_interest = datetime(yr,mo,day,hr,min,sec);

% find the difference between the desired time and reference solstice
time_difference = hours(date_of_interest-solstice);

% compute the 3 zenith vector components
zenith = [-cos(deg2rad(latitude))*sin(s*time_difference+deg2rad(longitude)-deg2rad(phi_s));
          cos(deg2rad(alpha))*cos(deg2rad(latitude))*cos(s*time_difference+deg2rad(longitude)-deg2rad(phi_s))+sin(deg2rad(alpha))*sin(deg2rad(latitude));
          -sin(deg2rad(alpha))*cos(deg2rad(latitude))*cos(s*time_difference+deg2rad(longitude)-deg2rad(phi_s))+cos(deg2rad(alpha))*sin(deg2rad(latitude))];
end