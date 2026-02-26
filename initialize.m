clear; clc;
% --- 1. Load Data ---
filename = 'saved-gps_trips_data_snapped.mat';
if ~exist(filename, 'file'), error('File not found.'); end
load(filename);
if exist('trips_snapped_filtered', 'var')
    data_list = trips_snapped_filtered;
else
    f = fieldnames(ans); data_list = eval(f{1});
end

% --- 2. Trip Selection ---
trip_idx = input(sprintf('Enter Trip ID (1 to %d): ', length(data_list)));
tripStruct = data_list{trip_idx};
data = tripStruct.Data.smoothedTT;

% Capture Raw GPS Baseline for the Geoplot and Export
tractor_lat_gps = data.Latitude; 
tractor_lon_gps = data.Longitude;

% Reference for initial conditions
start_lat = data.Latitude(1);
start_lon = data.Longitude(1);
time_sec = seconds(data.t_rs - data.t_rs(1)); 

% Prepare timeseries for Simulink
ts_speed = timeseries(data.Speed / 3.6, time_sec); 
ts_yaw   = timeseries(data.YawRad, time_sec);
ts_x     = timeseries(data.LocalX, time_sec);
ts_y     = timeseries(data.LocalY, time_sec);
t_final  = time_sec(end);

% --- 3. DIMENSIONS FROM SCHEMATIC ---
init_yaw = data.YawRad(1); 
param.W = 2.5;                
param.L_trac_total = 5.9;     
param.L_trail_total = 13.6;   
param.front_to_kingpin = 1.6; 

% Kinematic offsets
L_wb = 3.8; Fz1 = 73575; Fz2 = 112815;
dist_front_to_cog = L_wb * (Fz2 / (Fz1 + Fz2)); 
param.d = 3.06 - dist_front_to_cog; 
param.L = 8.47; 

fprintf('\nInitialization Complete. Trip %d loaded.\n', trip_idx);