%% --- 1. SETUP & CONFIGURATION ---
clear writer; clc;

% Check if data exists
if ~exist('Combined_Vehicle_Data', 'var')
    error(' Data not found. Please run your simulation script first!');
end

% Create Unique Bag Name
str_time = char(datetime('now', 'Format', 'yyyy_MM_dd_HH_mm_ss'));
if exist('trip_idx', 'var')
    folder_name = sprintf('Saved_Trip_%d_Data_%s', trip_idx, str_time);
else
    folder_name = ['Saved_Trip_Data_' str_time];
end
bag_path = fullfile(pwd, folder_name);

% Create the Writer
try
    writer = ros2bagwriter(bag_path); 
    fprintf(' Created bag writer at:\n   %s\n', bag_path);
catch ME
    error(' ros2bagwriter failed: %s', ME.message); 
end

% CONFIGURATION
topic_name = "/tractor_01/saved_vehicle_control";
msg_type   = "saved_msgs/TractorTrailerState"; 


%% --- PREPARE SOURCE DATA (PRE-FILTERED, UPSAMPLED, CARLA FIXED) ---
orig_time = Combined_Vehicle_Data.Time_s;

% 1. DEFINE NEW SAMPLING RATE (100 Hz)
target_dt = 0.01; 
time_vec = (orig_time(1) : target_dt : orig_time(end))'; 

% 2. EXTRACT & PRE-FILTER ORIGINAL DATA (The Anti-Jitter Fix)
% Smooth over 5 original frames to remove sensor noise before upsampling
filter_window = 5; 

orig_truck_x = smoothdata(ts_x.Data, 'sgolay', filter_window); 
orig_truck_y = smoothdata(ts_y.Data, 'sgolay', filter_window); 

if exist('ts_yaw', 'var')
    orig_truck_yaw = ts_yaw.Data;
else
    orig_truck_yaw = deg2rad(Combined_Vehicle_Data.Tractor_Yaw_deg);
end

% Trailer Position (using rear axle)
orig_trailer_x = smoothdata(x_trail, 'sgolay', filter_window); 
orig_trailer_y = smoothdata(y_trail, 'sgolay', filter_window); 

if exist('yaw_trail_rad', 'var')
    orig_trailer_yaw = yaw_trail_rad;
else
    orig_trailer_yaw = deg2rad(Combined_Vehicle_Data.Trailer_Yaw_deg);
end

% 3. UNWRAP AND FILTER YAWS (Crucial for smooth turning logic)
orig_truck_yaw   = smoothdata(unwrap(orig_truck_yaw), 'sgolay', filter_window);
orig_trailer_yaw = smoothdata(unwrap(orig_trailer_yaw), 'sgolay', filter_window);


% 4. INTERPOLATE ALL SIGNALS TO NEW TIME VECTOR
% Changing to 'spline' instead of 'pchip'. Spline guarantees continuous acceleration 
% (2nd derivative), which makes physics engines much happier through corners!
interp_method = 'spline'; 

truck_x   = interp1(orig_time, orig_truck_x, time_vec, interp_method);
truck_y   = interp1(orig_time, orig_truck_y, time_vec, interp_method);
trailer_x = interp1(orig_time, orig_trailer_x, time_vec, interp_method);
trailer_y = interp1(orig_time, orig_trailer_y, time_vec, interp_method);

truck_yaw_raw   = interp1(orig_time, orig_truck_yaw, time_vec, interp_method);
trailer_yaw_raw = interp1(orig_time, orig_trailer_yaw, time_vec, interp_method);


% --- 5. MATHEMATICALLY PERFECT SPEEDS FOR BOTH VEHICLES ---
% Tractor Speed
vx_truck = gradient(truck_x) ./ target_dt;
vy_truck = gradient(truck_y) ./ target_dt;
speed_kmh = sqrt(vx_truck.^2 + vy_truck.^2) .* 3.6;
speed_kmh = smoothdata(speed_kmh, 'movmean', 20); % Polish at 100Hz

% Trailer Speed
vx_trailer = gradient(trailer_x) ./ target_dt;
vy_trailer = gradient(trailer_y) ./ target_dt;
trail_speed_data = sqrt(vx_trailer.^2 + vy_trailer.^2) .* 3.6;
trail_speed_data = smoothdata(trail_speed_data, 'movmean', 20); % Polish at 100Hz


% 6. APPLY CARLA INVERSIONS AND WRAP TO [-PI, PI]
% Invert the sign (-) to fix CARLA's Left-Handed turns
truck_yaw   = atan2(sin(-truck_yaw_raw), cos(-truck_yaw_raw));
trailer_yaw = atan2(sin(-trailer_yaw_raw), cos(-trailer_yaw_raw));


%% --- 2. WRITE LOOP ---
fprintf(' Writing %d frames (Fully Smoothed & Fixed Coordinates)...\n', length(time_vec));
msg = ros2message(msg_type);

for i = 1:length(time_vec)
    
    % --- A. TRACTOR CONTROLS ---
    msg.tractor_velocity = double(speed_kmh(i)); % km/h
    
    is_reverse = (speed_kmh(i) < -0.1);
    msg.tractor_reverse = boolean(is_reverse); 
    
    if is_reverse
        msg.tractor_gear = int8(-1); 
    else
        msg.tractor_gear = int8(1);
    end
    
    % Placeholders
    msg.tractor_steer = double(0);
    msg.tractor_throttle = double(0);
    msg.tractor_brake = double(0);
    msg.tractor_manual = boolean(0);
    
    % --- B. TRACTOR POSITION ---
    msg.tractor_x = double(truck_x(i));
    msg.tractor_y = double(truck_y(i));
    msg.tractor_yaw = double(truck_yaw(i)); 
    msg.tractor_pitch = double(0);
    msg.tractor_roll = double(0);
    
    % --- C. TRAILER POSITION ---
    msg.trailer_x = double(trailer_x(i));
    msg.trailer_y = double(trailer_y(i));
    msg.trailer_yaw = double(trailer_yaw(i)); 
    msg.trailer_roll = double(0);
    msg.trailer_pitch = double(0);
    
    % --- D. TRAILER SPEED ---
    try
        msg.trailer_velocity = double(trail_speed_data(i));
    catch
        % If field doesn't exist, do nothing.
    end
    
    % --- E. Timestamp ---
    sim_time = time_vec(i);
    ros_time = struct('sec', int32(floor(sim_time)), ...
                      'nanosec', uint32(mod(sim_time, 1) * 1e9));
    
    % --- F. Write ---
    try
        write(writer, topic_name, ros_time, msg);
    catch ME
        error(' Frame %d Failed: %s', i, ME.message);
    end
    
    if mod(i, 500) == 0
        fprintf('   ... written frame %d\n', i);
    end
end

clear writer; 
fprintf(' Success! Bag saved.\n');