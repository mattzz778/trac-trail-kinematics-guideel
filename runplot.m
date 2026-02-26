%  1. Run Simulation 
model_name = 'tractor_trailer_kinematics';
if ~bdIsLoaded(model_name), load_system(model_name); end
simOut = sim(model_name, 'StopTime', num2str(t_final));
%  2. Retrieve & Interpolate 
try
    logs = simOut.logsout;
    x_raw = logs.get('TrailerX_sim').Values.Data;
    y_raw = logs.get('TrailerY_sim').Values.Data;
    yaw_raw = logs.get('TrailerYaw_sim').Values.Data;
    t_sim = logs.get('TrailerX_sim').Values.Time;
catch
    x_raw = simOut.TrailerX_sim.Data; y_raw = simOut.TrailerY_sim.Data;
    yaw_raw = simOut.TrailerYaw_sim.Data; t_sim = simOut.TrailerX_sim.Time;
end
time_ref = ts_yaw.Time;
x_trail = interp1(t_sim, x_raw, time_ref, 'linear', 'extrap');
y_trail = interp1(t_sim, y_raw, time_ref, 'linear', 'extrap');
yaw_trail_rad = interp1(t_sim, yaw_raw, time_ref, 'linear', 'extrap');
%  3. Angular Correction (Phase Unwrapping) 
yaw_trac_unwrapped = unwrap(ts_yaw.Data);
yaw_trail_unwrapped = unwrap(yaw_trail_rad);
art_angle = atan2(sin(yaw_trac_unwrapped - yaw_trail_unwrapped), ...
                  cos(yaw_trac_unwrapped - yaw_trail_unwrapped));
%  4. GEOGRAPHIC ALIGNMENT FIX 
lat_deg_per_m = 1/111320; 
lon_deg_per_m = 1./(111320 * cosd(tractor_lat_gps)); 
trailer_lat = tractor_lat_gps + (y_trail - ts_y.Data) .* lat_deg_per_m;
trailer_lon = tractor_lon_gps + (x_trail - ts_x.Data) .* lon_deg_per_m;
%  5. KINEMATIC SPEED & LOW-PASS FILTER 
tractor_speed_kmh = ts_speed.Data * 3.6;
% Hitch Offset Calculation (Lever Arm Effect) 
% 1. Calculate Yaw Rate (rad/s)
% We differentiate the yaw angle over time to see how fast it's turning.
dt_avg = mean(diff(ts_speed.Time));
yaw_rate = gradient(ts_yaw.Data) ./ dt_avg; 
% 2. Calculate Velocity Components (in m/s)
v_tractor_ms = tractor_speed_kmh / 3.6;
% Term 1: Forward Pull (The cosine component - what you had before)
term_pull = v_tractor_ms .* cos(art_angle);
% Term 2: Sideways Swing (The new Lever Arm correction)
% 'param.d' is the hitch distance you defined in Step 3
term_swing = (yaw_rate .* param.d) .* sin(art_angle); 
% 3. Combine, Convert to km/h, and Save to YOUR variable name
raw_trailer_speed = abs((term_pull + term_swing) * 3.6);
% --- END NEW CALCULATION ---
%  Filtering (Standard Moving Average) 
windowSize = 3; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
trailer_speed_kmh = filter(b, a, raw_trailer_speed);
% Refined Startup Correction: Sync acceleration
low_speed_threshold = 0.5;
for i = 1:length(trailer_speed_kmh)
    if tractor_speed_kmh(i) < low_speed_threshold
        trailer_speed_kmh(i) = 0;
    elseif i <= windowSize
        trailer_speed_kmh(i) = raw_trailer_speed(i); 
    end
end
% Final physical ceiling (Trailer cannot exceed Tractor speed significantly)
trailer_speed_kmh = min(trailer_speed_kmh, tractor_speed_kmh);
%  6. COMBINED DATA EXPORT 
Combined_Vehicle_Data = table(time_ref, ...
    tractor_lat_gps, tractor_lon_gps, tractor_speed_kmh, rad2deg(ts_yaw.Data), ...
    trailer_lat, trailer_lon, trailer_speed_kmh, rad2deg(yaw_trail_rad), ...
    rad2deg(art_angle), ...
    'VariableNames', {...
    'Time_s', 'Tractor_Lat', 'Tractor_Lon', 'Tractor_Speed_kmh', 'Tractor_Yaw_deg', ...
    'Trailer_Lat', 'Trailer_Lon', 'Trailer_Speed_kmh', 'Trailer_Yaw_deg', 'Articulation_Angle_deg'});
save(sprintf('Full_Vehicle_Data_Trip_%d.mat', trip_idx), 'Combined_Vehicle_Data');
writetable(Combined_Vehicle_Data, sprintf('Full_Vehicle_Data_Trip_%d.csv', trip_idx));
%  7. ALL LABELED PLOTS 
% PLOT 1: Speed Validation (Restored)
figure('Name', 'Plot 1: Speed Profiles', 'Color', 'w');
plot(time_ref, tractor_speed_kmh, 'b', 'LineWidth', 2); hold on;
plot(time_ref, trailer_speed_kmh, 'r-', 'LineWidth', 1.5);
ylabel('Speed [km/h]'); xlabel('Time [s]');
legend('Tractor', 'Trailer (Kinematic-Smoothed)'); grid on;
title('Velocity Profile Comparison');
% PLOT 2: Angular Dynamics (Restored)
figure('Name', 'Plot 2: Angular Dynamics', 'Color', 'w', 'Position', [100 100 900 700]);
subplot(2,1,1);
plot(time_ref, rad2deg(yaw_trac_unwrapped), 'b'); hold on;
plot(time_ref, rad2deg(yaw_trail_unwrapped), 'r--');
ylabel('Heading [deg]'); legend('Tractor', 'Trailer'); grid on;
title('Vehicle Heading Angles');
subplot(2,1,2);
plot(time_ref, rad2deg(art_angle), 'g', 'LineWidth', 1.2);
ylabel('Articulation [deg]'); xlabel('Time [s]'); grid on; ylim([-60 60]);
title('Relative Articulation Angle');
% PLOT 3: Detailed Local Path (Corrected Cab-Over Dimensions & Axles)
figure('Name', 'Plot 3: Detailed Local Path', 'Color', 'w', 'Position', [150 150 900 800]);
hold on; grid on; axis equal;
% Helper: Draw Body (anchored at Hitch)
drawBoxPivot = @(px, py, yaw, L_fwd, L_rear, W, color, alpha) fill(...
    px + [L_fwd, L_fwd, -L_rear, -L_rear]*cos(yaw) - [-W/2, W/2, W/2, -W/2]*sin(yaw), ...
    py + [L_fwd, L_fwd, -L_rear, -L_rear]*sin(yaw) + [-W/2, W/2, W/2, -W/2]*cos(yaw), ...
    color, 'FaceAlpha', alpha, 'EdgeColor', color, 'HandleVisibility', 'off');
% Helper: Draw Axles (Thick Black Lines)
drawAxle = @(px, py, yaw, dist_from_pivot, W_axle) plot(...
    px + dist_from_pivot*cos(yaw) + [-W_axle/2, W_axle/2]*sin(yaw), ...
    py + dist_from_pivot*sin(yaw) - [-W_axle/2, W_axle/2]*cos(yaw), ...
    'k', 'LineWidth', 4, 'HandleVisibility', 'off');
% Calculate Hitch Position
H_x = ts_x.Data - param.d * cos(ts_yaw.Data);
H_y = ts_y.Data - param.d * sin(ts_yaw.Data);
idx_snapshots = round(linspace(1, length(time_ref), 5)); 
for i = idx_snapshots
    %  TRACTOR DIMENSIONS (Corrected Cab-Over) 
    % Front Axle is 3.06m ahead of Hitch.
    % Front Overhang (Bumper to Axle) set to 1.4m (Standard Cab-Over)
    % Rear Overhang estimated 1.5m behind hitch to cover rear axle.
    trac_overhang = 1.4; 
    t_fwd = 3.06 + trac_overhang; % 4.46m total ahead of hitch
    t_rear = 1.5; 
    drawBoxPivot(H_x(i), H_y(i), ts_yaw.Data(i), t_fwd, t_rear, param.W, 'b', 0.2);
    
    %  TRACTOR AXLES 
    drawAxle(H_x(i), H_y(i), ts_yaw.Data(i), 3.06, param.W); % Front Axle
    drawAxle(H_x(i), H_y(i), ts_yaw.Data(i), -0.74, param.W); % Rear Axle (-0.74m)
    %  TRAILER DIMENSIONS 
    % Nose: 1.6m ahead of Hitch | Rear: 12.0m behind Hitch
    tr_fwd = 1.6; 
    tr_rear = 12.0;
    drawBoxPivot(H_x(i), H_y(i), yaw_trail_rad(i), tr_fwd, tr_rear, param.W, 'r', 0.4);
    
    %  TRAILER AXLES 
    drawAxle(H_x(i), H_y(i), yaw_trail_rad(i), -8.47, param.W); % Bogie Center
    
    %  5TH WHEEL PIVOT 
    plot(H_x(i), H_y(i), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
end
plot(ts_x.Data, ts_y.Data, 'b-', 'LineWidth', 1, 'DisplayName', 'Tractor GPS Path');
plot(x_trail, y_trail, 'r--', 'DisplayName', 'Simulated Trailer Path');
ylabel('Y [m]'); xlabel('X [m]'); legend; 
title('Kinematics Verification: Corrected Cab & Axles');
% PLOT 4: Aligned Geographic Trajectory (Restored)
figure('Name', 'Plot 4: Geographic Path Aligned', 'Color', 'w');
geoplot(tractor_lat_gps, tractor_lon_gps, 'b', 'LineWidth', 2, 'DisplayName', 'Tractor (GPS)'); 
hold on;
geoplot(trailer_lat, trailer_lon, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Trailer (Sim)');
geobasemap satellite; 
title(sprintf('Trip %d: Aligned Satellite View', trip_idx));
legend;
% PLOT 5: FIGURE 5.5 (Yaw Signal Comparison)
figure('Name', 'Figure 5.5: Yaw Signal Comparison', 'Color', 'w', 'Position', [100 100 1000 500]);
subplot(1, 2, 1);
plot(time_ref, rad2deg(ts_yaw.Data), 'r-', 'LineWidth', 1);
ylabel('Heading [deg]'); xlabel('Time [s]');
title('Raw GPS Yaw (Wrapped)'); grid on;
ylim([-190 190]); yticks(-180:90:180);
subplot(1, 2, 2);
plot(time_ref, rad2deg(yaw_trac_unwrapped), 'b-', 'LineWidth', 1.5);
ylabel('Heading [deg]'); xlabel('Time [s]');
title('Filtered & Unwrapped Yaw (Continuous)'); grid on;
sgtitle('FIGURE 5.5: Comparison Plot of Raw GPS Yaw vs. Unwrapped/Filtered Yaw Signal');

% --- NEW PLOT 6: WRAPPED TRACTOR VS TRAILER YAW (From user request) ---
figure('Name', 'Plot 6: Vehicle Heading Angles (Wrapped)', 'Color', 'w', 'Position', [200 200 900 400]);
% Force both signals to strictly wrap between -pi and pi for safe plotting
yaw_trac_wrapped_safe = atan2(sin(ts_yaw.Data), cos(ts_yaw.Data));
yaw_trail_wrapped_safe = atan2(sin(yaw_trail_rad), cos(yaw_trail_rad));

plot(time_ref, rad2deg(yaw_trac_wrapped_safe), 'b-', 'LineWidth', 1.2); hold on;
plot(time_ref, rad2deg(yaw_trail_wrapped_safe), 'r--', 'LineWidth', 1.2);
ylabel('Heading [deg]'); 
xlabel('Time [s]'); 
title('Vehicle Heading Angles');
legend('Tractor', 'Trailer'); 
grid on;
ylim([-190 190]); 
yticks(-180:90:180);