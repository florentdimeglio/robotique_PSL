%Duration of the experiment
Tmax = 2;

%Serial port name
port_name = "/dev/cu.usbserial-14110";

%CONTROL_LOOP_PERIOD constant in the Arduino sketch
dt_arduino = 5e-3;

%Launch experiment and log data
[log_time, data_values] = get_data_from_robot(port_name, Tmax, dt_arduino);
    
%Plot logged data
figure;
hold on;
legend_labels = data_values.keys;
for i = 1:numel(legend_labels)
    d = legend_labels{i};
    v = data_values(d);
    plot(log_time, v, 'DisplayName', d);
end
hold off;
legend('Location', 'best');
grid on;
