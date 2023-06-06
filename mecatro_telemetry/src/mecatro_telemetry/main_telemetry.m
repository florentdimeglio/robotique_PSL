%Duration of the experiment
Tmax = 10;

%Serial port name
port_name = "/dev/cu.usbserial-14110";

%CONTROL_LOOP_PERIOD constant in the Arduino sketch
dt_arduino = 5e-3;
[log_time, data_values] = get_data_from_robot(port_name, Tmax, dt_arduino);