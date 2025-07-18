classdef roboracer < handle
    % AutoDRIVE RoboRacer MATLAB API
    % Attributes and methods to store and parse RoboRacer data and commands

    properties
        % RoboRacer data
        id = uint16(1);
        throttle = double(0.0);
        steering = double(0.0);
        speed = double(0.0);
        encoder_ticks = int32([0; 0]);
        encoder_angles = double([0; 0]);
        position = double([0; 0; 0]);
        orientation_quaternion = double([0; 0; 0; 0]);
        orientation_euler_angles = double([0; 0; 0]);
        angular_velocity = double([0; 0; 0]);
        linear_acceleration = double([0; 0; 0]);
        lidar_scan_rate = double(0.0);
        lidar_range_array = double(zeros(1081,1));
        lidar_intensity_array = double([]);
        front_camera_image = uint8(zeros(108,192,3));
        % Race data
        lap_count = uint16(0);
        lap_time = double(0.0);
        last_lap_time = double(0.0);
        best_lap_time = double(0.0);
        collision_count = uint16(0);
        % RoboRacer commands
        throttle_command = double(0);
        steering_command = double(0);
        % Simulation commands
        reset_command = "False";
    end

    methods
        function parse_data(obj,data)
            % Actuator feedbacks
            obj.throttle = str2double(data.V1Throttle);
            obj.steering = str2double(data.V1Steering);
            % Speed
            obj.speed = str2double(data.V1Speed);
            % Wheel encoders
            obj.encoder_ticks = cell2mat(textscan(data.V1EncoderTicks,'%d'));
            obj.encoder_angles = cell2mat(textscan(data.V1EncoderAngles,'%f'));
            % IPS
            obj.position = cell2mat(textscan(data.V1Position,'%f'));
            % IMU
            obj.orientation_quaternion = cell2mat(textscan(data.V1OrientationQuaternion,'%f'));
            obj.orientation_euler_angles = cell2mat(textscan(data.V1OrientationEulerAngles,'%f'));
            obj.angular_velocity = cell2mat(textscan(data.V1AngularVelocity,'%f'));
            obj.linear_acceleration = cell2mat(textscan(data.V1LinearAcceleration,'%f'));
            % LIDAR
            obj.lidar_scan_rate = str2double(data.V1LIDARScanRate);
            obj.lidar_intensity_array = [];
            decodedBytes = matlab.net.base64decode(data.V1LIDARRangeArray);
            byteStream = java.io.ByteArrayInputStream(decodedBytes);
            gzipStream = java.util.zip.GZIPInputStream(byteStream);
            inputStreamReader = java.io.InputStreamReader(gzipStream, 'UTF-8');
            bufferedReader = java.io.BufferedReader(inputStreamReader);
            str = "";
            line = bufferedReader.readLine();
            while ~isempty(line)
                str = str + string(line) + newline;
                line = bufferedReader.readLine();
            end
            obj.lidar_range_array = sscanf(str, '%f');
            % Camera
            obj.front_camera_image = matlab.net.base64decode(data.V1FrontCameraImage);
            obj.front_camera_image = javax.imageio.ImageIO.read(java.io.ByteArrayInputStream(obj.front_camera_image));
            obj.front_camera_image = reshape(typecast(obj.front_camera_image.getData.getDataStorage, 'uint8'), [3,192,108]);
            obj.front_camera_image = cat(3,transpose(reshape(obj.front_camera_image(3,:,:), [192,108])), ...
                                           transpose(reshape(obj.front_camera_image(2,:,:), [192,108])), ...
                                           transpose(reshape(obj.front_camera_image(1,:,:), [192,108])));
            % Race data
            obj.lap_count = uint16(str2double(data.V1LapCount));
            obj.lap_time = str2double(data.V1LapTime);
            obj.last_lap_time = str2double(data.V1LastLapTime);
            obj.best_lap_time = str2double(data.V1BestLapTime);
            obj.collision_count = uint16(str2double(data.V1Collisions));
        end

        function data = generate_commands(obj)
            data = strcat('42["Bridge",{"V1 Throttle":"',num2str(obj.throttle_command), ...
                          '","V1 Steering":"',num2str(obj.steering_command), ...
                          '","Reset":"',num2str(obj.reset_command),'"}]');
        end
    end
end