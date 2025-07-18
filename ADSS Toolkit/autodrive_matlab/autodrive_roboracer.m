classdef autodrive_roboracer < handle
    % AutoDRIVE RoboRacer API
    % Attributes and methods to store and parse RoboRacer data and commands

    properties
        % RoboRacer data
        id;
        throttle;
        steering;
        speed;
        encoder_ticks;
        encoder_angles;
        position;
        orientation_quaternion;
        orientation_euler_angles;
        angular_velocity;
        linear_acceleration;
        lidar_scan_rate;
        lidar_range_array;
        lidar_intensity_array;
        front_camera_image;
        % Race data
        lap_count;
        lap_time;
        last_lap_time;
        best_lap_time;
        collision_count;
        % RoboRacer commands
        throttle_command;
        steering_command;
        % Simulation commands
        reset_command = "False";
    end

    methods
        function parse_data(obj,data,frontcamera_ax,laserscan_ax,verbose)
            % Actuator feedbacks
            obj.throttle = str2double(data.V1Throttle);
            obj.steering = str2double(data.V1Steering);
            % Speed
            obj.speed = str2double(data.V1Speed);
            % Wheel encoders
            obj.encoder_ticks = cell2mat(textscan(data.V1EncoderTicks,'%d'));
            obj.encoder_angles = cell2mat(textscan(data.V1EncoderAngles,'%d'));
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
            % Cameras
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
            if verbose
                fprintf('\n--------------------------------\n')
                fprintf('Receive Data from RoboRacer:\n')
                fprintf('--------------------------------\n\n')
                % Monitor RoboRacer data
                fprintf('Throttle: %f\n',obj.throttle)
                fprintf('Steering: %f\n',obj.steering)
                fprintf('Speed: %f\n',obj.speed)
                fprintf('Encoder Ticks: [%d %d]\n',obj.encoder_ticks(1),obj.encoder_ticks(2))
                fprintf('Encoder Angles: [%d %d]\n',obj.encoder_angles(1),obj.encoder_angles(2))
                fprintf('Position: [%f %f %f]\n',obj.position(1),obj.position(2),obj.position(3))
                fprintf('Orientation [Quaternion]: [%f %f %f %f]\n',obj.orientation_quaternion(1),obj.orientation_quaternion(2),obj.orientation_quaternion(3),obj.orientation_quaternion(4))
                fprintf('Orientation [Euler Angles]: [%f %f %f]\n',obj.orientation_euler_angles(1),obj.orientation_euler_angles(2),obj.orientation_euler_angles(3))
                fprintf('Angular Velocity: [%f %f %f]\n',obj.angular_velocity(1),obj.angular_velocity(2),obj.angular_velocity(3))
                fprintf('Linear Acceleration: [%f %f %f]\n',obj.linear_acceleration(1),obj.linear_acceleration(2),obj.linear_acceleration(3))
                fprintf('Lap Count: %f\n',obj.lap_count)
                fprintf('Lap Time: %f\n',obj.lap_time)
                fprintf('Last Lap Time: %f\n',obj.last_lap_time)
                fprintf('Best Lap Time: %f\n',obj.best_lap_time)
                fprintf('Collision Count: %d\n',obj.collision_count)
                % Visualize camera frame
                imshow(imresize(obj.front_camera_image, 0.5),'Parent',frontcamera_ax);
                drawnow;
                % Visualize LIDAR laser scan
                angles = deg2rad(-45:0.25:225);
                scatter(laserscan_ax, angles,obj.lidar_range_array,1,'filled','red');
                title('LIDAR Laser Scan');
                drawnow;
            end
        end

        function data = generate_commands(obj,verbose)
            if verbose
                fprintf('\n--------------------------------\n')
                fprintf('Transmit Data to RoboRacer:\n')
                fprintf('--------------------------------\n\n')
                % Monitor RoboRacer control commands
                fprintf('Throttle Command: %d\n',obj.throttle_command)
                fprintf('Steering Command: %d\n',obj.steering_command)
                fprintf('Reset Command: %s\n',mat2str(obj.reset_command))
            end
            data = strcat('42["Bridge",{"V1 Throttle":"',num2str(obj.throttle_command), ...
                          '","V1 Steering":"',num2str(obj.steering_command), ...
                          '","Reset":"',num2str(obj.reset_command),'"}]');
        end
    end
end