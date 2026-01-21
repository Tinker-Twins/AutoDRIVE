classdef autodrive_husky < handle
    % AutoDRIVE Husky API
    % Attributes and methods to store and parse Husky data and commands

    properties
        % Husky data
        id;
        throttle;
        differential;
        encoder_ticks;
        encoder_angles;
        position;
        orientation_quaternion;
        orientation_euler_angles;
        angular_velocity;
        linear_acceleration;
        lidar_pointcloud;
        camera_image;
        % Husky commands
        cosim_mode;
        posX_command;
        posY_command;
        posZ_command;
        rotX_command;
        rotY_command;
        rotZ_command;
        rotW_command;
        lin_vel_command;
        ang_vel_command;
    end

    methods
        function parse_data(obj,data,frontcamera_ax,pointcloud_ax,verbose)
            % Actuator feedbacks
            obj.throttle = str2double(data.V1Throttle);
            obj.differential = str2double(data.V1Steering);
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
            obj.lidar_pointcloud = matlab.net.base64decode(data.V1LIDARPointcloud);
            % Camera
            obj.camera_image = matlab.net.base64decode(data.V1FrontCameraImage);
            obj.camera_image = javax.imageio.ImageIO.read(java.io.ByteArrayInputStream(obj.camera_image));
            obj.camera_image = reshape(typecast(obj.camera_image.getData.getDataStorage, 'uint8'), [3,1280,720]);
            obj.camera_image = cat(3,transpose(reshape(obj.camera_image(3,:,:), [1280,720])), ...
                                           transpose(reshape(obj.camera_image(2,:,:), [1280,720])), ...
                                           transpose(reshape(obj.camera_image(1,:,:), [1280,720])));
            if verbose
                fprintf('\n--------------------------------\n')
                fprintf('Receive Data from Husky:\n')
                fprintf('--------------------------------\n\n')
                % Monitor Husky data
                fprintf('Throttle: %f\n',obj.throttle)
                fprintf('Differential: %f\n',obj.differential)
                fprintf('Encoder Ticks: [%d %d]\n',obj.encoder_ticks(1),obj.encoder_ticks(2))
                fprintf('Encoder Angles: [%d %d]\n',obj.encoder_angles(1),obj.encoder_angles(2))
                fprintf('Position: [%f %f %f]\n',obj.position(1),obj.position(2),obj.position(3))
                fprintf('Orientation [Quaternion]: [%f %f %f %f]\n',obj.orientation_quaternion(1),obj.orientation_quaternion(2),obj.orientation_quaternion(3),obj.orientation_quaternion(4))
                fprintf('Orientation [Euler Angles]: [%f %f %f]\n',obj.orientation_euler_angles(1),obj.orientation_euler_angles(2),obj.orientation_euler_angles(3))
                fprintf('Angular Velocity: [%f %f %f]\n',obj.angular_velocity(1),obj.angular_velocity(2),obj.angular_velocity(3))
                fprintf('Linear Acceleration: [%f %f %f]\n',obj.linear_acceleration(1),obj.linear_acceleration(2),obj.linear_acceleration(3))
                % Visualize camera frames
                % Camera
                imshow(imresize(obj.camera_image, 0.5),'Parent',frontcamera_ax);
                drawnow;
                % Visualize LIDAR pointcloud
                % Convert the byte array to numeric values
                numericValues = typecast(uint8(obj.lidar_pointcloud), 'single');
                % Reshape the numeric values into a 3-column matrix (assuming each point has x, y, and z coordinates)
                numPoints = numel(numericValues) / 3;
                pointCloud = reshape(numericValues, 3, numPoints)';                
                % Extract x, y, and z coordinates
                x = pointCloud(:, 1);
                y = pointCloud(:, 2);
                z = pointCloud(:, 3);
                % Plot pointcloud
                colormap jet
                scatter3(x, y, z, 1, z, 'filled','Parent',pointcloud_ax);
                xlabel('X');
                ylabel('Y');
                zlabel('Z');
                title('LIDAR Pointcloud');
                colorbar;
                axis equal;
                drawnow;
            end
        end

        function data = generate_commands(obj,verbose)
            if verbose
                fprintf('\n--------------------------------\n')
                fprintf('Transmit Data to Husky:\n')
                fprintf('--------------------------------\n\n')
                % Monitor Husky control commands
                if obj.cosim_mode == 0
                    cosim_mode_str = 'False';
                else
                    cosim_mode_str = 'True';
                end
                fprintf('Co-Simulation Mode: %s\n',cosim_mode_str)
                fprintf('Position Command: [%d %d %d]\n',obj.posX_command, obj.posY_command, obj.posZ_command)
                fprintf('Rotation Command: [%d %d %d %d]\n',obj.rotX_command, obj.rotY_command, obj.rotZ_command, obj.rotW_command)
                fprintf('Lin. Vel. Command: %d\n',obj.lin_vel_command)
                fprintf('Ang. Vel. Command: %d\n',obj.ang_vel_command)
            end
            data = strcat('42["Bridge",{"V1 CoSim":"',num2str(obj.cosim_mode), ...
                          '","V1 PosX":"',num2str(obj.posX_command), ...
                          '","V1 PosY":"',num2str(obj.posY_command), ...
                          '","V1 PosZ":"',num2str(obj.posZ_command), ...
                          '","V1 RotX":"',num2str(obj.rotX_command), ...
                          '","V1 RotY":"',num2str(obj.rotY_command), ...
                          '","V1 RotZ":"',num2str(obj.rotZ_command), ...
                          '","V1 RotW":"',num2str(obj.rotW_command), ...
                          '","V1 Linear Velocity":"',num2str(obj.lin_vel_command), ...
                          '","V1 Angular Velocity":"',num2str(obj.ang_vel_command),'"}]');
        end
    end
end