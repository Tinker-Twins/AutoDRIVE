classdef example_husky < WebSocketServer
    % AutoDRIVE Husky Example
    % Implements a MATLAB WebSocket server to communicate with AutoDRIVE
    % Simulator
    
    properties
        husky_1 = autodrive_husky;
        frontcamera_fig = figure(1)
        frontcamera_ax = axes;
        pointcloud_fig = figure(2)
        pointcloud_ax = axes;
    end
    
    methods
        function obj = example_husky(varargin)
            % Constructor
            obj@WebSocketServer(varargin{:});
        end
    end
    
    methods (Access = protected)
        function onOpen(obj,conn,message)
            fprintf('%s\n',message)
        end

        function onTextMessage(obj,conn,message)
            % This function recieves the incoming data from WebSocket,
            % parses incoming data, prepares outgoing data, and transmits
            % the outgoing data over WebSocket

            % Recieve and decode incoming JSON message
            in_data = jsondecode(message(13:end-1));
            % Parse incoming data
            obj.husky_1.parse_data(in_data,obj.frontcamera_ax,obj.pointcloud_ax,true);
            
            % Prepare outgoing data
            obj.husky_1.cosim_mode = 0;
            obj.husky_1.posX_command = 0;
            obj.husky_1.posY_command = 0;
            obj.husky_1.posZ_command = 0;
            obj.husky_1.rotX_command = 0;
            obj.husky_1.rotY_command = 0;
            obj.husky_1.rotZ_command = 0;
            obj.husky_1.rotW_command = 0;
            obj.husky_1.lin_vel_command = 1;
            obj.husky_1.ang_vel_command = 0;
            % Encode outgoing data as a JSON message
            out_data = obj.husky_1.generate_commands(true);
            % Transmit outgoing JSON message
            conn.send(out_data);
        end
        
        function onBinaryMessage(obj,conn,bytearray)
            % This function sends an echo back to the client
            conn.send(bytearray); % Echo
        end
        
        function onError(obj,conn,message)
            fprintf('%s\n',message)
        end
        
        function onClose(obj,conn,message)
            fprintf('%s\n',message)
        end
    end
end

%autodrive = example_husky(4567)
%autodrive.stop
%delete(autodrive)
%clear autodrive