#!/usr/bin/env python

# Import libraries
import socketio
import eventlet
from flask import Flask
from itertools import chain
import autodrive

################################################################################

# Initialize vehicle(s)
nigel_1 = autodrive.Nigel()
nigel_1.id = 'V1'

# Initialize traffic light(s)
tl_1 = autodrive.TrafficLight()
tl_2 = autodrive.TrafficLight()
tl_3 = autodrive.TrafficLight()
tl_4 = autodrive.TrafficLight()
tl_1.id = 'TL1'
tl_2.id = 'TL2'
tl_3.id = 'TL3'
tl_4.id = 'TL4'

# Initialize the server
sio = socketio.Server()

# Flask (web) app
app = Flask(__name__) # '__main__'

# Registering "connect" event handler for the server
@sio.on('connect')
def connect(sid, environ):
    print('Connected!')

# Registering "Bridge" event handler for the server
@sio.on('Bridge')
def bridge(sid, data):
    if data:
        
        ########################################################################
        # PERCEPTION
        ########################################################################

        # Vehicle data
        nigel_1.parse_data(data, verbose=True)

        # Traffic light data
        tl_1.parse_data(data, verbose=True)
        tl_2.parse_data(data, verbose=True)
        tl_3.parse_data(data, verbose=True)
        tl_4.parse_data(data, verbose=True)

        '''
        Implement peception stack here.
        '''

        ########################################################################
        # PLANNING
        ########################################################################

        '''
        Implement planning stack here.
        '''

        ########################################################################
        # CONTROL
        ########################################################################

        '''
        Implement control stack here.
        '''

        # Vehicle control
        nigel_1.throttle_command = 1 # [-1, 1]
        nigel_1.steering_command = 1 # [-1, 1]
        nigel_1.headlights_command = 1 # [0 = disabled, 1 = low beam, 2 = high beam]
        nigel_1.indicators_command = 3 # [0 = disabled, 1 = left turn indicator, 2 = right turn indicator, 3 = hazard indicator]

        # Traffic light control
        tl_1.command = 1 # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
        tl_2.command = 2 # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
        tl_3.command = 3 # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
        tl_4.command = 3 # [0 = disabled, 1 = red, 2 = yellow, 3 = green]

        ########################################################################

        json_msg = nigel_1.generate_commands(verbose=True) # Generate vehicle 1 message
        json_msg.update(tl_1.generate_commands(verbose=True)) # Append traffic light 1 message
        json_msg.update(tl_2.generate_commands(verbose=True)) # Append traffic light 2 message
        json_msg.update(tl_3.generate_commands(verbose=True)) # Append traffic light 3 message
        json_msg.update(tl_4.generate_commands(verbose=True)) # Append traffic light 4 message

        try:
            sio.emit('Bridge', data=json_msg)
        except Exception as exception_instance:
            print(exception_instance)

################################################################################

if __name__ == '__main__':
    app = socketio.Middleware(sio, app) # Wrap flask application with socketio's middleware
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app) # Deploy as an eventlet WSGI server
