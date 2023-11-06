#!/usr/bin/env python

# Import libraries
import socketio
import eventlet
from flask import Flask
import autodrive

################################################################################

# Initialize vehicle(s)
f1tenth_1 = autodrive.F1TENTH()
f1tenth_1.id = 'V1'

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
        f1tenth_1.parse_data(data, verbose=True)

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
        f1tenth_1.throttle_command = 1 # [-1, 1]
        f1tenth_1.steering_command = 1 # [-1, 1]

        ########################################################################

        json_msg = f1tenth_1.generate_commands(verbose=True) # Generate vehicle 1 message

        try:
            sio.emit('Bridge', data=json_msg)
        except Exception as exception_instance:
            print(exception_instance)

################################################################################

if __name__ == '__main__':
    app = socketio.Middleware(sio, app) # Wrap flask application with socketio's middleware
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app) # Deploy as an eventlet WSGI server
