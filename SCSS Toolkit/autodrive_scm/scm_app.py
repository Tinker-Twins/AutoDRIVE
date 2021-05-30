import sqlite3
from flask import Flask, render_template, request

DTState = 'Vehicles'
VData = [(1, 1.384, 0.655, 5.498)] # (id, posX, posY, yaw)
TSData = [(1, 0.901, 0.835, 'Left Curve'), (2, 1.859, 0.835, 'Right Curve'), (3, 0.901, 0.475, 'Right Curve'), (4, 1.859, 0.475, 'Left Curve')] # (id, posX, posY, name)
TLData = [(1, 1.204, 1.130, 'Green'), (2, 1.564, 1.130, 'Green'), (3, 1.204, 0.180, 'Green'), (4, 1.564, 0.180, 'Green')] # (id, posX, posY, status)
TL1State = TL2State = TL3State = TL4State = 'Green'

app = Flask(__name__)

@app.route('/', methods=['GET', 'POST'])
def index():
    global DTState, VData, TSData, TLData, TL1State, TL2State, TL3State, TL4State

    if request.method == 'POST':
        response = request.form['response']
        if response == 'V':
            DTState = 'Vehicles'
        elif response == 'TL':
            DTState = 'Traffic Lights'
        elif response == 'TS':
            DTState = 'Traffic Signs'
        elif response == 'TL1R':
            TL1State = 'Red'
        elif response == 'TL1Y':
            TL1State = 'Yellow'
        elif response == 'TL1G':
            TL1State = 'Green'
        elif response == 'TL2R':
            TL2State = 'Red'
        elif response == 'TL2Y':
            TL2State = 'Yellow'
        elif response == 'TL2G':
            TL2State = 'Green'
        elif response == 'TL3R':
            TL3State = 'Red'
        elif response == 'TL3Y':
            TL3State = 'Yellow'
        elif response == 'TL3G':
            TL3State = 'Green'
        elif response == 'TL4R':
            TL4State = 'Red'
        elif response == 'TL4Y':
            TL4State = 'Yellow'
        elif response == 'TL4G':
            TL4State = 'Green'

        connection = sqlite3.connect('database.db')
        connection.execute('UPDATE TrafficLights SET status = ?' ' WHERE id = ?', (TL1State, 1))
        connection.execute('UPDATE TrafficLights SET status = ?' ' WHERE id = ?', (TL2State, 2))
        connection.execute('UPDATE TrafficLights SET status = ?' ' WHERE id = ?', (TL3State, 3))
        connection.execute('UPDATE TrafficLights SET status = ?' ' WHERE id = ?', (TL4State, 4))
        connection.commit()

        cursor = connection.cursor()
        cursor.execute('SELECT * from Vehicles')
        VData = cursor.fetchall()
        cursor.execute('SELECT * from TrafficLights')
        TLData = cursor.fetchall()
        cursor.execute('SELECT * from TrafficSigns')
        TSData = cursor.fetchall()
        cursor.close()
        connection.close()

    return render_template('index.html', DTState = DTState, VData = VData, TSData = TSData, TLData = TLData)
