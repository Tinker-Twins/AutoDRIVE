import sqlite3

connection = sqlite3.connect('database.db')

with open('schema.sql') as f:
    connection.executescript(f.read())

cursor = connection.cursor()

cursor.execute("INSERT INTO Vehicles (posX, posY, yaw) VALUES (?, ?, ?)", (1.384, 0.655, 5.498))

cursor.execute("INSERT INTO TrafficSigns (posX, posY, name) VALUES (?, ?, ?)", (0.901, 0.835, 'Left Curve'))
cursor.execute("INSERT INTO TrafficSigns (posX, posY, name) VALUES (?, ?, ?)", (1.859, 0.835, 'Right Curve'))
cursor.execute("INSERT INTO TrafficSigns (posX, posY, name) VALUES (?, ?, ?)", (0.901, 0.475, 'Right Curve'))
cursor.execute("INSERT INTO TrafficSigns (posX, posY, name) VALUES (?, ?, ?)", (1.859, 0.475, 'Left Curve'))

cursor.execute("INSERT INTO TrafficLights (posX, posY, status) VALUES (?, ?, ?)", (1.204, 1.130, 'Green'))
cursor.execute("INSERT INTO TrafficLights (posX, posY, status) VALUES (?, ?, ?)", (1.564, 1.130, 'Green'))
cursor.execute("INSERT INTO TrafficLights (posX, posY, status) VALUES (?, ?, ?)", (1.204, 0.180, 'Green'))
cursor.execute("INSERT INTO TrafficLights (posX, posY, status) VALUES (?, ?, ?)", (1.564, 0.180, 'Green'))

connection.commit()
connection.close()
