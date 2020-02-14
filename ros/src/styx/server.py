#!/usr/bin/env python

import eventlet
eventlet.monkey_patch(socket=True, select=True, time=True)

import eventlet.wsgi
import socketio
import time
from flask import Flask, render_template

from bridge import Bridge
from conf import conf
from params_config.params_config import ParamsConfig

sio = socketio.Server()
app = Flask(__name__)
msgs = []
lastTime = time.time()

dbw_enable = False

@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)

def send(topic, data):
    s = 1
    msgs.append((topic, data))
    #sio.emit(topic, data=json.dumps(data), skip_sid=True)

bridge = Bridge(conf, send)

@sio.on('telemetry')
def telemetry(sid, data):
    global dbw_enable
    if data["dbw_enable"] != dbw_enable:
        dbw_enable = data["dbw_enable"]
        bridge.publish_dbw_status(dbw_enable)
    bridge.publish_odometry(data)
    for i in range(len(msgs)):
        topic, data = msgs.pop(0)
        sio.emit(topic, data=data, skip_sid=True)

@sio.on('control')
def control(sid, data):
    bridge.publish_controls(data)

#######################################################################
#######################################################################
##### USE CONFIG TO TURN OFF/ON THESE SIMULATOR = OFF, CARLA = ON #####
#######################################################################
@sio.on('obstacle')
def obstacle(sid, data):
    if ParamsConfig.isSite():
        # running on Carla
        bridge.publish_obstacles(data)
    else:
        # running in simulator
        pass

@sio.on('lidar')
def obstacle(sid, data):
    if ParamsConfig.isSite():
        # running on Carla
        bridge.publish_lidar(data)
    else:
        # running in simulator
        pass
#######################################################################
#######################################################################
@sio.on('trafficlights')
def trafficlights(sid, data):
    bridge.publish_traffic(data)

@sio.on('image')
def image(sid, data):
    bridge.publish_camera(data) # existing line

if __name__ == '__main__':

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
