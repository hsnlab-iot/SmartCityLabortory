#!/usr/bin/env python3.7

import glob
import os
import sys
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

import random
import time
import numpy as np
import cv2
import queue
import copy
import re

IM_WIDTH = 640
IM_HEIGHT = 480

last_rgb = None
last_seg = None
last_ins = None

cache = set()
lnum = 0
snum = 0

def filer_image(seg, ins, t, fn):
    global cache
    f = np.where(seg == t)
    if len(f[0]) == 0:
        return []

    #print("Typte %d found! %.6d" % (t, fn))

    # draw all objetcs
    segi = np.where(seg == t, 255, 0)
    segi = segi.reshape(IM_HEIGHT, IM_WIDTH)
    #r = cv2.imwrite("images/image%.6d_%d_all.jpg" % (fn, t), np.uint8(segi))
    ret = []

    # draw objects individually
    values = ins[f[0]]
    for x in np.unique(values):
        insi = np.where((ins == x) & (seg == t), 255, 0)
        insi = insi.reshape(IM_HEIGHT, IM_WIDTH)

        contours, _ = cv2.findContours(insi.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            # Calculate bounding box for each contour
            cx, cy, cw, ch = cv2.boundingRect(contour)
            if cx < 0:
                cx = 0
            if cy < 0:
                cy = 0
            if cw >= 16 and ch >= 16:
                #print(f"{fn}, {x}, {cx}, {cy}, {cw}, {ch}")
                # Save image
                #mask = np.zeros_like(insi)
                #mask[cy:cy+ch, cx:cx+cw] = 255
                #out = cv2.bitwise_and(insi, mask)
                #r = cv2.imwrite("images/image%.6d_%d_%i_%i.jpg" % (fn, t, x, cx), out)
                t = (x, cx, cy, cw, ch)
                if not t in cache:
                    ret.append((cx, cy, cw, ch))
                    cache.add(t)

    return ret

def process_checkall():

    if last_rgb == None or last_seg == None or last_ins == None:
        #if last_rgb is not None:
        #    last_rgb.save_to_disk("images/image%.6d.jpg" % last_rgb.frame)
        return
    if not (last_rgb.frame == last_seg.frame and last_rgb.frame == last_ins.frame):
        return
    
    cai = last_rgb.frame
    #print(f"CheckAll {cai}")
    global lnum, snum

    ix = np.array(last_seg.raw_data)[2::4]
    ii = np.array(last_ins.raw_data)[::4] * 256 + np.array(last_ins.raw_data)[1::4]
    ir = np.array(last_rgb.raw_data)

    found = False
    #print(np.unique(ix))
    lights = filer_image(ix, ii, 7, cai)
    signs = filer_image(ix, ii, 8, cai)
    if len(lights) + len(signs) > 0:
        print(cai, lights, signs)
        bgri = ir.reshape((IM_HEIGHT, IM_WIDTH, 4))
        bgri = cv2.cvtColor(bgri, cv2.COLOR_BGRA2BGR)
        bgri_o = copy.deepcopy(bgri)
        with open("labels/image%.6d.txt" % cai, "w") as file:
            for i in lights:
                cv2.rectangle(bgri, (i[0], i[1]), (i[0]+i[2], i[1]+i[3]), (0,255,0), 2)
                file.write("0, %.6f, %.6f, %.6f, %.6f\n" % ((i[0]+i[2]/2.0)/IM_WIDTH, (i[1]+i[3]/2.0)/IM_HEIGHT, i[2]/IM_WIDTH, i[3]/IM_HEIGHT))
            for i in signs:
                cv2.rectangle(bgri, (i[0], i[1]), (i[0]+i[2], i[1]+i[3]), (0,0,255), 2)
                file.write("1, %.6f, %.6f, %.6f, %.6f\n" % ((i[0]+i[2]/2.0)/IM_WIDTH, (i[1]+i[3]/2.0)/IM_HEIGHT, i[2]/IM_WIDTH, i[3]/IM_HEIGHT))

        r = cv2.imwrite("images/image%.6d_boxes.jpg" % cai, bgri)
        r = cv2.imwrite("images/image%.6d.jpg" % cai, bgri_o)
        found = True

        lnum = lnum + len(lights)
        snum = snum + len(signs)
        print(f"Lights: {lnum}, Signs: {snum}")

    #if found:
        #last_rgb.save_to_disk("images/image%.6d.jpg" % last_rgb.frame)
        #last_seg.save_to_disk("images/image%.6d_seg.jpg" % last_seg.frame, carla.ColorConverter.CityScapesPalette)
        #last_ins.save_to_disk("images/image%.6d_ins.jpg" % last_ins.frame)
    
    #print(f"CheckAllEnd {cai}")

def process_img(image):
    global last_rgb
    last_rgb = image
    #print(f"new RGB {last_rgb.frame}")
    process_checkall()

def process_seg_img(image):
    global last_seg
    last_seg = image
    #print("new SEG")
    process_checkall()

    #image.save_to_disk("images/image%.6d_seg.jpg" % image.frame, carla.ColorConverter.CityScapesPalette)

def process_ins_img(image):
    global last_ins
    last_ins = image
    #print("new INS")
    process_checkall()

    #image.save_to_disk("images/image%.6d_ins.jpg" % image.frame)

actor_list = []
try:
    client = carla.Client('10.6.6.20', 2000)
    client.set_timeout(2.0)

    world = client.get_world()
    #world = client.load_world("Town05")

    blueprint_library = world.get_blueprint_library()

    #blueprints = [bp for bp in world.get_blueprint_library().filter('*')]
    #for blueprint in blueprints:
    #    print(blueprint.id)
    #    for attr in blueprint:
    #        print('  - {}'.format(attr))

    bp = blueprint_library.filter('charger_police')[0]
    print(bp)

    #sp = int(random.uniform(0, len(world.get_map().get_spawn_points())-1))
    sp = 138
    print("Spawn point (from %d): %d" % (len(world.get_map().get_spawn_points()), sp))
    spawn_point = world.get_map().get_spawn_points()[sp]

    vehicle = world.spawn_actor(bp, spawn_point)
    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
    # vehicle.set_autopilot(True)  # if you just wanted some NPCs to drive.

    actor_list.append(vehicle)

    vehicle.set_autopilot(True)


    # https://carla.readthedocs.io/en/latest/cameras_and_sensors
    # get the blueprint for this sensor
    blueprint = blueprint_library.find('sensor.camera.rgb')
    # change the dimensions of the image
    blueprint.set_attribute('image_size_x', f'{IM_WIDTH}')
    blueprint.set_attribute('image_size_y', f'{IM_HEIGHT}')
    blueprint.set_attribute('fov', '110')

    # Adjust sensor relative to vehicle
    spawn_point = carla.Transform(carla.Location(x=2.5, z=1.4))

    # spawn the sensor and attach to vehicle.
    sensor = world.spawn_actor(blueprint, spawn_point, attach_to=vehicle)

    # add sensor to list of actors
    actor_list.append(sensor)

    # do something with this sensor

    blueprint = blueprint_library.find('sensor.camera.semantic_segmentation')
    blueprint.set_attribute('image_size_x', f'{IM_WIDTH}')
    blueprint.set_attribute('image_size_y', f'{IM_HEIGHT}')
    blueprint.set_attribute('fov', '110')
    spawn_point = carla.Transform(carla.Location(x=2.5, z=1.4))
    sensor_s = world.spawn_actor(blueprint, spawn_point, attach_to=vehicle)
    actor_list.append(sensor_s)
    
    blueprint = blueprint_library.find('sensor.camera.instance_segmentation')
    blueprint.set_attribute('image_size_x', f'{IM_WIDTH}')
    blueprint.set_attribute('image_size_y', f'{IM_HEIGHT}')
    blueprint.set_attribute('fov', '110')
    spawn_point = carla.Transform(carla.Location(x=2.5, z=1.4))
    sensor_i = world.spawn_actor(blueprint, spawn_point, attach_to=vehicle)
    actor_list.append(sensor_i)


    image_queue = queue.Queue()
    seg_queue = queue.Queue()
    ins_queue = queue.Queue()

    #sensor.listen(image_queue.put)
    #sensor_s.listen(seg_queue.put)
    #sensor_i.listen(ins_queue.put)

    sensor.listen(lambda image: process_img(image))
    sensor_s.listen(lambda image: process_seg_img(image))
    sensor_i.listen(lambda image: process_ins_img(image))
    

    wps =  [carla.WeatherParameters.ClearNoon,
            carla.WeatherParameters.CloudyNoon,
            carla.WeatherParameters.SoftRainSunset,
            carla.WeatherParameters.WetCloudyNoon,
            carla.WeatherParameters.MidRainSunset,
            carla.WeatherParameters.SoftRainNight,
            carla.WeatherParameters.ClearNoon
            ]

    # Set default weather
    for w in wps:
        print("Change weather")
        client.get_world().set_weather(w)
        time.sleep(120)

    #process_img(image_queue.get())
    #process_seg_img(seg_queue.get())
    #process_ins_img(ins_queue.get())


finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')
