import math


def reward_function(params):
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    all_wheels_on_track = params['all_wheels_on_track']
    speed = params['speed']
    SPEED_THRESHOLD = 1
    steering = abs(params['steering_angle'])
    progress = params['progress']
    ABS_STEERING_THRESHOLD = 15

    marker_1 = 0.1 * track_width
    # marker_2 = 0.25 * track_width
    marker_2 = 0.20 * track_width
    marker_3 = 0.5 * track_width
    marker_4 = 0.65 * track_width

    reward = 1
    distance_from_border = 0.5 * track_width - distance_from_center

    # Reward higher if the car stays inside the track borders
    if distance_from_border >= 0.05:
        reward *= 1.0
    else:
        reward = 0.01

    # Give higher reward if the car is closer to center line and vice versa
    # reward = 1 - track_offset_penalty
    # if reward < 0:
    #    reward = 0

    if not all_wheels_on_track:
        # Penalize if the car goes off track
        reward = 0
    elif speed < SPEED_THRESHOLD:
        # Penalize if the car goes too slow
        reward = reward * speed
    #elif steering <= ABS_STEERING_THRESHOLD / 2 and steering > 0:
    #    reward = reward * 0.75
    #elif steering > ABS_STEERING_THRESHOLD:
    #    reward = reward * 0.5
    else:
        # High reward if the car stays on track and goes fast
        reward = reward + 1.0

    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    x = params['x']
    y = params['y']

    # handle 59,60,1,2
    targetwaypointidx = waypoints.index(waypoints[closest_waypoints[1]])
    look_ahead_offset = 0
    look_ahead_index = targetwaypointidx + look_ahead_offset
    if look_ahead_index > len(waypoints)-1:
        look_ahead_index = look_ahead_index - len(waypoints)-1 + look_ahead_offset
    # targetwaypoint = waypoints[look_ahead_index]

    # Calculate the direction of the center line based on the closest waypoints
    iceberg = [0, 0]
    jack = [x, y]
    kate = [0, 0]

    # How far is Kate from jack ?
    kate = waypoints[look_ahead_index]
    dist_j_k = math.hypot(jack[0] - kate[0], jack[1] - kate[1])

    # (x1-x)^2+(y1-y)^2
    iceberg = [x + dist_j_k * math.cos(heading), y + dist_j_k * math.sin(heading)]

    # Kate and Iceberg should converge
    dist_i_k = math.hypot(kate[0] - iceberg[0], kate[1] - iceberg[1])

    if 0 <= dist_i_k <= (dist_j_k * 0.1):
        reward = reward + 1
    else:
        reward = reward * 0.5

    if reward < 0:
        reward = 0


    TOTAL_NUM_STEPS = 350
    steps = params['steps']
    # Give additional reward if the car pass every 100 steps faster than expected
    if (steps % 100) == 0 and progress > (steps / TOTAL_NUM_STEPS) * 100:
        reward += 10.0

    if steps > TOTAL_NUM_STEPS:
        reward = 0

    if progress == 100:
        reward += 100

    return float(reward)