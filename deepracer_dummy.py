import math


class DECORewardCalculator:
    all_wheels_on_track = None
    x = None
    y = None
    distance_from_center = None
    is_left_of_center = None
    is_reversed = None
    heading = None
    progress = None
    steps = None
    speed = None
    steering_angle = None
    track_width = None
    waypoints = None
    closest_waypoints = None
    prev_waypoint_idx = None
    next_waypoint_idx = None
    is_left_of_center = None
    distance_from_border = None
    is_offtrack = None
    SPEED_THRESHOLD = 4
    CAR_DIRECTION_DIFF = 10
    GENTLE_STEERING_UB = 20
    MAX_STEERING_UB = 30

    def __init__(self, params):
        self.track_width = params['track_width']
        self.distance_from_center = params['distance_from_center']
        self.all_wheels_on_track = params['all_wheels_on_track']
        self.speed = params['speed']
        self.waypoints = params['waypoints']
        self.closest_waypoints = params['closest_waypoints']
        self.heading = params['heading']
        self.is_offtrack = params['is_offtrack']

        self.x = params['x']
        self.y = params['y']
        self.steering_angle = abs(params['steering_angle'])
        self.progress = params['progress']
        self.steps = params['steps']
        self.prev_waypoint_idx = self.closest_waypoints[0]
        self.next_waypoint_idx = self.closest_waypoints[1]
        self.is_left_of_center = params['is_left_of_center']
        self.distance_from_border = self.c_distance_from_border()

    def c_rwpnumber(self, ind):
        if ind < 0:
            return len(self.waypoints) + ind
        else:
            return ind if ind < len(self.waypoints) else ind - len(self.waypoints)

    def c_distance_from_border(self):
        return 0.5 * self.track_width - self.distance_from_center

    def c_distance_between_p(self, p1, p2):
        return math.hypot(p2[1] - p1[1], p2[0] - p1[0])

    def c_angle_between_p(self, p1, p2):
        return math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))

    def c_angle_diff(self, angle1, angle2):
        result = angle2 - angle1
        if angle2 < -90 and angle1 > 90:
            return 360 + result
        elif result > 180:
            return -180 + (result - 180)
        elif result < -180:
            return 180 - (result + 180)
        else:
            return result

    def c_car_heading_difference(self, track_angle, car_heading):
        if track_angle >= 0 and car_heading >= 0:
            return abs(track_angle - car_heading)
        elif track_angle <= 0 <= car_heading:
            return abs(track_angle + car_heading)
        elif track_angle >= 0 >= car_heading:
            return abs(track_angle + car_heading)
        elif track_angle <= 0 and car_heading <= 0:
            return abs(track_angle - car_heading)

    def c_r_distance_from_border(self, reward):
        if self.distance_from_border >= 0.4:
            return reward + 4 * 50
        elif self.distance_from_border >= 0.2:
            return reward + 2 * 25
        elif self.distance_from_border >= 0.1:
            return reward + 1 * 10
        elif self.is_offtrack:
            return 0
        else:
            return 0.1

    def evaluate_rp(self, params):
        self.__init__(params)
        reward = 1
        # Calculate 3 markers that are at varying distances away from the center line
        # track_offset_penalty = math.pow(distance_from_center / (track_width / 2.5), 0.2)
        # Calculate 3 markers that are at varying distances away from the center line
        # marker_1 = 0.1 * track_width
        # marker_2 = 0.25 * track_width
        # marker_2 = 0.20 * track_width
        # marker_3 = 0.5 * track_width
        # marker_4 = 0.65 * track_width

        reward = self.c_r_distance_from_border(reward)
        if reward <= 0.1:
            return 0

        past_way_point_index = self.prev_waypoint_idx

        dist_between_wp = self.c_distance_between_p(self.waypoints[self.prev_waypoint_idx],
                                                    self.waypoints[self.next_waypoint_idx])
        angle_between_wp = self.c_angle_between_p(self.waypoints[self.prev_waypoint_idx],
                                                  self.waypoints[self.next_waypoint_idx])

        past_way_point_index = self.c_rwpnumber(past_way_point_index - 1)
        past_angle_between_wp = self.c_angle_between_p(self.waypoints[past_way_point_index],
                                                       self.waypoints[self.prev_waypoint_idx])

        fut_way_point_index = self.next_waypoint_idx
        fut_angle_between_wp = angle_between_wp
        prediction = False
        loop_wp_index = self.prev_waypoint_idx
        dist_between_wp_fut = dist_between_wp
        wp_counter = 0
        bs = "LIGHT"
        while wp_counter <= 5:
            fut_way_point_index = fut_way_point_index + 1
            fut_way_point_index = self.c_rwpnumber(fut_way_point_index)
            fut_angle_between_wp = self.c_angle_between_p(self.waypoints[self.prev_waypoint_idx],
                                                          self.waypoints[fut_way_point_index])
            dist_between_wp_fut = self.c_distance_between_p(self.waypoints[self.prev_waypoint_idx],
                                                            self.waypoints[fut_way_point_index])
            fut_angle_between_wp = abs(self.c_angle_diff(past_angle_between_wp, fut_angle_between_wp))
            if 90 < fut_angle_between_wp < 180:
                fut_angle_between_wp = abs(180 - fut_angle_between_wp)
            loop_wp_index = loop_wp_index + 1
            prediction = True
            if fut_angle_between_wp > 10:
                break
            wp_counter = wp_counter + 1


        if 3<wp_counter<=4:
            bs = "MODERATE"

        if 0<=wp_counter<=3:
            bs = "HARD"

        track_dir_ang = angle_between_wp

        angle_ahead_wp_diff = abs(self.c_angle_diff(past_angle_between_wp, angle_between_wp))
        angle_fut_wp_diff = self.c_angle_diff(past_angle_between_wp,fut_angle_between_wp)
        car_heading_diff = self.c_car_heading_difference(angle_between_wp, self.heading)

        fut_way_point_index = self.next_waypoint_idx
        fut_angle_between_wp = angle_ahead_wp_diff
        prediction = False
        loop_wp_index = self.prev_waypoint_idx
        dist_between_wp_fut = dist_between_wp

        a = False
        b = False
        c = False
        d = False
        e = False
        f = False
        g = False
        h = False
        i = False
        j = False
        k = False
        l = False
        m = False
        n = False
        o = False
        p = False

        # steering angle
        #if 0 <= self.steering_angle <= 2:
        #    reward = reward + 2 * 50
        #    a = True
        #elif 2 < self.steering_angle <= self.GENTLE_STEERING_UB:
        #    reward = reward + 1 * 5
        #    b = True
        #elif self.steering_angle > self.GENTLE_STEERING_UB:
        #    reward = reward * 0.75
        #    c = True

        # car heading diff
        if 0 < car_heading_diff < self.CAR_DIRECTION_DIFF:
            reward = reward + ((10 - car_heading_diff) * 50)
        elif self.CAR_DIRECTION_DIFF < car_heading_diff < 2*self.CAR_DIRECTION_DIFF:
            reward = reward + 1 * abs(car_heading_diff - 10) / 10
            e = True
        else:
            reward = reward * 0.5
            f = True

        if reward <= 0.1:
            return 0

        # speed
        if 0 <= abs(angle_ahead_wp_diff) <= 10:
            if 0 <= self.steering_angle <= 2:
                reward = reward + (2 * 50)
                a = True
            elif 2 < self.steering_angle <= self.GENTLE_STEERING_UB/3:
                reward = reward + 1 * 5
                b = True
            elif self.steering_angle > self.GENTLE_STEERING_UB/3:
                reward = reward * 0.75
                c = True
                return  reward
            if bs == "LIGHT":
                if 3 <= self.speed <= 4:
                    reward = reward + (10 * 50)
                elif self.speed <= 0.75 * self.SPEED_THRESHOLD:
                    reward = 0.1
            elif bs == "MODERATE":
                if 2 <= self.speed <= 3:
                    reward = reward + (10 * 50)
                elif self.speed <= 0.75 * self.SPEED_THRESHOLD:
                    reward = 0.1
            elif bs == "HARD":
                if 1.5 <= self.speed <= 2:
                    reward = reward + (10 * 50)
                elif self.speed <= 0.75 * self.SPEED_THRESHOLD:
                    reward = 0.1

        if reward <= 0.2:
            return 0

        if 10 < abs(angle_ahead_wp_diff) < 15:
            if 0 <= self.steering_angle <= 1:
                reward = reward + 1 * 5
                a = True
            elif 1 < self.steering_angle <= self.GENTLE_STEERING_UB:
                reward = reward + 2 * 50
                b = True
            elif self.steering_angle > self.GENTLE_STEERING_UB:
                reward = reward * 0.75
                c = True
            if 2.5 <= self.speed <= 3:
                reward = reward + (10 * 50)
                j = True
            else:
                reward = 0.1
                k = True

        if reward <= 0.2:
            return 0

        if abs(angle_ahead_wp_diff) > 15:
            if 0 <= self.steering_angle <= 1:
                reward = reward + (2 * 1)
                a = True
            else:
                reward = reward + (2*50)
                c = True
            if self.speed <= 2:
                reward = reward + (10 * 50)
                m = True
            #elif 0.5 * self.SPEED_THRESHOLD < self.speed < 0.67 * self.speed:
            #    reward = reward + (10*5)
            else:
                reward = 0.1


        if reward <= 0.2:
            return 0

        print("KR|" + str(self.prev_waypoint_idx)+"|"+str(self.next_waypoint_idx) + "|" + str(self.speed) + "|" + str(angle_ahead_wp_diff) + "|" + str(
               track_dir_ang) + "|" + str(car_heading_diff) + "|" + str(angle_between_wp)
                 + "|" + str(past_angle_between_wp) + "|" + str(reward) +"|"+str(self.heading)+ "|" + str(self.steering_angle) + "|"+str(fut_angle_between_wp)+"|"
              +str(dist_between_wp_fut)+"|"+str(fut_way_point_index)
              )

        TOTAL_NUM_STEPS = 120
        # Give additional reward if the car pass every n steps faster than expected

        if self.progress == 100:
            reward = reward + self.progress * 30

        if self.steps >120:
            reward = 0

        # elif self.progress % 66 == 0:
        #     if self.progress > (self.steps / ((TOTAL_NUM_STEPS*2)/3)) * 100:
        #         reward = reward + (((TOTAL_NUM_STEPS*2)/3) - self.steps)*100
        #     else:
        #         reward = 0.2
        # elif self.progress % 33 == 0:
        #     if self.progress > (self.steps / ((TOTAL_NUM_STEPS*1)/3)) * 100:
        #         reward = reward + (((TOTAL_NUM_STEPS*1)/3) - self.steps)*100
        #     else:
        #         reward = 0.2

        if reward <= 0.2:
            return 0

        return reward


def reward_function(params):
    re = DECORewardCalculator(params)
    return float(re.evaluate_rp(params))
