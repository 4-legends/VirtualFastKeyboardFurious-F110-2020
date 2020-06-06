import math
import random
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv
from PIL import Image , ImageFilter
import cv2
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.path_x = []
        self.path_y = []
        self.parent = None

class RRT:
    def __init__(self, expand_dis=5.0, path_resolution=0.5, goal_sample_rate=5, max_iter=15000):
        
        self.read_map()
        self.map_copy()
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.node_list = []

    def read_map(self):
        im = Image.open('../../f110-skeletons-spring2020/race_monitor/maps/race_track_f110.pgm')
        im = im.filter(ImageFilter.MinFilter(7))

        width, height = im.size 
        im = np.array(im.getdata(), dtype=np.float64)
        im = im.reshape((height, width))
        im = np.rot90(im, 3) 
        im[im<255] = 0
        self.im_copy = im.copy()

        self.map = im
        self.x_bounds, self.y_bounds  = width, height 
    
    def map_copy(self):
        im = Image.open('../../f110-skeletons-spring2020/race_monitor/maps/race_track_f110.pgm')
        width, height = im.size 
        im = np.array(im.getdata(), dtype=np.float64)
        im = im.reshape((height, width))
        im = np.rot90(im, 3) 
        im[im<255] = 0
        self.map_copy = im.copy()

    def convert_from_map_pose_to_grid(self, map_point):  
        (map_x, map_y) = map_point
        grid_x = int((map_x + 34.03)/0.05)
        grid_y = int((map_y + 35.33)/0.05)
        return (grid_x, grid_y)

    def convert_from_grid_to_map_pose(self, grid_point):  
        (grid_x, grid_y) = grid_point
        map_x = grid_x*0.05 - 34.03
        map_y = grid_y*0.05 - 35.33 
        return (map_x, map_y)        
        
    def init_planner(self,start, goal):
        
        start = self.convert_from_map_pose_to_grid(start)
        goal = self.convert_from_map_pose_to_grid(goal)

        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.node_list = [self.start]
    
    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]

                
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = Node(int(random.random()*self.x_bounds), int(random.random()*self.y_bounds))
        else:  # goal point sampling
            rnd = Node(self.end.x, self.end.y)
        return rnd

    def check_collision(self, node):

        if node is None:
            return False
        if int(self.map[int(round(node.x)), int(round(node.y))]) == 0:
            return False
        return True  # safe

    def start_planning(self, start, goal, animation=True):

        self.init_planner(start, goal)

        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)
            if self.check_collision(new_node):
                self.node_list.append(new_node)
            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.check_collision(final_node):
                    print ('Path found')
                    return self.generate_final_course(len(self.node_list) - 1)
        return None

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = int(math.floor(extend_length / self.path_resolution))
        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)

        new_node.parent = from_node

        return new_node

    def plot_image(self, checkPoint_idx, waypoints ):
        self.map_copy = cv2.circle(self.map_copy, (self.start.y, self.start.x), 2, (0,0,0),2 )
        self.map_copy = cv2.circle(self.map_copy, (self.end.y, self.end.x), 2, (0,0,0), 2)
        for point in waypoints:
            x = int(round(point[1]))
            y = int(round(point[0]))
            self.map_copy = cv2.circle(self.map_copy, (x,y), 1, (0,0,0),1 )
        cv2.imwrite('../imgs/map_with_rrt_planned_waypoints_{}.png'.format(checkPoint_idx), self.map_copy)
        print ('Saved path planned in .../imgs/ for Checkpoint', checkPoint_idx)

        plt.imshow(self.map_copy)
        plt.show()

    def save_all_map_waypoints(self, map_way_points ):
        csv_file = open('../waypoints/rrt_waypoints.csv', 'w')
        for wp in map_way_points:
            x,y, yaw = wp
            csv_file.write('{}, {}, {}\n'.format(x,y, yaw))
        print ('Saved RRT path in ../waypoints/')

    def get_map_waypoints(self, grid_waypoints):
        map_waypoints = []
        grid_waypoints.reverse()
        for i in range(len(grid_waypoints)):
            if i == 0:
                curr_wp = self.convert_from_grid_to_map_pose(grid_waypoints[i])
                yaw = 0.0
            else:
                curr_wp = self.convert_from_grid_to_map_pose(grid_waypoints[i])
                yaw = self.calculate_yaw(prev_wp, curr_wp)
            map_waypoints.append((curr_wp[0], curr_wp[1], yaw))
            prev_wp = curr_wp
        return map_waypoints


    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y)
                 ** 2 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

    @staticmethod
    def calculate_yaw(point1, point2):
        return math.atan2(point2[1]-point1[1],point2[0]-point1[0])



def main():
    rrt  = RRT()
    start = [0.0, 0.0]    # origin

    # Checkpoint 1
    checkpoint1 = [13.47, -10.33] 
    checkpoint2 = [1.395, -11.8700]    
    checkpoint3 = [-18.815, -28.605]
    checkpoint4 = [-24.02, -21.135]
    checkpoint5 = [-16.82, -4.93]


    waypoints_C1 = rrt.start_planning(start, checkpoint1)
    map_waypoints_C1 = rrt.get_map_waypoints(waypoints_C1)
    rrt.plot_image(1, waypoints_C1)


    waypoints_C2 = rrt.start_planning(checkpoint1, checkpoint2)
    map_waypoints_C2 = rrt.get_map_waypoints(waypoints_C2)
    rrt.plot_image(2, waypoints_C2)


    waypoints_C3 = rrt.start_planning(checkpoint2, checkpoint3)
    map_waypoints_C3 = rrt.get_map_waypoints(waypoints_C3)
    rrt.plot_image(3, waypoints_C3)


    waypoints_C4 = rrt.start_planning(checkpoint3, checkpoint4)
    map_waypoints_C4 = rrt.get_map_waypoints(waypoints_C4)
    rrt.plot_image(4, waypoints_C4)


    waypoints_C5 = rrt.start_planning(checkpoint4, checkpoint5)
    map_waypoints_C5 = rrt.get_map_waypoints(waypoints_C5)
    rrt.plot_image(5, waypoints_C5)


    waypoints_C6 = rrt.start_planning(checkpoint5, start)
    map_waypoints_C6 = rrt.get_map_waypoints(waypoints_C6)
    rrt.plot_image(6, waypoints_C6)

    ##storing all waypoints in CSV
    rrt_map_waypoints = map_waypoints_C1 + map_waypoints_C2 + map_waypoints_C3 + map_waypoints_C4 + map_waypoints_C5 + map_waypoints_C6
    rrt.save_all_map_waypoints(rrt_map_waypoints)

if __name__ == '__main__':
    main()
