#!/usr/bin/env python3
import rospy
import rospkg
import tf
from geometry_msgs.msg import Point, PointStamped, PoseStamped, TwistStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty, EmptyResponse
from custom_sim.msg import Obstacle, ObstacleArray
from custom_sim.srv import CheckCollision, CheckCollisionResponse  # Import the service message
import numpy as np
import yaml  # Import yaml to read the file
from nav_msgs.msg import Odometry

class MovingObstacle:
    def __init__(self, id, initial_pose=[0.0,0.0], size=[0.0,0.0], vel=[0.0, 0.0], bounds=[-10,10]):
        
        self.initial_pose = Point()
        self.initial_pose.x = initial_pose[0]
        self.initial_pose.y = initial_pose[1]
        self.initial_pose.z = 0.0

        self.length = size[0]
        self.width = size[1]
        
        self.velocity = Point()
        self.velocity.x = vel[0]
        self.velocity.y = vel[1]
        self.velocity.z = 0.0
        self.id = id

        self.current_pose = Point()
        self.current_pose = self.initial_pose
        
        self.bounds = Point()
        self.bounds.x = bounds[0]
        self.bounds.y = bounds[1]
        self.bounds.z = 0.0
        self.freq = 10.0
        
        self.dt = 1.0/self.freq

        if(np.any(vel)):
            self.timer = rospy.Timer(rospy.Duration(self.dt), self.move_obs)
            print(f"Created Moving Obstacle:\n ID={self.id},\n Initial Pose=({self.initial_pose.x}, {self.initial_pose.y}),\n Velocity=({self.velocity.x}, {self.velocity.y}),\n Bounds=({self.bounds.x}, {self.bounds.y})")
            
    
    def inside_bounds(self, value, min_bound, max_bound):
        return ((min_bound <= value)  and (value <= max_bound))
    
    def is_collision(self, pose):
        min_x = self.current_pose.x
        max_x = self.current_pose.x + self.length
        min_y = self.current_pose.y
        max_y = self.current_pose.y + self.width

        if ((pose.x >= min_x) and (pose.x <= max_x) and (pose.y >= min_y) and (pose.y <= max_y)):
            return True
        
        return False
    
    def move_obs(self, event):
        
        vx = self.velocity.x
        vy = self.velocity.y
        
        self.current_pose.x = self.current_pose.x + vx * self.dt
        self.current_pose.y = self.current_pose.y + vy * self.dt
        # self.current_pose.z = 0.0  # Keep z constant at 0
        
        #if robot is outof bounds invert velocity
        if(not self.inside_bounds(self.current_pose.x, min_bound=self.bounds.x, max_bound=self.bounds.y)):
            if(self.current_pose.x>=self.bounds.y) and (self.velocity.x>0.0):
                self.velocity.x = -1.0*self.velocity.x
            elif(self.current_pose.x<=self.bounds.x) and (self.velocity.x<0.0):
                self.velocity.x = -1.0*self.velocity.x
            
            
        if(not self.inside_bounds(self.current_pose.y, min_bound=self.bounds.x, max_bound=self.bounds.y)):
            if(self.current_pose.y>=self.bounds.y) and (self.velocity.y>0.0):
                self.velocity.y = -1.0*self.velocity.y
            elif(self.current_pose.y<=self.bounds.x) and (self.velocity.y<0.0):
                self.velocity.y = -1.0*self.velocity.y

    def getCurrentPose(self):
        return self.current_pose

class MarkerMover:
    def __init__(self):
        # Initialize variables
        self.target_pose = Point()
        self.current_pose = PoseStamped()
        self.current_vel = TwistStamped()
        self.obstacle_list = ObstacleArray()
        
        self.moving_obs = {}
        self.bounds = [-50.0,50.0]

  
        self.is_moving = False
        self.sensor_radius = 20.0
        
        self.frequency = 20.0  # Hz
        self.dt = 1.0/self.frequency  # Timer period in seconds
        self.default_obs_hieght = 10.0
        
        rospack = rospkg.RosPack()
        package_name = 'custom_sim'  # Replace with your package name
        package_path = rospack.get_path(package_name)
        
        # Construct the full path to the YAML file
        self.param_path = f"{package_path}/param/obstacle_list.yaml"
        
        rospy.loginfo("Param Path: %s", self.param_path)

        self.init_obstacles(self.param_path)

        
        
        # Create marker publisher
        self.marker_pub = rospy.Publisher('robot_and_sensor', MarkerArray, queue_size=10)
        self.obs_marker_pub = rospy.Publisher('obstacles_marker', MarkerArray, queue_size=10)
        self.obs_pub = rospy.Publisher('obstacles', ObstacleArray, queue_size=10)
        self.marker_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=10)
        self.current_vel_pub = rospy.Publisher('current_velocity',TwistStamped,queue_size=10)
        
        
        # Create subscriber instead of service
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        
        # Set up the marker
        self.marker = Marker()
        self.marker.header.frame_id = "map"  # Change to your frame
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        
        # Start the movement timer using frequency
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.move_marker)
        rospy.loginfo("Simulation Setup Done! Waiting for target pose")

        # Create the service server
        # self.collision_service = rospy.Service('check_collision', CheckCollision, self.handle_collision_check)

    def init_obstacles(self, path):
        with open(path, 'r') as file:
            obstacle_data = yaml.safe_load(file)  # Load the YAML file
            
            # Clear the existing obstacle list
            self.obstacle_list.obstacles.clear()
            
            # Populate the obstacle_list with new data
            for obs in obstacle_data['obstacles']:
                obstacle = Obstacle()
                obstacle.id = obs['id']
                obstacle.position.x = obs['position'][0]#+(obs['size'][0]/2.0)  # Accessing x position
                obstacle.position.y = obs['position'][1]#+(obs['size'][1]/2.0)  # Accessing y position
                obstacle.position.z = 0.0
                obstacle.size.x = obs['size'][0]#/2.0
                obstacle.size.y = obs['size'][1]#/2.0
                obstacle.size.z  = self.default_obs_hieght

                self.obstacle_list.obstacles.append(obstacle)
                
                vel = [0.0,0.0]
                bounds = self.bounds

                if('velocity' in obs):
                    vel  = obs['velocity']
                    # Create MovingObstacle instance
                if( 'bounds' in obs):
                    bounds = obs['bounds']

                mov_obs = MovingObstacle(id=obs['id'], initial_pose=obs['position'], size=obs['size'], vel=vel,bounds=bounds)
                self.moving_obs[obs['id']] = mov_obs

        
        rospy.loginfo("Obstacles loaded: %d", len(self.obstacle_list.obstacles))
    def get_robot_and_sensor(self, robot_pose, sensor_radius):
        marker_array = MarkerArray()
        robot_marker = Marker()
        sensor_marker = Marker()
        robot_marker.header.frame_id = "map"  # Change to your frame
        robot_marker.header.stamp = rospy.Time.now()
        robot_marker.type = Marker.SPHERE
        robot_marker.action = Marker.ADD
        robot_marker.scale.x = 0.5
        robot_marker.scale.y = 0.5
        robot_marker.scale.z = 0.5
        robot_marker.color.r = 1.0
        robot_marker.color.g = 0.0
        robot_marker.color.b = 0.0
        robot_marker.color.a = 1.0        
        robot_marker.pose.position = robot_pose

        sensor_marker = robot_marker
        sensor_marker.type = Marker.CYLINDER
        robot_marker.scale.x = sensor_radius
        robot_marker.scale.y = sensor_radius
        robot_marker.scale.z = 0.5
        robot_marker.color.r = 0.0
        robot_marker.color.g = 1.0
        robot_marker.color.a = 0.1
        
        marker_array.markers.append(robot_marker)
        marker_array.markers.append(sensor_marker)

        return marker_array

    def get_obstacle_markers(self):
        """Create a MarkerArray for the obstacles in the obstacle_list."""
        marker_array = MarkerArray()  # Initialize a MarkerArray
        obsMsg = ObstacleArray()
        obsMsg.header.frame_id = "map"
        obsMsg.header.stamp = rospy.Time.now()

        for obstacle in self.obstacle_list.obstacles:
            marker = Marker()
            marker.header.frame_id = "map"  # Set the frame ID
            marker.header.stamp = rospy.Time.now()  # Set the current time
            marker.ns = "obstacles"  # Set a namespace for the markers
            marker.id = obstacle.id  # Use the obstacle ID as the marker ID
            marker.type = Marker.CUBE  # Use a cube to represent the obstacle
            marker.action = Marker.ADD  # Action to add the marker
            #update current obs position
            obstacle.position = self.moving_obs[obstacle.id].getCurrentPose()
            # marker.pose.position = obstacle.position #moving_obs[obstacle.id].getCurrentPose()
            marker.pose.position.x = obstacle.position.x + (obstacle.size.x/2.0)
            marker.pose.position.y = obstacle.position.y + (obstacle.size.y/2.0)
            marker.scale.x = obstacle.size.x  # Set the length of the obstacle
            marker.scale.y = obstacle.size.y   # Set the width of the obstacle
            marker.scale.z = 0.1  # Set a small height for visualization
            marker.color.r = 1.0  # Set the color to red
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5  # Set transparency

            marker_array.markers.append(marker)  # Add the marker to the array
            obstacle.header = marker.header
            obsMsg.obstacles.append(obstacle)

        
        return marker_array, obsMsg 
    
    def odom_callback(self, msg):
        """Subscriber callback to update target position"""
        self.current_pose.header = msg.header
        self.current_pose.header.frame_id = 'map'
        self.current_pose.pose = msg.pose.pose

        self.current_vel.header = self.current_pose.header
        self.current_vel.twist = msg.twist.twist
        self.is_moving = True
    def move_marker(self, event):
        if not self.is_moving:
            return
        
        # Update marker position
        self.marker.header= self.current_pose.header
        self.marker.pose = self.current_pose.pose
        combined_array = self.get_robot_and_sensor(self.current_pose.pose.position,self.sensor_radius)
        
        # print(" Current POseX: ", self.current_pose.x)
        # print(" Current POseY: ", self.current_pose.y)
        # print(" Current POse Theta: ", current_yaw)
        
        obsAray, obsMsg = self.get_obstacle_markers()

        self.marker_pub.publish(combined_array)#(self.marker)
        self.marker_pose_pub.publish(self.current_pose)
        self.current_vel_pub.publish(self.current_vel)
        self.obs_marker_pub.publish(obsAray)
        self.obs_pub.publish(obsMsg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('point_robo_sim')
    try:
        mover = MarkerMover()
        mover.run()
    except rospy.ROSInterruptException:
        pass

