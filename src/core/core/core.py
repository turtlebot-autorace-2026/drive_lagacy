import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class Core(Node):
    def __init__(self):
        super().__init__('core_node')
        
        # Publishers
        self.laneModePublisher = self.create_publisher(Bool, 'lane_mode', 10)
        self.laneStatePublisher = self.create_publisher(Bool, 'lane_state', 10)
        self.trafficLightStatePublisher = self.create_publisher(Bool, 'traffic_light_state', 10)
        self.intersectionStatePublisher = self.create_publisher(Bool, 'intersection_state', 10)
        self.constructionStatePublisher = self.create_publisher(Bool, 'construction_state', 10)
        self.parkingStatePublisher = self.create_publisher(Bool, 'parking_state', 10)
        self.levelCrossingStatePublisher = self.create_publisher(Bool, 'level_crossing_state', 10)
        self.tunnelStatePublisher = self.create_publisher(Bool, 'tunnel_state', 10)
        
        # Subscribers
        #self.laneStateSubscriber = self.create_subscription(
        #    Bool, '/lane_state', self.laneStateSubscriber_callback, 10)
        self.trafficLightStateSubscriber = self.create_subscription(
            Bool, '/traffic_light_state', self.trafficLightStateSubscriber_callback, 10)
        self.intersectionSubscriber = self.create_subscription(
            Bool, '/intersection_state', self.intersectionStateSubscriber_callback, 10)
        self.constructionStateSubscriber = self.create_subscription(
            Bool, '/construction_state', self.constructionStateSubscriber_callback, 10)
        self.parkingStateSubscriber = self.create_subscription(
            Bool, '/parking_state', self.parkingStateSubscriber_callback, 10)
        self.levelCrossingStateSubscriber = self.create_subscription(
            Bool, '/level_crossing_state', self.levelCrossingStateSubscriber_callback, 10)
        self.tunnelStateSubscriber = self.create_subscription(
            Bool, '/tunnel_state', self.tunnelStateSubscriber_callback, 10)
        
        self.timer = self.create_timer(0.1, self.main_loop)
        
        # State Variables
        self.lane_state = False
        self.traffic_light_state = False
        self.intersection_state = False
        self.construction_state = False
        self.parking_state = False
        self.level_crossing_state = False
        self.tunnel_state = False
        
        # Sub Variables
        self.mission_running = False
        self.initial_mission = self.declare_parameter('initial_mission', 'traffic_light').value
        self.mission_todo = self.initial_mission
        self.initial_delay_scale = self.declare_parameter('initial_delay_scale', 1.0).value
        self.stop_before_tunnel = self.declare_parameter('stop_before_tunnel', False).value
        
    #def laneStateSubscriber_callback(self, msg):
    #    self.get_logger().info("lane")
    
    def trafficLightStateSubscriber_callback(self, msg):
        if not msg.data:
            self.get_logger().info("Finished Traffic Light Mission!\n")
            self.mission_running = False
    
    def intersectionStateSubscriber_callback(self, msg):
        if not msg.data:
            self.get_logger().info("Finished Intersection Mission!\n")
            self.mission_running = False
            
    def constructionStateSubscriber_callback(self, msg):
        if not msg.data:
            self.get_logger().info("Finished Construction Mission!\n")
            self.mission_running = False
    
    def parkingStateSubscriber_callback(self, msg):
        if not msg.data:
            self.get_logger().info("Finished Parking Mission!\n")
            self.mission_running = False
    
    def levelCrossingStateSubscriber_callback(self, msg):
        if not msg.data:
            self.get_logger().info("Finished Level Crossing Mission!\n")
            self.mission_running = False
    
    def tunnelStateSubscriber_callback(self, msg):
        if not msg.data:
            self.get_logger().info("Finished Tunnel Mission!\n")
            self.mission_running = False
    
    def create_bool_msg(self, param):
        msg = Bool()
        msg.data = param
        return msg
    
    def schedule(self, seconds, callback):
        def once():
            timer.cancel()
            callback()
        scale = self.initial_delay_scale if self.mission_todo == self.initial_mission else 1.0
        timer = self.create_timer(seconds * scale, once)

    def main_loop(self):
        if self.mission_running: return
        
        #================Traffic Light================#
        if self.mission_todo == 'traffic_light':
            self.get_logger().info("Start Traffic Light Mission")
            
            # mission publisher
            self.trafficLightStatePublisher.publish(self.create_bool_msg(True))
            
            # lane publisher
            self.laneModePublisher.publish(self.create_bool_msg(True))
            self.laneStatePublisher.publish(self.create_bool_msg(False))
            
            # ready next mission
            self.mission_todo = 'intersection'
            self.mission_running = True
        
        #=================Intersection=================#
        elif self.mission_todo == 'intersection':
            self.get_logger().info("Start Intersection Mission")
            
            # mission publisher
            def intersection_thread(): self.intersectionStatePublisher.publish(self.create_bool_msg(True))
            self.schedule(12.0, intersection_thread)
            
            # lane publisher
            self.laneModePublisher.publish(self.create_bool_msg(True))
            self.laneStatePublisher.publish(self.create_bool_msg(True))
            
            # ready next mission
            self.mission_todo = 'construction'
            self.mission_running = True
        
        #=================Construction=================#
        elif self.mission_todo == 'construction':
            self.get_logger().info("Start Construction Mission")
            
            # mission publisher
            def construction_thread(): self.constructionStatePublisher.publish(self.create_bool_msg(True))
            self.schedule(30.0, construction_thread)
            
            # lane publisher
            def lane_mode_thread(): self.laneModePublisher.publish(self.create_bool_msg(False))
            def lane_state_thread(): self.laneStatePublisher.publish(self.create_bool_msg(True))
            self.schedule(30.0, lane_mode_thread)
            self.schedule(30.0, lane_state_thread)
            
            # ready next mission
            self.mission_todo = 'parking'
            self.mission_running = True
        
        #===================Parking====================#
        elif self.mission_todo == 'parking':
            self.get_logger().info("Start Parking Mission")
            
            # mission publisher
            def parking_thread():
                self.laneModePublisher.publish(self.create_bool_msg(True))
                self.parkingStatePublisher.publish(self.create_bool_msg(True))
            self.schedule(3.0, parking_thread)
            
            # lane publisher
            self.laneModePublisher.publish(self.create_bool_msg(False))
            self.laneStatePublisher.publish(self.create_bool_msg(True))
            
            # ready next mission
            self.mission_todo = 'level_crossing'
            self.mission_running = True
        
        #================Level Crossing================#
        elif self.mission_todo == 'level_crossing':
            self.get_logger().info("Start Level Crossing Mission")
            
            # mission publisher
            def level_crossing_thread(): self.levelCrossingStatePublisher.publish(self.create_bool_msg(True))
            self.schedule(29.3, level_crossing_thread)
            
            # lane publisher
            def lane_mode_white_thread(): self.laneModePublisher.publish(self.create_bool_msg(False))
            self.schedule(23.5, lane_mode_white_thread)
            self.laneStatePublisher.publish(self.create_bool_msg(True))
                        
            # ready next mission
            self.mission_todo = 'tunnel'
            self.mission_running = True
        
        #===================Tunnel=====================#
        elif self.mission_todo == 'tunnel':
            self.get_logger().info("Start Tunnel Mission")
            
            # mission publisher
            def tunnel_thread():
                if self.stop_before_tunnel:
                    self.laneStatePublisher.publish(self.create_bool_msg(False))
                    self.get_logger().info('Stopped before tunnel: requested route complete')
                else:
                    self.tunnelStatePublisher.publish(self.create_bool_msg(True))
            self.schedule(34.7, tunnel_thread)
            
            # lane publisher
            def lane_mode_white_thread(): self.laneModePublisher.publish(self.create_bool_msg(False))
            def lane_mode_yellow_thread(): self.laneModePublisher.publish(self.create_bool_msg(True))
            self.schedule(2.9, lane_mode_yellow_thread)
            self.schedule(22.0, lane_mode_white_thread)
            self.laneStatePublisher.publish(self.create_bool_msg(True))
            
            # ready next mission
            self.mission_todo = 'last_lane'
            self.mission_running = True
        
        #===================Finish=====================#
        elif self.mission_todo == 'last_lane':
            self.get_logger().info("Start Last Lane Mission")
            
            # mission & lane publisher
            self.laneModePublisher.publish(self.create_bool_msg(True))
            self.laneStatePublisher.publish(self.create_bool_msg(True))
            def stop_lane_thread(): self.laneStatePublisher.publish(self.create_bool_msg(False))
            self.schedule(10.0, stop_lane_thread)
            
            # ready next mission
            self.mission_todo = 'finish'
            self.mission_running = True
        
        else:
            self.get_logger().info("Finished All Missions!! (/^v^)/")
            self.get_logger().info("By! \(^v^\) (/^v^)/")
            self.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
            
def main(args=None):
    rclpy.init(args=args)
    node = Core()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
