import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
import time

class TurtleController(Node):
    def __init__(self, goalX):
        super().__init__('turtle_controller')
        self.linear_vel = 0
        self.u = 0
        self.error_prev = 0
        self.KP = 200.0
        self.KD = 300.0
        
        # DO NOT TOUCH THE CODE BELOW
        self.init(goalX)

    # Create a PD controller to update self.u
    # Calculate the derivative term using self.error_prev and the most recent error
    # This function must: 
    #   call self.setLinearVelocity()
    #   call self.getError()
    #   define derivative variable 'e.g. dedet = ... '
    #   assign control value to self.u using the formula from our PID lecture
    #   store previous error to self.error_prev
    
    def pd_controller(self, robotX, goalX, KP, KD, error_prev):
        e = self.getError1(goalX, robotX)
        # dedt calculation
        dedt = (e-error_prev)
        u = KP * e + KD * dedt
        error_prev = e
        return u
    
    def mainLoop(self):
        robotX = self.pose.x
        goalX = self.goalX
        KP = self.KP
        KD =self.KD
        error_prev = self.error_prev
        u = self.pd_controller(robotX, goalX, KP, KD, error_prev)
        self.setLinearVelocity(u)
        print(self.pose.x)
        self.isDone() # DO NOT REMOVE THIS LINE 

    # get the error using goalX and robotX
    #def getError(self, goalX, robotX):
    #    pass 
    
    def getError1(self, goalX, robotX):
        return goalX-robotX
        
    # publish the linear velocity using the publisher (u)
    def setLinearVelocity(self, u):
        pass
        
    def setLinearVelocity1(self, u):
        self.vel_msg.linear.x = u
        self.publisher.publish(self.vel_msg)
        #self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) 


    #############################################
    ##### Do not modify the functions below #####
    #############################################
    def init(self, goalX):
        self.vel_msg = Twist()
        self.goalX = goalX
        self.errorThreshold = 0.005 
        timer_period = 0.25  
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) 
        self.timer = self.create_timer(timer_period, self.mainLoop) 
        self.turtle_teleport = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        while not self.turtle_teleport.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TeleportAbsolute.Request()
        self.addNoise(0.0, 5.0, 0.0, 0.0)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 1)
        self.pose = Pose()
        self.rate = self.create_rate(10)
        self.startTime = time.time()

    def addNoise(self, x, y, theta, mag):
        self.req.x = x + self.u
        self.req.y = y
        self.req.theta = theta
        self.turtle_teleport.call_async(self.req)

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.addNoise(self.pose.x, 5.0, self.pose.theta, self.linear_vel)

    def stop(self):
        self.destroy_node()
        rclpy.shutdown()

    def isDone(self):
        if (abs(self.goalX - self.pose.x) < self.errorThreshold):   
            elapsedTime = time.time() - self.startTime
            print("Program Time (s)", elapsedTime)
            self.stop()

def main(args=None):
    rclpy.init(args=args)

    turtle_controller = TurtleController(5.0)

    rclpy.spin(turtle_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()