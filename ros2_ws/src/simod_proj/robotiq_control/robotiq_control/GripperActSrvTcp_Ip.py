import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from message.action import CommandRobotiqGripper
from robotiq_control.GripperCommon import RobotiqGripperType
from robotiq_control.GripperSocket import GripperSocket
import time
import rclpy.clock
from rclpy.impl.rcutils_logger import RcutilsLogger
import rclpy.waitable 
import argparse
from rclpy.executors import MultiThreadedExecutor
GOAL_DETECTION_THRESHOLD = 0.001 

class GripperActSrvTcp_Ip(Node, GripperSocket, ActionServer):

    def __init__(self, act_srv_name, robot_ip="127.0.0.1", port=63352, gripper_type = RobotiqGripperType.Hand_E):
        ur_type = act_srv_name.split('/')[0]
        super().__init__('robotiq_action')
        self.robot_ip = robot_ip
        self.port = port
        self._action_name = act_srv_name
        self.logger = RcutilsLogger()


        GripperSocket.__init__(self, robot_ip, port, gripper_type)
        ActionServer.__init__(self,self ,action_type=CommandRobotiqGripper, action_name=self._action_name, execute_callback=self.__execute_callBack)

       
        self.clock = rclpy.clock.Clock()
        whatchdog_connection = rclpy.timer.Timer(self.__connection_timeout,self, timer_period_ns=10, clock=self.clock)
        init_done = False
        while rclpy.ok() and not init_done:
            self.logger.warn(": Waiting for gripper to be ready...")
            init_done = super().initialize()
            time.sleep(0.5)

        
        whatchdog_connection.cancel()
        if super().isReady():
            # ActionServer.is_ready(self, rclpy.wait)
            self.logger.info("Action server {} is Active".format(self._action_name))
        else:
            self.logger.info("Gripper Is Connected but Can't Activate ")
        
        self._feedback = self.__buildFdbkMsg()


    def __connection_timeout(self, event):
        self.logger.fatal("Gripper on ip: {} seems not to respond".format(self.ip))
        rclpy.shutdown("Gripper on port {} seems not to respond".format(self.ip))

    def __buildFdbkMsg(self):
        status = CommandRobotiqGripper.Feedback()
        status.header.stamp         = self.get_clock().now().to_msg()
        status.is_ready             = super().isReady()
        status.is_reset             = super().isSleeping()
        status.is_moving            = False #can't obtain directly
        status.obj_detected         = super().graspDetected()
        status.fault_status         = super().getFaultId()
        status.position             = super().getActualPos()
        status.requested_position   = super().getRequestedPosition()
        status.current              = float(super().getCurrent())
        return status

    def __movement_timeout(self, event):
        self.logger.error("%s: Achieving goal is taking too long, dropping current goal")

    def __abortingActionServer(self, abort_error):
        self.logger.error("%s: Dropping current goal -> " + abort_error )
        ActionServer.set_aborted(self, self.__feedback , (self._action_name))

    def __execute_callBack(self, goal):
        self._processing_goal = False

        self.logger.info( (": New goal received Pos:%.3f Speed: %.3f Force: %.3f Force-Stop: %r") % (goal.request.position, goal.request.speed, goal.request.force, goal.request.stop) )
    
        success = False
        # rate = self.create_timer(1)

        if not goal.request.stop:
            cmd_sent = self.moveToPos( goal.request.position, goal.request.speed, goal.request.force)

        if not cmd_sent:
            self.__abortingActionServer("Unable to Send Tcp Command") 
        else:
            self._processing_goal = True 
        
        watchdog_move = rclpy.timer.Timer(self.__movement_timeout, self, timer_period_ns=10, clock=self.clock)
        
        while rclpy.ok() and self._processing_goal:
            self.__feedback = self.__buildFdbkMsg()
            self.logger.debug("Error = %.5f Requested position = %.3f Current position = %.3f" % (self.__PosError(), self.__feedback.requested_position, self.__feedback.position))
            
            if self.__feedback.fault_status != 0:
                self.__abortingActionServer("Fault status (FLT) is: %d" % self.__feedback.fault_status)
                self._processing_goal = False
                break
            if( self.__PosError() < GOAL_DETECTION_THRESHOLD or self.__feedback.obj_detected):
                self._processing_goal = False
                success = True
                break

            goal.publish_feedback(self.__feedback)

            time.sleep(1)

        watchdog_move.cancel()
        if success:
            self.logger.info(self._action_name + ": Goal reached or object detected Pos: %.3f PosRequested: %.3f ObjectDetected: %r" % (goal.request.position, self.__feedback.requested_position, self.__feedback.obj_detected) )
            goal.succeed()

        result = CommandRobotiqGripper.Result()
        result.is_ready = self.__feedback.is_ready
        result.is_reset = self.__feedback.is_reset
        result.is_moving = self.__feedback.is_moving
        result.obj_detected = self.__feedback.obj_detected
        result.fault_status = self.__feedback.fault_status
        result.position = self.__feedback.position
        result.requested_position = self.__feedback.requested_position
        result.current = self.__feedback.current
        return result

        
    def __PosError(self):
        return abs(self.__feedback.requested_position - self.__feedback.position)



def main(args=None):
    rclpy.init(args=args)

    # args_without_ros = rclpy.utilities.remove_ros_args(args)

    # print('start arg parse')
    # parser = argparse.ArgumentParser(
    #     description='Spawn gripper ip')
    # parser.add_argument('-gripper_ip'),
    # parser.add_argument('-topic_name'),
    
    # print('parse argument....')
    # args = parser.parse_args(args_without_ros[1:])

    # print('Parsed arguments')
    # print(args)


    # ip = args.gripper_ip
    # topic_name = args.topic_name
    ip_right = "192.168.0.103"
    ip_left = "192.168.0.102"
    topic_name_right = "ur_right/robotiq_hand_e"
    topic_name_left = "ur_left/robotiq_hand_e"
    ex = MultiThreadedExecutor()
 
    gripper_HandE_right = GripperActSrvTcp_Ip(act_srv_name = topic_name_right, robot_ip=ip_right)
    gripper_HandE_left = GripperActSrvTcp_Ip(act_srv_name = topic_name_left, robot_ip=ip_left)
    ex.add_node(gripper_HandE_right)
    ex.add_node(gripper_HandE_left)
    ex.spin()

    # Destroy the node explicitly
   
    rclpy.shutdown()

if __name__ == '__main__':
    main()
