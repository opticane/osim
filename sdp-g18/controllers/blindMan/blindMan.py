"""blindMan controller."""

# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pedestrian class container."""
from controller import Supervisor, Keyboard

import optparse
import math
import lidarOutputUtils as lou
from speechCommand import listen, executeCommand
from battery_indicator_utils import battery_to_haptic
from haptic_guide import haptic_guide
from common_utils import rotate_point

class Pedestrian (Supervisor):
    """Control a Pedestrian PROTO."""

    def __init__(self):
        """Constructor: initialize constants."""
        self.BODY_PARTS_NUMBER = 13
        self.WALK_SEQUENCES_NUMBER = 8
        self.ROOT_HEIGHT = 1.27
        self.CYCLE_TO_DISTANCE_RATIO = 0.22
        self.speed = 1.15
        self.clearLidarFeedback = False
        self.commandOutput = []
        self.buttonPressed = False
        self.add_new_current_waypoint = False
        self.caneMovement = True
        self.stopMoving = False
        self.stopMovingFeedbackLevel = 1.0
        self.delayMoving = False
        self.timeNotMoving = 0
        self.stopTimestamp = 0
        self.delayStartTimestamp = 0
        self.resumeMovingDelay = 1.5
        self.timeAfterDelayStart = 0
        self.runTime = 0
        self.current_height_offset = 0
        self.relativeCaneAngle = 0
        self.locationOne = [-15, 15]
        self.joints_position_field = []
        self.joint_names = [
            "leftArmAngle", "leftLowerArmAngle", "leftHandAngle",
            "rightArmAngle", "rightLowerArmAngle", "rightHandAngle", 
            "rightArmAngle2", "rightLowerArmAngle2","rightHandAngle2",
            "leftLegAngle", "leftLowerLegAngle", "leftFootAngle",
            "rightLegAngle", "rightLowerLegAngle", "rightFootAngle",
            "headAngle"
        ]
        self.height_offsets = [  # those coefficients are empirical coefficients which result in a realistic walking gait
            -0.02, 0.04, 0.08, -0.03, -0.02, 0.04, 0.08, -0.03
        ]
        
        self.angles = [  # those coefficients are empirical coefficients which result in a realistic walking gait
            [-0.52, -0.15, 0.58, 0.7, 0.52, 0.17, -0.36, -0.74],  # left arm
            [0.0, -0.16, -0.7, -0.38, -0.47, -0.3, -0.58, -0.21],  # left lower arm
            [0.12, 0.0, 0.12, 0.2, 0.0, -0.17, -0.25, 0.0],  # left hand
            #[0.52, 0.17, -0.36, -0.74, -0.52, -0.15, 0.58, 0.7],  # right arm
            #[-0.47, -0.3, -0.58, -0.21, 0.0, -0.16, -0.7, -0.38],  # right lower arm
            #[0.0, -0.17, -0.25, 0.0, 0.12, 0.0, 0.12, 0.2],  # right hand
            [-0.4], # right arm
            [-0.5], # right lower arm
            [0.8, 0.75, 0.75, 0.9, 0.75, 0.75, 0.9, 1], # right hand
            [0.0], # right arm "horizontal"
            [0.15, 0.1, 0.05, 0, 0.05, 0.1, 0.15, 0.20], # right lower arm "horizontal"
            [0.37, 0.14, -0.08, -0.3, -0.08, 0.14, 0.37, 0.6], # right hand "horizontal"
            [-0.55, -0.85, -1.14, -0.7, -0.56, 0.12, 0.24, 0.4],  # left leg
            [1.4, 1.58, 1.71, 0.49, 0.84, 0.0, 0.14, 0.26],  # left lower leg
            [0.07, 0.07, -0.07, -0.36, 0.0, 0.0, 0.32, -0.07],  # left foot
            [-0.56, 0.12, 0.24, 0.4, -0.55, -0.85, -1.14, -0.7],  # right leg
            [0.84, 0.0, 0.14, 0.26, 1.4, 1.58, 1.71, 0.49],  # right lower leg
            [0.0, 0.0, 0.42, -0.07, 0.07, 0.07, -0.07, -0.36],  # right foot
            [0.18, 0.09, 0.0, 0.09, 0.18, 0.09, 0.0, 0.09]  # head
        ]
        Supervisor.__init__(self)

    def updateWaypointsDistance(self):
        self.waypoints_distance = []
        for j in range(0, self.number_of_waypoints):
            x = self.waypoints[j][0] - self.waypoints[(j + 1) % self.number_of_waypoints][0]
            z = self.waypoints[j][1] - self.waypoints[(j + 1) % self.number_of_waypoints][1]
            if j == 0:
                self.waypoints_distance.append(math.sqrt(x * x + z * z))
            else:
                self.waypoints_distance.append(self.waypoints_distance[j - 1] + math.sqrt(x * x + z * z))


    def run(self):
        
        # initate lidar
        timeStep = 64
        lidar = self.getDevice('LDS-01')
        lidar.enable(timeStep)
        # lidarWidth = lidar.getHorizontalResolution()
        
        """Set the Pedestrian pose and position."""
        opt_parser = optparse.OptionParser()
        opt_parser.add_option("--trajectory", default="", help="Specify the trajectory in the format [x1 y1, x2 y2, ...]")
        opt_parser.add_option("--speed", type=float, help="Specify walking speed in [m/s]")
        opt_parser.add_option("--step", type=int, help="Specify time step (otherwise world time step is used)")
        opt_parser.add_option("--caneMovement", type=int, help="Should the cane move sideways?")
        options, args = opt_parser.parse_args()
        if not options.trajectory or len(options.trajectory.split(',')) < 2:
            print("You should specify the trajectory using the '--trajectory' option.")
            print("The trajectory shoulld have at least 2 points.")
            return
        if options.speed and options.speed > 0:
            self.speed = options.speed
        if options.step and options.step > 0:
            self.time_step = options.step
        else:
            self.time_step = int(self.getBasicTimeStep())
        if options.caneMovement == 0:
            self.caneMovement = bool(options.caneMovement)

        point_list = options.trajectory.split(',')
        self.number_of_waypoints = len(point_list)
        self.waypoints = []
        for i in range(0, self.number_of_waypoints):
            self.waypoints.append([])
            self.waypoints[i].append(float(point_list[i].split()[0]))
            self.waypoints[i].append(float(point_list[i].split()[1]))
        self.root_node_ref = self.getSelf()
        self.root_translation_field = self.root_node_ref.getField("translation")
        self.root_rotation_field = self.root_node_ref.getField("rotation")
        for joint in self.joint_names:
            self.joints_position_field.append(self.root_node_ref.getField(joint))
            
        # compute waypoints distance
        self.updateWaypointsDistance()

        # enable keyboard
        keyboard = Keyboard()
        keyboard.enable(self.time_step)

        # main cycle
        while not self.step(self.time_step) == -1:
            
            # lidar
            # inf if nothing, outputs a float if there is a solid object in way 
            imageData = lidar.getRangeImage()
            # choose number of partitions and field of view
            partitionMinima, partitionBorders = lou.findPartitionMinima(imageData, 5, 180)
            print("Border angles of partitions: " + str (partitionBorders))
            print("Closest distance for each partition: " + str(["%.2f"%x for x in partitionMinima]))
            # choose min and max values for distance to be detected and converted to feedback
            # LDS-01 range on manufacturer website: 0.12m ~ 3.5m
            feedbackLevels = lou.getFeedbackLevels(partitionMinima, 0.5, 3)
            print("Feedback levels for each partition: " + str(["%.2f"%x for x in feedbackLevels]))
            
            if not self.clearLidarFeedback:
                self.commandOutput = feedbackLevels

            # print table
            lou.printFeedbackLevels(self.commandOutput, 5)

            # listen for keyboard press
            key = keyboard.getKey()

            # action: voice recognition
            if (key == ord('V')):
                self.buttonPressed = True
                self.clearLidarFeedback = True

                commands = ["where is location one", "check the battery"]
                phrases = [(command, 1.0) for command in commands]
                locations = {}
                locations["location one"] = self.locationOne

                command = listen(phrases)
                if command != "":
                    executionOutput = executeCommand(command, locations, {})

                    if isinstance(executionOutput, int):
                        self.commandOutput = battery_to_haptic(executionOutput)
                    elif isinstance(executionOutput, list):
                        currentAngle = math.degrees(self.root_rotation_field.getSFRotation()[3])
                        currentLocation = [self.root_translation_field.getSFVec3f()[i] for i in [0, 2]]

                        self.commandOutput = haptic_guide(currentLocation, executionOutput, -currentAngle - self.relativeCaneAngle)

            # action: show and go to location
            if (key == ord('G')):
                self.buttonPressed = True
                self.clearLidarFeedback = True

                currentAngle = math.degrees(self.root_rotation_field.getSFRotation()[3])
                currentLocation = [self.root_translation_field.getSFVec3f()[i] for i in [0, 2]]
                newDestination = self.locationOne

                self.commandOutput = haptic_guide(currentLocation, newDestination, -currentAngle - self.relativeCaneAngle)

                self.add_new_current_waypoint = True
                self.new_current_waipoint = newDestination

            # action: show location
            if (key == ord('H')):
                self.buttonPressed = True
                self.clearLidarFeedback = True

                currentAngle = math.degrees(self.root_rotation_field.getSFRotation()[3])
                currentLocation = [self.root_translation_field.getSFVec3f()[i] for i in [0, 2]]
                newDestination = self.locationOne

                self.commandOutput = haptic_guide(currentLocation, newDestination, -currentAngle - self.relativeCaneAngle)
            
            # action: stop
            if (key == ord(' ')):
                self.buttonPressed = True

            # action: show battery
            if (key == ord('B')):
                self.buttonPressed = True
                self.clearLidarFeedback = True

                self.commandOutput = battery_to_haptic(35)

            if (key == ord(' ')):
                self.buttonPressed = True

            # action: turn right 90
            if (key == ord('D')):
                currentAngle = math.degrees(self.root_rotation_field.getSFRotation()[3])
                x, z = [self.root_translation_field.getSFVec3f()[i] for i in [0, 2]]

                newDestination = [x + 100 * math.cos(math.radians(-180 - currentAngle)), z + 100 * math.sin(math.radians(-180 - currentAngle))]

                self.add_new_current_waypoint = True
                self.new_current_waipoint = newDestination

            # action: turn right 45
            if (key == ord('E')):
                currentAngle = math.degrees(self.root_rotation_field.getSFRotation()[3])
                x, z = [self.root_translation_field.getSFVec3f()[i] for i in [0, 2]]

                newDestination = [x + 100 * math.cos(math.radians(-225 - currentAngle)), z + 100 * math.sin(math.radians(-225 - currentAngle))]

                self.add_new_current_waypoint = True
                self.new_current_waipoint = newDestination

            # action: turn left 90
            if (key == ord('A')):
                currentAngle = math.degrees(self.root_rotation_field.getSFRotation()[3])
                x, z = [self.root_translation_field.getSFVec3f()[i] for i in [0, 2]]

                newDestination = [x + 100 * math.cos(math.radians(-currentAngle)), z + 100 * math.sin(math.radians(-currentAngle))]

                self.add_new_current_waypoint = True
                self.new_current_waipoint = newDestination

            # action: turn left 45
            if (key == ord('Q')):
                currentAngle = math.degrees(self.root_rotation_field.getSFRotation()[3])
                x, z = [self.root_translation_field.getSFVec3f()[i] for i in [0, 2]]

                newDestination = [x + 100 * math.cos(math.radians(45 - currentAngle)), z + 100 * math.sin(math.radians(45 - currentAngle))]

                self.add_new_current_waypoint = True
                self.new_current_waipoint = newDestination

            # action: turn back 180
            if (key == ord('S')):
                currentAngle = math.degrees(self.root_rotation_field.getSFRotation()[3])
                x, z = [self.root_translation_field.getSFVec3f()[i] for i in [0, 2]]

                newDestination = [x + 100 * math.cos(math.radians(-90 - currentAngle)), z + 100 * math.sin(math.radians(-90 - currentAngle))]

                self.add_new_current_waypoint = True
                self.new_current_waipoint = newDestination

            # action: continue further forward 0
            if (key == ord('W')):
                currentAngle = math.degrees(self.root_rotation_field.getSFRotation()[3])
                x, z = [self.root_translation_field.getSFVec3f()[i] for i in [0, 2]]

                newDestination = [x + 100 * math.cos(math.radians(-270 - currentAngle)), z + 100 * math.sin(math.radians(-270 - currentAngle))]

                self.add_new_current_waypoint = True
                self.new_current_waipoint = newDestination

            # get front partition that relative to the user
            relativeCaneAngle = 0 
            for i in [7, 8]:
                relativeCaneAngle += self.joints_position_field[i].getSFFloat()
            self.relativeCaneAngle = math.degrees(relativeCaneAngle)
            relativeFrontPartitionIndex = [idx for idx, border in enumerate(partitionBorders) if border > relativeCaneAngle][0] - 1

            # following stops on any feedback in any of the patitions
            #if any(map(lambda x: x > 0, feedbackLevels)):
            if feedbackLevels[relativeFrontPartitionIndex] > self.stopMovingFeedbackLevel or self.buttonPressed:
                # stop movement, reset delay
                self.stopMoving = True
                self.delayMoving = False
                self.buttonPressed = False
                self.timeNotMoving += self.timeAfterDelayStart
                self.delayStartTimestamp = 0
                self.timeAfterDelayStart = 0

                if self.stopTimestamp == 0:
                    self.stopTimestamp = self.getTime()  
            else:
                # path is clear
                if self.delayMoving:
                    self.timeAfterDelayStart = self.getTime() - self.delayStartTimestamp

                    if self.timeAfterDelayStart >= self.resumeMovingDelay:
                        # delay ended, resume movement
                        self.timeNotMoving += self.timeAfterDelayStart
                        self.delayMoving = False
                        self.clearLidarFeedback = False
                        self.delayStartTimestamp = 0
                        self.timeAfterDelayStart = 0
                
                if self.stopMoving:
                    # path just got cleared, start delay
                    self.timeNotMoving += self.getTime() - self.stopTimestamp
                    self.stopMoving = False
                    self.stopTimestamp = 0
                    self.delayMoving = True
                    self.delayStartTimestamp = self.getTime()

            # internal total runtime of robot
            time = self.getTime() - self.timeNotMoving          

            if ((not self.stopMoving) and (not self.delayMoving)):      
                # free to move

                current_sequence = int(((time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO) % self.WALK_SEQUENCES_NUMBER)
                # compute the ratio 'distance already covered between way-point(X) and way-point(X+1)'
                # / 'total distance between way-point(X) and way-point(X+1)'
                ratio = (time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO - \
                    int(((time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO))
                    
                # body parts movement
                for i in range(0, len(self.angles)):
                    if i in [3,4,6]:
                        current_angle = self.angles[i][0]
                        self.joints_position_field[i].setSFFloat(current_angle)
                    elif (not self.caneMovement and i in [5,7,8]):
                        if i in [7,8]:
                            current_angle = 0
                        else:
                            current_angle = self.angles[i][1]
                        self.joints_position_field[i].setSFFloat(current_angle)
                    else:
                        current_angle = self.angles[i][current_sequence] * (1 - ratio) + \
                            self.angles[i][(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio
                        self.joints_position_field[i].setSFFloat(current_angle)
                
                # adjust height
                self.current_height_offset = self.height_offsets[current_sequence] * (1 - ratio) + \
                    self.height_offsets[(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio

                # total distance covered
                distance = time * self.speed

                # subtract distance covered in previous full cycles
                relative_distance = distance - int(distance / self.waypoints_distance[self.number_of_waypoints - 1]) * \
                    self.waypoints_distance[self.number_of_waypoints - 1]

                # find current waypoint that we are heading to
                for i in range(0, self.number_of_waypoints):
                    if self.waypoints_distance[i] > relative_distance:
                        break
                
                # if new waypoint is to be added
                if self.add_new_current_waypoint:
                    self.add_new_current_waypoint = False

                    self.number_of_waypoints += 2
                    current_translation = self.root_translation_field.getSFVec3f()
                    x = current_translation[0]
                    z = current_translation[2]
                    self.waypoints = self.waypoints[:i + 1] + [[x, z]] + [self.new_current_waipoint] + self.waypoints[i + 1:]
                    
                    # calculate the new waypoint distances list
                    self.updateWaypointsDistance()

                    # note: relative distance does not need to be updated after waypoint is added

                # how much distance is covered between the last and current waypoint
                distance_ratio = 0
                if i == 0:
                    distance_ratio = relative_distance / self.waypoints_distance[0]
                else:
                    distance_ratio = (relative_distance - self.waypoints_distance[i - 1]) / \
                        (self.waypoints_distance[i] - self.waypoints_distance[i - 1])
                # new coordinates
                x = distance_ratio * self.waypoints[(i + 1) % self.number_of_waypoints][0] + \
                    (1 - distance_ratio) * self.waypoints[i][0]
                z = distance_ratio * self.waypoints[(i + 1) % self.number_of_waypoints][1] + \
                    (1 - distance_ratio) * self.waypoints[i][1]
                root_translation = [x, self.ROOT_HEIGHT + self.current_height_offset, z]
                # new angle
                angle = math.atan2(self.waypoints[(i + 1) % self.number_of_waypoints][0] - self.waypoints[i][0],
                                    self.waypoints[(i + 1) % self.number_of_waypoints][1] - self.waypoints[i][1])
                rotation = [0, 1, 0, angle]

                # update position and rotation
                self.root_translation_field.setSFVec3f(root_translation)
                self.root_rotation_field.setSFRotation(rotation)
            


controller = Pedestrian()
controller.run()
