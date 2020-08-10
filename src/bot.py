import math

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, Vector3, Rotator, GameInfoState

from rlutilities.simulation import Ball, Field, Game, Car, Input
from rlutilities.linear_algebra import vec2, vec3, mat3, euler_to_rotation, look_at, norm, normalize, angle_between, dot, orthogonalize, project

from rlutilities.mechanics import Drive as RLUDrive

import time

import sys


TURN_AFTER_RELEASE_DRIFT = True


TICKSTEP = 2
outFile = ""
def testPoints():
    global outFile
    for action in ("brake", "accel", "boost", "drift",):
        outFile = "turn." + action
        drift = action == "drift"
        boost = action == "boost"
        brake = action == "brake"
        print("=========================")
        print(outFile)
        print("=========================")
        
        outFile += ".txt"
        open(outFile, 'w').close()
        for speed in range(0, 2301, 20):
            for ticks in range(0, 800, TICKSTEP):
                yield speed, ticks, drift, boost, brake

testPointGen = testPoints()



class MeasureBot(BaseAgent): 

    def initialize_agent(self):
        
        self.game = Game()
        self.game.set_mode("soccar")
        
        self.needsReset = True
        self.currentTest = next(testPointGen)
        self.remainingTicks = 0
        self.repeatStateSet = 0
        
        self.lastTime = 0
        self.realLastTime = 0
        self.tick = 0
        self.skippedTicks = 0
        self.doneTicks = 0
        self.ticksPassed = 0
        self.driftHadStarted = False
        self.turnPastHalfwayPoint = False
        self.FPS = 120
        self.startPos = None
        self.brake = False
        self.writeBuffer = ""
        self.restartReset = False
        self.lastBeforeResetState = None
        self.lastOutFile = outFile


    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        try:

            self.renderer.begin_rendering()
            self.packet = packet
            self.game.read_game_information(packet, self.get_rigid_body_tick(), self.get_field_info())
            self.car = self.game.cars[self.index]

            self.handleTime()
            self.packet = packet
            if not packet.game_info.is_round_active:
                self.needsReset = True

            controller = SimpleControllerState()
            #controller.steer = 1
            controller.throttle = -1 if self.brake else 1
            
            if self.needsReset:
                if self.restartReset and self.lastBeforeResetState is not None:
                    self.currentTest = self.lastBeforeResetState
                    self.restartReset = False
                startSpeed, self.remainingTicks, drift, boost, self.brake = self.currentTest
                self.reset(startSpeed)
                self.needsReset = False
                self.repeatStateSet = 100
                self.lingeringTicks = 0
                self.writeBuffer = ""
            elif self.repeatStateSet > 0:
                startSpeed, self.remainingTicks, drift, boost, self.brake = self.currentTest
                self.set_game_state(GameState(cars={self.index:  CarState(boost_amount=100, physics=Physics(velocity=Vector3(0, startSpeed, 0)))}))
                self.repeatStateSet -= 1
                if self.repeatStateSet == 0:
                    self.startPos = vec3(self.car.position)
            elif self.remainingTicks > 0:
                _, _, drift, boost, brake = self.currentTest
                controller.steer = 1
                controller.handbrake = drift
                controller.boost = boost
                self.remainingTicks -= 1
            else:
                startSpeed, turnTicks, drift, boost, self.brake = self.currentTest
                if TURN_AFTER_RELEASE_DRIFT:
                    controller.steer = 1
                controller.boost = boost
                if not drift:
                    self.remainingTicks += TICKSTEP - 1

                sidewaysSpeed = norm(project(self.car.velocity, self.car.left()))
                if sidewaysSpeed > 180:
                    controller.boost = boost
                    self.lingeringTicks += 1
                else:
                    rotation = self.packet.game_cars[self.index].physics.rotation.yaw - math.pi/2
                    if rotation < 0:
                        rotation += 2 * math.pi
                    self.currentTest = next(testPointGen)
                    if (self.lingeringTicks > 0 or self.driftHadStarted) if drift else (dot(self.car.velocity, self.car.forward()) >= 0 if self.brake else (rotation > math.pi or not self.turnPastHalfwayPoint)): 
                        self.driftHadStarted = True
                        if rotation > math.pi:
                            self.turnPastHalfwayPoint = True
                        finalOffset = self.car.position - self.startPos
                        outstr = "\t".join(map(str, (startSpeed, rotation, turnTicks + self.lingeringTicks, turnTicks, dot(self.car.velocity, normalize(orthogonalize(self.car.forward(), vec3(0, 0, 1)))), finalOffset[0], finalOffset[1])))
                        print(outstr)
                        self.writeBuffer += outstr + "\n"
                    if (self.driftHadStarted and self.lingeringTicks == 0) if drift else (dot(self.car.velocity, self.car.forward()) < 0 if self.brake else (rotation < math.pi and self.turnPastHalfwayPoint)):
                        print(self.currentTest[0])
                        while startSpeed == self.currentTest[0]: ## drift finished, skip till next
                            self.currentTest = next(testPointGen)
                        self.needsReset = True
                        with open(self.lastOutFile, "a") as f:
                            f.write(self.writeBuffer)
                        self.lastOutFile = outFile
                        self.writeBuffer = ""

                    if startSpeed != self.currentTest[0]:
                        self.driftHadStarted = False
                        self.turnPastHalfwayPoint = False
                        self.lastBeforeResetState = self.currentTest
                    if drift:
                        self.lastBeforeResetState = self.currentTest
                        self.needsReset = True
                        with open(self.lastOutFile, "a") as f:
                            f.write(self.writeBuffer)
                        self.lastOutFile = outFile
                        self.writeBuffer = ""


            self.renderer.end_rendering()
            return controller
        except StopIteration:
            if self.writeBuffer != "":
                with open(outFile, "a") as f:
                    f.write(self.writeBuffer)
                self.writeBuffer = ""
                print("FINISHED!")
            return SimpleControllerState()


    def handleTime(self):
        # this is the most conservative possible approach, but it could lead to having a "backlog" of ticks if seconds_elapsed
        # isnt perfectly accurate.
        if not self.lastTime:
            self.lastTime = self.packet.game_info.seconds_elapsed
        else:
            if self.realLastTime == self.packet.game_info.seconds_elapsed:
                self.needsReset = True
                self.restartReset = True
                return

            if int(self.lastTime) != int(self.packet.game_info.seconds_elapsed):
                #print(f"did {self.doneTicks}, skipped {self.skippedTicks}")
                self.skippedTicks = self.doneTicks = 0

            self.ticksPassed = round(max(1, (self.packet.game_info.seconds_elapsed - self.lastTime) * self.FPS))
            self.lastTime = min(self.packet.game_info.seconds_elapsed, self.lastTime + self.ticksPassed)
            self.realLastTime = self.packet.game_info.seconds_elapsed
            if self.ticksPassed > 1:
                print(f"Skipped {self.ticksPassed - 1} ticks!")
                self.skippedTicks += self.ticksPassed - 1
                self.needsReset = True
                self.restartReset = True
            self.doneTicks += 1


    def reset(self, startSpeed):
        
        car_state = CarState(boost_amount=100, 
                            physics=Physics(location=Vector3(0, -3000, 17.01), velocity=Vector3(0, startSpeed, 0), rotation=Rotator(0, math.pi/2, 0),
                            angular_velocity=Vector3(0, 0, 0)))
        ball_state = BallState(Physics(location=Vector3(3000, 3000, 3000)))
        game_state = GameState(ball=ball_state, cars={self.index: car_state})
        self.set_game_state(game_state)