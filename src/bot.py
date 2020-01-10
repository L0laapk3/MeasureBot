import math

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket


import time

class MyBot(BaseAgent):

    def initialize_agent(self):
        
        self.ticks = 0
        self.lastTime = 0

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
 
        self.ticks += 1
        if int(self.lastTime) != int(packet.game_info.seconds_elapsed):
            print(f"did {self.ticks} ticks this second.")
            self.ticks = 0
        
        self.lastTime = packet.game_info.seconds_elapsed

        # sleeps for 50% of the available time. This should still give it plenty of time..
        time.sleep(1 / 120 * 0.5)
        


        return SimpleControllerState()
