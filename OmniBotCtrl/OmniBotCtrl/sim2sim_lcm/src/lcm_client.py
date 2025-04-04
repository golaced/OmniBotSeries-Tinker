import lcm
import sys
sys.path.append('/home/pi/Downloads/LocomotionWithNP3O-master/sim2sim_lcm/build')#包含殡仪后的头文件
import numpy as np
from my_lcm import Request, Response

class LCMClient:
    def __init__(self):
        self.lc = lcm.LCM()
        self.subscription = self.lc.subscribe("RESPONSE_CHANNEL", self.handle_response)

    def send_request(self, data):
        policy_input = Request()
        # msg.timestamp = self.get_current_time()
        array = np.random.rand(1, 736)
        policy_input.rows, policy_input.cols = array.shape
        policy_input.data = array.flatten().tolist()
        self.lc.publish("REQUEST_CHANNEL", policy_input.encode())

    def handle_response(self, channel, data):
        action = Response.decode(data)
        for i in action.data:
            print(i)
        return np.array(action.data) # 这里的msg其实就是经过c++处理obs后返回的action
        

    # def get_current_time(self):
    #     # Implement a function to get the current timestamp
    #     return 0


    def run(self):
        # try:
            # while True:
        self.lc.handle()
        # except KeyboardInterrupt:
        #     pass

if __name__ == "__main__":
    client = LCMClient()
    c = 1
    while True:
        client.send_request("Hello, server!!@!")
        client.run()
        c += 1
        if c == 10:
            break