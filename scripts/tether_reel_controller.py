#!/usr/bin/python
import rospy, re
from aerowake_flight.srv import TetherLength, TetherLengthResponse
from std_msgs.msg import Float64

# for tether hardware control
from multiprocessing import Queue
from Queue import Empty
from reel import reel_run
import math

class TetherReelController(object):
    def __init__(self): # if sim, publish to tether_dynamics; if not, use controller thread
        self.sim = rospy.get_param('~sim', False)
        self.length = rospy.get_param('~tether_takeoff_length', 0.2)
        if self.sim:
            self.pub = rospy.Publisher('sim_tether_limit', Float64, queue_size=1)
        else:
            # establish tether reel comms thread (requires USB connection to reel controller interface)
            # and set takeoff length
            self.commands_to_reel = Queue()
            self.data_from_reel = Queue()
            try:
                self.reel = reel_run(self.commands_to_reel, self.data_from_reel, "tether")
            except Exception as e:
                rospy.logerr('Problem connecting to reel. Aborting.')
                raise e
            self.reel.start()
            self.commands_to_reel.put({"cmd": "goto", "L": self.length})

        self.tltS = rospy.Service('tether_length_tether', TetherLength, self.TLTCallback)

    def shutdown(self):
        if not self.sim:
            self.commands_to_reel.put({'cmd':'exit'})
            self.reel.join()

    def get_reel_data(self): # currently unused...
        '''
        Returns the current length of the tether in meters as believed by the reel controller.
        Expected to return {"L": <length in meters as double>, "T": <tension in newtons as double>}
        '''
        try:
            reel_reading = self.data_from_reel.get(False)
        except Empty:
            reel_reading = {'L':'-', 'T':'-'}
        return reel_reading

    def TLTCallback(self, req):
        matchstr = re.findall(r'(\+|-)?((\d+(\.\d+))|(\d+))?',req.tether_length_string)[0][0:2]
        if matchstr[0] == '+':
            self.length += float(matchstr[1])
        elif matchstr[0] == '-':
            self.length -= float(matchstr[1])
        else:
            self.length = float(matchstr[1])

        # Command tether length change
        if self.sim:
            comm = Float64()
            comm.data = self.length
            self.pub.publish(comm)
        else:
            self.commands_to_reel.put({"cmd": "goto", "L": self.length})

        return TetherLengthResponse(True)

if __name__ == "__main__":
    rospy.init_node("tether_reel_controller", anonymous=True)
    TRC = TetherReelController()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down tether reel controller Node."
        TRC.shutdown()
