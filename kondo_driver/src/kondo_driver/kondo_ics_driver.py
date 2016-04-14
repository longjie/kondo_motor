import pykondo

class Kondo(object):
    def __init__(self):
        self.ics = ICSData()
        self.pub = 

    def connect(self, port, baudrate, readback_echo=False):
        ret = ics_init(ics)
        if ret < 0:
            sys.exit(ics.error)

    def update():
        self.ics.
        
    def run(self):
        self.update()
        rospy.spin()

if __name__ == '__main__':
    while true:
        kondo = KondoDriver()
        kondo.connect()
        rate = rospy.Rate()
        kondo.update()
        rate.sleep()
