import rosbag
from std_msgs.msg import Int32, String

def main(args=None):

    bag = rosbag.Bag('test.bag', 'w')

    try:
        s = String()
        s.data = 'foo'

        i = Int32()
        i.data = 42

        bag.write('chatter', s)
        bag.write('numbers', i)
    finally:
        bag.close()


if __name__ == '__main__':
    main()
