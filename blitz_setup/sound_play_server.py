import rospy
import subprocess

from std_msgs.msg import String

play_pub = rospy.Publisher('audio_finish', String, queue_size=1)
def callback(data):

    subprocess.call('rosrun sound_play play.py /home/fetch/blitz_setup/wavefile/TTS.wav', shell=True)
    play_pub.publish('done')
    return

def main():

    rospy.init_node('sound_play_server', anonymous=True)
    rospy.Subscriber('audio_cmd', String, callback)
    rospy.spin()

if __name__ == '__main__':

    main()
