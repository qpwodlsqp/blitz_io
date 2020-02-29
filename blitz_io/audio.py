import rospy
import time
import io
import subprocess

from audio_common_msgs.msg import AudioData
from google.cloud import speech
from pydub import AudioSegment
from gtts import gTTS
from std_msgs.msg import String

class BlitzListen:

    def __init__(self):

        self.audio_topic = '/audio/audio'
        self.file_path = 'audio_files/'
        self.mp3_contents = ''
        self.speech_client = speech.Client.from_service_account_json('speech_auth/speech-e0c1e1f36fe2.json')
        self.play_pub = rospy.Publisher('audio_cmd', String, queue_size=1)
        return

    def mp3_callback(self, audio):

        self.mp3_contents += str(audio.data)
        return

    def topic_to_mp3(self, timeout=5.0):

        rospy.Subscriber(self.audio_topic, AudioData, self.mp3_callback)
        st = time.time()

        while time.time() - st < timeout:
            pass

        self.mp3_file = open(self.file_path+'output.mp3', 'w')
        self.mp3_file.write(self.mp3_contents)
        self.mp3_file.close()
        self.mp3_contents = ''
        return

    def STT(self, filename='output.mp3'):

        sound = AudioSegment.from_mp3(self.file_path+filename)
        sound.export(self.file_path+'stereo_output.wav', format='wav')

        sound = AudioSegment.from_wav(self.file_path+'stereo_output.wav')
        sound = sound.set_channels(1)
        sound.export(self.file_path+'mono_output.wav', format='wav')

        with io.open(self.file_path+'mono_output.wav', 'rb') as audio_file:

            content = audio_file.read()
            sample = self.speech_client.sample(content, encoding='LINEAR16', sample_rate_hertz=16000)

        alts = sample.recognize(language_code='en-US')
        result = alts[0].transcript.lower()

        for alt in alts:

            print alt.transcript.lower()

        print result
        return

    # send this to robot
    def TTS(self, text):

        tts = gTTS(text)
        tts.save(self.file_path+'TTS.mp3')
        sound = AudioSegment.from_mp3(self.file_path+'TTS.mp3')
        sound.export(self.file_path+'TTS.wav', format='wav')
        subprocess.call('sshpass -p robotics scp {}TTS.wav fetch@192.168.1.167:/home/fetch/blitz_setup/wavefile/'.format(self.file_path), shell=True)
        return

    def say(self, text):

        self.TTS(text)
        self.play_pub.publish('play')
        rospy.wait_for_message('audio_finish', String)
        return

    def listen(self, timeout=5.0):

        self.topic_to_mp3(timeout)
        self.STT()

        return
        

####################################################################


if __name__ == '__main__':

    rospy.init_node('blitz_audio_test', anonymous=True)
    listener = BlitzListen()

    '''
    print('Record Starts after 5 secs')
    rospy.sleep(5)
    print('Record START!')
    listener.topic_to_mp3(timeout=10.0)
    '''

    print('start after 5 secs')
    rospy.sleep(5)
    print('start')
    listener.listen(timeout=10.0)

    listener.say('A sample sentence here.')
