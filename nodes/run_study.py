#!/usr/bin/env rosh
import webbrowser
import hashlib, time, random
import rospy
import numpy as np
import os

COND_BASELINE   = 0
COND_SCREEN     = 1
COND_PROJ_MOUSE = 2
COND_PROJ_GLASS = 3

CONDITIONS = [1,2,3]

VIDEO_DIR = '/home/lazewatd/videos'

class VideoInfo:
    def __init__(self, quiz_url, filename, duration, instr_text='', pre=[], post=[]):
        self.quiz = quiz_url
        self.filename = filename
        self.duration = duration
        self.instr_text = instr_text
        self.pre = pre
        self.post = post

class Videos:

    class __metaclass__(type):
        def __len__(self):
            return 4

        def __getitem__(self, idx):
            return {
                COND_BASELINE: self.london1,
                COND_SCREEN: self.uk,
                COND_PROJ_MOUSE: self.eu,
                COND_PROJ_GLASS: self.london2
            }[idx]

        def __iter__(self):
            return (self[i] for i in xrange(len(self)))

    eu = VideoInfo(
        'http://oregonstate.qualtrics.com/SE/?SID=SV_b89FnVhhBcavwZ7&subject_id=%s',
        'eu.mp4',
        rospy.Duration(350)
    )
    london1 = VideoInfo(
        'http://oregonstate.qualtrics.com/SE/?SID=SV_a2Gew0tRFAMLUY5&subject_id=%s',
        'london1.mp4',
        rospy.Duration(287)
    )
    london2 = VideoInfo(
        'http://oregonstate.qualtrics.com/SE/?SID=SV_eP71O4yatU9m24l&subject_id=%s',
        'london2.mp4',
        rospy.Duration(347)
    )
    uk = VideoInfo(
        'http://oregonstate.qualtrics.com/SE/?SID=SV_0PvRmY0tbwg2Vpj&subject_id=%s',
        'uk.mp4',
        rospy.Duration(314)
    )

# COND_VIDEOS = {
#     COND_BASELINE   : Videos.london1,
#     COND_SCREEN     : Videos.uk,
#     COND_PROJ_MOUSE : Videos.eu,
#     COND_PROJ_GLASS : Videos.london2
# }

STIMULUS_SEQUENCE = [
    services.forward10,
    services.forward10,
    services.forward10,
    services.forward10,
    services.forward10,
    services.pause,
    services.pause,
    services.pause,
    services.pause,
    services.pause,
    services.toggle_mute,
    services.toggle_mute,
    services.toggle_mute,
    services.toggle_mute,
    services.toggle_mute,
    lambda : (services.vol_dn() and services.vol_dn() and services.vol_dn() and services.vol_dn()),
    lambda : (services.vol_dn() and services.vol_dn() and services.vol_dn() and services.vol_dn()),
    lambda : (services.vol_dn() and services.vol_dn() and services.vol_dn() and services.vol_dn()),
    lambda : (services.vol_dn() and services.vol_dn() and services.vol_dn() and services.vol_dn()),
    lambda : (services.vol_dn() and services.vol_dn() and services.vol_dn() and services.vol_dn()),
]
SEQ_TITLES = [
    'forward10',
    'forward10',
    'forward10',
    'forward10',
    'forward10',
    'pause',
    'pause',
    'pause',
    'pause',
    'pause',
    'mute',
    'mute',
    'mute',
    'mute',
    'mute',
    'vol_dn',
    'vol_dn',
    'vol_dn',
    'vol_dn',
    'vol_dn',
]

def show_instructions_and_wait(text):
    topics.splashscreen.message(text)
    rospy.wait_for_message('/click', msg.std_msgs.Empty)

def get_stimulus_sequence(video):
    # set the seeed so we get "random" but consistent sequences
    random.seed(video.duration.to_sec())
    seq = zip(STIMULUS_SEQUENCE[:], SEQ_TITLES[:])
    random.shuffle(seq)
    # reverse so we can pop as we go
    return list(reversed(zip(np.linspace(10,video.duration.to_sec()-10, len(STIMULUS_SEQUENCE)), seq)))

def generate_subject_id():
    return hashlib.md5(str(time.time())).hexdigest()[:10]

def generate_sequence():
    return sample(list(enumerate(Videos)), len(Videos))

def show_quiz(vid, subject_id):
    print 'opening quiz'
    # open up the quiz in the browser
    rospy.Timer(rospy.Duration(0.0000001), lambda x: webbrowser.open(vid.quiz % subject_id), oneshot=True)
    # webbrowser.open(vid.quiz)
    print 'quiz open, waiting for finish'
    # quiz redirects to a local page that publishes an empty 
    # message to let us know when the quiz has been completed
    rospy.wait_for_message('/survey_done', msg.std_msgs.Empty)
    print 'quiz done'

def run_study():
    subject_id = generate_subject_id()

    # have the user calibrate the glass frame
    topics.splashscreen.message(
        'Please put on Google Glass.\n'
        'Note that you will not need to be able to read its screen.\n'
        'When you have it on comfortably, tap on the right stem to continue.\n'
    )

    rospy.wait_for_service('/projected/display_mute')
    services.projected.display_mute()

    # wait for click
    rospy.wait_for_message('/click', msg.std_msgs.Empty)
    show_instructions_and_wait(
        'Move your head so that the right edge of the Glass screen\n'
        'is just touching the right edge of the screen you\'re reading this on.\n'
        'When the edges are aligned, tap on the right stem to continue.\n')
    packages.microinteraction_study.nodes.glass_offset_py('glass_adjust')

    # now that glass is calibrated, start some practice
    topics.splashscreen.message(
        'You should see a series of shapes projected on the wall.\n'
        'The rotation of your head controls the blue cursor.\n'
        'Please move the cursor into the orange shape and click.\n'
        'If you have any questions, ask the experimenter.'
    )
    services.practice.display_unmute()
    rospy.wait_for_message('/practice/finished', msg.std_msgs.Empty)
    services.practice.display_mute()

    # Instructions
    # topics.splashscreen.message(
    #     'You will be shown a short informational video.\n'
    #     'Please pay attention to details. There will be a quiz at the end\n'
    #     'Tap on the right stem to continue.')

    # # wait for the start video service
    # rospy.wait_for_message('/click', msg.std_msgs.Empty)
    # rospy.wait_for_service('start_video')

    # # run the practice video
    # vid = Videos.london1
    # services.start_video(os.path.join(VIDEO_DIR, vid.filename))

    # # wait for vlc to be ready after starting the video
    # while 'vlc_ready' not in parameters and not parameters.vlc_ready:
    #     rospy.sleep(0.1)

    # # wait for the practice video to run
    # rate = Rate(2)
    # print 'Playing practice video'
    # while ok() and topics.playback_time[0].data.to_sec() < vid.duration.to_sec()-2:
    #     print topics.playback_time[0].data, vid.duration
    #     rate.sleep()

    show_quiz(vid, subject_id)
    # make really sure the video is stopped
    # services.stop()

    print 'Starting conditions'
    for condition, vid in generate_sequence():
        to_kill = []
        print 'starting condition', condition
        if condition COND_PROJ_MOUSE:
            # show the projected interface
            services.projected.display_unmute()
            to_kill.append(packages.splashscreen.nodes.mouse_capture_py(
                '_frame_id:=%s' % wall_frame_id,
                node_name=parameters.mouse_capture_node))
        elif condition == COND_PROJ_GLASS:
            services.projected.display_unmute()
        elif condition == COND_SCREEN:
            pass
            # TODO: make sure the web interface is showing
            # rospy.Timer(rospy.Duration(0.0000001), lambda x: webbrowser.open(WEB_INTERFACE), oneshot=True)

        # vid = COND_VIDEOS[condition]
        stim_seq = get_stimulus_sequence(vid)
        services.start_video(os.path.join(VIDEO_DIR, vid.filename))
        
        # wait for vlc to be ready after starting the video
        while 'vlc_ready' not in parameters and not parameters.vlc_ready:
            rospy.sleep(0.1)    

        # while topics.playback_time[0].data <= vid.duration:
        stim_time, (stim, st) = stim_seq.pop()
        for t in topics.playback_time[:]:
            print t.data.to_sec(), 'of', vid.duration.to_sec()
            # stimuli go here
            if t.data.to_sec() >= stim_time:
                print st
                stim()
                if stim_seq:
                    stim_time, (stim, st) = stim_seq.pop()
            if t.data >= vid.duration:
                print 'done'
                break
        for n in to_kill: kill(n)

        # make really sure the video is stopped
        # services.stop()

        # hide the projected interface (regardless of if it's open)
        services.projected.display_mute()

        show_quiz(vid, subject_id)

run_study()