#!/usr/bin/env rosh
import webbrowser as wb
import hashlib
import time
import random
import rospy
import numpy as np
import os
from functools import partial
import rosgraph
import shlex
import subprocess

webbrowser = wb.get('firefox')

COND_BASELINE   = 0
COND_SCREEN     = 1
COND_PROJ_MOUSE = 2
COND_PROJ_GLASS = 3

CONDITIONS = [1,2,3]

VIDEO_DIR = '/home/thrain/videos'

WEB_INTERFACE = 'file://' + os.path.join(packages.microinteraction_study.path, 'interfaces', 'web', 'simple.html')
class VideoInfo:
    def __init__(self, quiz_url, filename, duration, instr_text='', pre=[], post=[]):
        self.quiz = quiz_url
        self.filename = filename
        self.duration = duration
        self.instr_text = instr_text
        self.pre = pre
        self.post = post

COND_STR = {
    COND_BASELINE: 'COND_BASELINE: London 1',
    COND_SCREEN: 'COND_SCREEN: UK',
    COND_PROJ_MOUSE: 'COND_PROJ_MOUSE: EU',
    COND_PROJ_GLASS: 'COND_PROJ_GLASS: London 2'
}

class Videos:

    class __metaclass__(type):
        def __len__(self):
            #return 4
            return 3

        def __getitem__(self, idx):
            return {
                COND_BASELINE: self.london1,
                COND_SCREEN: self.uk,
                COND_PROJ_MOUSE: self.eu,
                #COND_PROJ_GLASS: self.london2
            }[idx]

        def __iter__(self):
            return (self[i] for i in xrange(len(self)))

    eu = VideoInfo(  # COND_PROJ_MOUSE
        'http://oregonstate.qualtrics.com/SE/?SID=SV_b89FnVhhBcavwZ7&subject_id=%s',
        'eu.mp4',
        rospy.Duration(350),
        # rospy.Duration(10),
        pre=[services.projected.display_unmute,
             partial(packages.splashscreen.nodes.mouse_capture_py,
                node_name=parameters.mouse_capture_name())
            ],
        post=[partial(kill, nodes[parameters.mouse_capture_name()]),
              services.projected.display_mute],
        instr_text=('In this condition, you will use a projected interface\n'
                    'in conjunction with a standard computer mouse.\n'
                    'When you\'re ready, click to continue.')
    )
    london1 = VideoInfo(  # COND_BASELINE
        'http://oregonstate.qualtrics.com/SE/?SID=SV_a2Gew0tRFAMLUY5&subject_id=%s',
        'london1.mp4',
        rospy.Duration(287),
        # rospy.Duration(10),
        instr_text=('This condition does not have any interruptions.\n'
                    'Sit back, relax, and absorb the information.\n'
                    'When you\'re ready, click to continue.')

    )
    london2 = VideoInfo(  # COND_PROJ_GLASS
        'http://oregonstate.qualtrics.com/SE/?SID=SV_eP71O4yatU9m24l&subject_id=%s',
        'london2.mp4',
        rospy.Duration(347),
        # rospy.Duration(10),
        pre=[packages.glass_ros_bridge.nodes.start_glass,
             services.projected.display_unmute],
        post=[packages.glass_ros_bridge.nodes.stop_glass,
             services.projected.display_mute,
             partial(kill, nodes.hide_web)],
        instr_text=('In this condition, you will use a projected interface\n'
                    'in conjunction with Google Glass.\n'
                    'When you\'re ready, click to continue.')
    )
    uk = VideoInfo(  # COND_SCREEN
        'http://oregonstate.qualtrics.com/SE/?SID=SV_0PvRmY0tbwg2Vpj&subject_id=%s',
        'uk.mp4',
        rospy.Duration(314),
        # rospy.Duration(10),
        pre=[partial(webbrowser.open, WEB_INTERFACE)],
        instr_text=('In this condition, you will use a browser-based interface\n'
                    'in conjunction with a standard computer mouse.\n'
                    'When you\'re ready, click to continue.')
    )

STIMULUS_SEQUENCE = [
    services.forward10slow,
    services.forward10slow,
    services.forward10slow,
    services.forward10slow,
    services.forward10slow,
    services.forward10slow,
    services.pause,
    services.pause,
    services.pause,
    services.pause,
    services.pause,
    services.pause,
    lambda: services.set_vol(160),
    lambda: services.set_vol(160),
    lambda: services.set_vol(160),
    lambda: services.set_vol(160),
    lambda: services.set_vol(160),
    lambda: services.set_vol(160),
]
SEQ_TITLES = [
    'forward10',
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
    'pause',
    'vol_dn',
    'vol_dn',
    'vol_dn',
    'vol_dn',
    'vol_dn',
    'vol_dn',
]


def show_instructions_and_wait(text, local=False):
    ns = 'splashscreen' + ('_local' if local else '')
    topics[ns].message(text)
    rospy.wait_for_message(rosgraph.names.ns_join(ns, 'advance'), msg.std_msgs.Empty)

def get_stimulus_sequence(video):
    # set the seeed so we get "random" but consistent sequences
    random.seed(video.duration.to_sec())
    seq = zip(STIMULUS_SEQUENCE[:], SEQ_TITLES[:])
    random.shuffle(seq)
    # reverse so we can pop as we go
    return list(reversed(zip(np.linspace(10,video.duration.to_sec()-10, len(STIMULUS_SEQUENCE)), seq)))

def generate_subject_id():
    return hashlib.md5(str(time.time())).hexdigest()[:10]

def generate_sequence(subject_id):
    #random.seed(subject_id)
    random.seed(0)
    return random.sample(list(enumerate(Videos)), len(Videos))
    # return [(0, Videos[0]), (1, Videos[1])]
    #return [(COND_PROJ_MOUSE, Videos[COND_PROJ_MOUSE])]
    #return [(COND_PROJ_GLASS, Videos[COND_PROJ_GLASS])]

def show_quiz(vid, subject_id):
    print 'opening quiz: ', vid.quiz % subject_id
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
    # show_instructions_and_wait(
    #     'Please put on Google Glass.\n'
    #     'Note that you will not need to\n'
    #     'be able to read its screen.\n'
    #     'When you have it on\n'
    #     'comfortably, tap on the\n'
    #     'right stem to continue.\n'
    # )

    # rospy.wait_for_service('/projected/display_mute')
    # services.projected.display_mute()

    # # wait for click
    # show_instructions_and_wait(
    #     'Move your head so that the\n'
    #     'right edge of the Glass screen\n'
    #     'is just touching the right edge\n'
    #     'of the screen you\'re reading this\n'
    #     'on. When the edges are aligned,\n'
    #     'tap on the right stem to continue.\n')


    # packages.microinteraction_study.nodes.glass_offset_py('glass_adjust', node_name='glass_offset_node')

    # #now that glass is calibrated, start some practice
    # services.practice.display_unmute()
    # topics.splashscreen.message(
    #     'You should see a series of\n'
    #     'shapes projected to the right.\n'
    #     'The rotation of your head\n'
    #     'controls the blue cursor.\n'
    #     'Please move the cursor into\n'
    #     'the orange shape and click.\n'
    #     'If you have any questions,\n'
    #     'ask the experimenter.'
    # )
    # rospy.wait_for_message('/practice/finished', msg.std_msgs.Empty)
    # services.practice.display_mute()

    packages.glass_ros_bridge.nodes.stop_glass()

    parameters.subject_id = subject_id

    print 'Starting conditions'
    for condition, vid in generate_sequence(subject_id):
        to_kill = []
        parameters.condition = COND_STR[condition]
        print 'starting condition', COND_STR[condition]

        # show the instructions
        show_instructions_and_wait(vid.instr_text, local=True)

        # run the prereqs
        for c in vid.pre:
            called = c()

        services.start_video(os.path.join(VIDEO_DIR, vid.filename))

        # wait for vlc to be ready after starting the video
        while 'vlc_ready' not in parameters and not parameters.vlc_ready():
            rospy.sleep(0.1)

        # make sure the volume is at ~150% (calibrated)
        services.set_vol(400)

        if condition != COND_BASELINE:
            stim_seq = get_stimulus_sequence(vid)
        else:
            # since stim_seq needs to be nonempty,
            # let's add one that does nothing way in the future
            stim_seq = [(1000000000, (lambda : None, 'noop'))]
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

        services.stop()

        for n in to_kill: kill(n)        

        # run the postreqs
        print 'killing condition-speicific nodes'
        for c in vid.post:
            c()
        # show the quiz
        show_quiz(vid, subject_id)


run_study()
