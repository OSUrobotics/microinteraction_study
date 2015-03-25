#!/usr/bin/env rosh
import webbrowser
import hashlib, time, random
import rospy
import numpy as np

COND_PRACTICE   = 0
COND_BASELINE   = 1
COND_SCREEN     = 2
COND_PROJECTED  = 3

CONDITIONS = [1,2,3]

VIDEO_DIR='/home/lazewatd/videos'

class VideoInfo:
    def __init__(self, quiz_url, filename, duration):
        self.quiz = quiz_url
        self.filename = filename
        self.duration = duration

class Videos:
    eu = VideoInfo(
        'http://oregonstate.qualtrics.com/SE/?SID=SV_b89FnVhhBcavwZ7&subject_id=%s',
        'eu.mp4',
        rospy.Duration(350)
        # rospy.Duration(5)
    )
    london1 = VideoInfo(
        'http://oregonstate.qualtrics.com/SE/?SID=SV_a2Gew0tRFAMLUY5&subject_id=%s',
        'london1.mp4',
        rospy.Duration(287)
        # rospy.Duration(5)
    )    
    london2 = VideoInfo(
        'http://oregonstate.qualtrics.com/SE/?SID=SV_eP71O4yatU9m24l&subject_id=%s',
        'london2.mp4',
        rospy.Duration(347)
        # rospy.Duration(5)
    )
    uk = VideoInfo(
        'http://oregonstate.qualtrics.com/SE/?SID=SV_0PvRmY0tbwg2Vpj&subject_id=%s',
        'uk.mp4',
        rospy.Duration(314)
        # rospy.Duration(5)

    )

COND_VIDEOS = {
    1: Videos.uk,
    2: Videos.eu,
    3: Videos.london2
}

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
    seq = CONDITIONS[:]
    random.shuffle(seq)
    return seq

def run_study():
    subject_id = generate_subject_id()

    # wait for the start video service
    rospy.wait_for_service('start_video')

    # run the practice video
    vid = Videos.london1
    services.start_video(os.path.join(VIDEO_DIR, vid.filename))

    # wait for vlc to be ready after starting the video
    while 'vlc_ready' not in parameters and not parameters.vlc_ready:
        rospy.sleep(0.1)    

    # wait for the practice video to run
    rate = Rate(2)
    print 'Playing practice video'
    while ok() and topics.playback_time[0].data.to_sec() < vid.duration.to_sec()-2:
        print topics.playback_time[0].data, vid.duration
        rate.sleep()

    # make really sure the video is stopped
    # services.stop()

    print 'Starting conditions'
    for condition in generate_sequence():
        print 'starting condition', condition
        if condition == COND_PROJECTED:
            print '1'
            # show the projected interface
            services.projected.display_unmute()
        else:
            # TODO: make sure the web interface is showing
            # rospy.Timer(rospy.Duration(0.0000001), lambda x: webbrowser.open(WEB_INTERFACE), oneshot=True)
            pass

        vid = COND_VIDEOS[condition]
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

        # make really sure the video is stopped
        # services.stop()

        # hide the projected interface (regardless of if it's open)
        services.projected.display_mute()

        print 'opening quiz'
        # open up the quiz in the browser
        rospy.Timer(rospy.Duration(0.0000001), lambda x: webbrowser.open(vid.quiz), oneshot=True)
        # webbrowser.open(vid.quiz)
        print 'quiz open, waiting for finish'
        # quiz redirects to a local page that publishes an empty 
        # message to let us know when the quiz has been completed
        rospy.wait_for_message('/survey_done', msg.std_msgs.Empty)
        print 'quiz done'

run_study()