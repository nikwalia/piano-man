import numpy as np
import mido
from piano import Piano

def load_midi(file_path: str, track_id: 1):
    """
    Loads in a .mid file from path. Since there are potentially multiple tracks, the file will need
    to be examined beforehand to manually select a track. Compatible only with grand piano.
    
    :param: file_path: path to the file on disk
    :param: track_id: which track to load. the zeroth track is usually control and can be ignored.
    """
    if file_path is None:
        return None
    
    mid = mido.MidiFile(file_path)
    return mid.tracks[track_id]

class KeyAction(object):
    """
    Represents a note/chord being played.
    """
    def __init__(self, key_ids: list, start_time: int, duration: int):
        self.start_time = start_time
        self.duration = duration
        self.keys = sorted(key_ids)
        self.target_locs = {}
        # 0th- thumb...4th- pinkie
        for i in range(5):
            self.target_locs[i] = -1

    def convert_targets(self, piano: Piano):
        # sourced from https://www.liveabout.com/piano-fingering-placement-guide-2701363#:~:text=Piano%20Chord%20Fingering&text=For%20example%2C%20a%20C%20chord,4%2D5%20is%20also%20acceptable.
        if len(self.keys) == 5:
            for i, key in enumerate(self.keys):
                self.target_locs[i] = piano.get_key_target(key)
        elif len(self.keys) == 4:
            self.target_locs[0] = piano.get_key_target(self.keys[0])
            self.target_locs[1] = piano.get_key_target(self.keys[1])
            self.target_locs[3] = piano.get_key_target(self.keys[2])
            self.target_locs[4] = piano.get_key_target(self.keys[3])
        elif len(self.keys) == 3:
            self.target_locs[0] = piano.get_key_target(self.keys[0])
            self.target_locs[2] = piano.get_key_target(self.keys[1])
            self.target_locs[4] = piano.get_key_target(self.keys[2])
        elif len(self.keys) == 2:
            diff = self.target_locs[1] - self.target_locs[0]
            if diff <= 4:
                # can use index finger
                self.target_locs[1] = piano.get_key_target(self.keys[0])
                self.target_locs[diff - 1] = piano.get_key_target(self.keys[1])
            else:
                # have to use thumb
                self.target_locs[0] = piano.get_key_target(self.keys[0])
                self.target_locs[4] = piano.get_key_target(self.keys[1])
        else:
            self.target_locs[1] = piano.get_key_target(self.keys[0])
    
    def delete_targets(self):
        for i in range(5):
            self.target_locs[i] = -1

def track_to_seq(track):
    """
    Converts a track to a python list of KeyActions
    """
    actions = []

    running_time = 0
    down_keys = []
    up_keys = []
    start_time = running_time
    duration = 0

    for i in range(len(track)):
        if track[i].type == 'note_on':
            down_keys.append(track[i].note)
            if track[i].time != 0:
                start_time = running_time + track[i].time
        elif track[i].type == 'note_off':
            up_keys.append(track[i].note)
            if track[i].time != 0:
                duration = track[i].time
        else:
            continue
        if len(down_keys) == len(up_keys):
            actions.append(KeyAction(down_keys, start_time, duration))

            running_time += track[i].time
            down_keys = []
            up_keys = []
            start_time = 0
            duration = 0
    
    return actions
