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
        self.target_locs = []
    
    def convert_targets(self, piano: Piano):
        for key in self.keys:
            self.target_locs.append(piano.get_key_target(key))

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
