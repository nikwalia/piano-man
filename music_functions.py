import numpy as np
import mido
import piano

def load_midi(file_path: str, track_id: 1):
    if file_path is None:
        return None
    
    mid = mido.MidiFile(file_path)
    return mid.tracks[track_id]

class KeyAction(object):
    def __init__(self, key_ids: list, start_time: int, end_time: int):
        self.start_time = start_time
        self.end_time = end_time
        self.keys = sorted(key_ids)
        self.target_locs = []
    
    def convert_targets(self, piano_definition: dict):
        for key in self.keys:
            if key in piano_definition['white']:
                self.target_locs.append(piano_definition['white'][key])
            else:
                self.target_locs.append(piano_definition['black'][key])

    def attempt_solve_ik(self, robot):
        pass

def track_to_seq(track):
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
