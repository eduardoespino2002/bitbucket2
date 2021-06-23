


import argparse
import glob
import numpy as np
import json_tricks as json
import os
import sys
from datetime import datetime

import re


from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import rosbag2_py  # noqa



#print(sys.path)


def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options



def get_args():
    parser = argparse.ArgumentParser(
        #formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description="Convert rosbag to json")

    parser.add_argument("infile", metavar="INFILE", type=str, nargs=1, help="The input bag file")
    parser.add_argument("--pretty", dest="pretty", action="store_true", default=False, help="Pretty print the output JSON")
    parser.add_argument('--topics', '-t', dest="topics", action="store_true", default=False, help="Show topics")
    parser.add_argument('--outfile', '-o', dest='outfile', action='store', default='output.json', help="Output file")
    parser.add_argument('--filter', '-f', dest='filter', action='store', help='Topic filter (regex)')

    args = parser.parse_args(sys.argv[1:])
    return args

import array
def fallback(obj):
    # The multiarray data section identifies as array.array, just convert to a list. The 
    # array typecode doesn't matter
    if type(obj) == type(array.array('I')):
        return list(obj)
    if type(obj) == type(bytes()):
        return list(obj)
    return obj

def main():
    args = get_args()

    infile = args.infile

    #filter = {'/robot_88e9a579/sensor/imu', '/robot_88e9a579/odometry/filtered', '/robot_88e9a579/cmd_vel'}
    if args.filter == None:
        args.filter = '.*'
    matcher = re.compile(args.filter)


    
    storage_options, converter_options = get_rosbag_options(infile[0])
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    topic_names = sorted([topic_types[i].name for i in range(len(topic_types))])
    if args.topics:
        for t in topic_names:
            print('%-80s %s' % (t, type_map[t]))
        sys.exit(0)

    # List of each filter matching message in the bagfile
    events = []
    while reader.has_next():
        (topic, data, t)    = reader.read_next()
        if matcher.match(topic):
            msg_type            = get_message(type_map[topic])
            msg                 = deserialize_message(data, msg_type)

            #print(topic, t, msg_type, msg)
            d = {'topic': topic, 'stamp': t, 'data': msg}
            events.append(d)

    with open(args.outfile, 'w') as f:
        if args.pretty:
            f.write(json.dumps(events, indent=4, primitives=True, extra_obj_encoders=(fallback,)))
        else:
            f.write(json.dumps(events, primitives=True, extra_obj_encoders=(fallback,)))







if __name__ == '__main__':
    main()
