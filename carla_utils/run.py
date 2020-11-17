
import os
from os.path import join
import argparse


if __name__ == "__main__":
    argparser = argparse.ArgumentParser(
            description='Run carla server.')
    argparser.add_argument(
        '-l', '--low-quality',
        action='store_true',
        dest='quality',
        help='quality level')
    argparser.add_argument(
        '-o', '--opengl',
        action='store_false',
        dest='opengl',
        help='use opengl')
    args = argparser.parse_args()

    server_path = os.environ['CARLAPATH']

    cmd = 'bash ' + join(server_path, 'CarlaUE4.sh')
    if args.quality:
        cmd += ' -quality-level=Low'
    if args.opengl:
        cmd += ' -opengl'

    print('\nrunning:\n    '+cmd+'\n')
    os.system(cmd)
