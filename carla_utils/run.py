
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
    argparser.add_argument(
        '--no',
        action='store_true',
        dest='no_display',
        help='whether display')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port of CARLA Simulator (default: 2000)')
    args = argparser.parse_args()

    server_path = os.environ['CARLAPATH']

    cmd = 'bash ' + join(server_path, 'CarlaUE4.sh')

    assert args.port % 2 == 0
    cmd += ' -carla-rpc-port=' + str(args.port)

    if args.quality:
        cmd += ' -quality-level=Low'
    if args.opengl:
        cmd += ' -opengl'
    if args.no_display:
        cmd = 'DISPLAY= ' + cmd
    print('\nrunning:\n    '+cmd+'\n')
    os.system(cmd)
