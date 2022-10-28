
import os
import argparse

from .world_map.core import get_port, generate_server_cmd


if __name__ == "__main__":
    argparser = argparse.ArgumentParser(
            description='Run carla server.')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port of CARLA Simulator (default: 2000)')
    argparser.add_argument(
        '-l', '--low-quality',
        action='store_true',
        dest='quality',
        help='quality level')
    argparser.add_argument(
        '-o', '--no-opengl',
        action='store_true',
        dest='no_opengl',
        help='use opengl or not')
    argparser.add_argument(
        '-i', '--gpu-index',
        dest='gpu_index',
        default=0,
        type=int,
        help='gpu index')
    argparser.add_argument(
        '--no',
        action='store_true',
        dest='no_display',
        help='whether display')
    args = argparser.parse_args()

    port = get_port(args.port)
    cmd = generate_server_cmd(port, args.gpu_index, args.quality, not args.no_opengl, args.no_display)

    print('\nrunning:\n    '+cmd+'\n')
    os.system(cmd)
