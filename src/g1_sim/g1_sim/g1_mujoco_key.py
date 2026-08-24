import argparse
import subprocess
import sys


def _parse_args(argv):
    parser = argparse.ArgumentParser(description='Send a key press to the MuJoCo viewer with xdotool.')
    parser.add_argument('key', choices=['7', '8', '9'])
    parser.add_argument('--window-name', default='MuJoCo')
    return parser.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    result = subprocess.run(
        ['xdotool', 'search', '--name', args.window_name],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    windows = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    if result.returncode != 0 or not windows:
        raise SystemExit(f'No MuJoCo window found matching {args.window_name!r}')

    window = windows[-1]
    subprocess.run(['xdotool', 'windowactivate', window, 'key', args.key], check=True)


if __name__ == '__main__':
    main()