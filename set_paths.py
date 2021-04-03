"""Set input and output paths for container extraction."""

import argparse
import os


def main():
    """Main method."""

    paths = {
        "in": "../25d_051_2020_11_25_18_54_50_cleaned.ply",
        "out": "waypoints.pkl"
    }

    if os.path.exists("files.path"):
        with open("files.path", "r") as f:
            for line in f.readlines():
                key, value = line.split('=')
                paths[key] = value.strip()

    parser = argparse.ArgumentParser()
    parser.add_argument('-in',
                        dest="input",
                        help='path to the input point cloud data (.ply)')
    parser.add_argument('-out',
                        dest="output",
                        help='path to the output waypoints list (.pkl)')

    args = parser.parse_args()

    if args.input:
        if not args.input.endswith(".ply"):
            raise Exception("Input file does not end with .ply")
        paths["in"] = args.input
    if args.output:
        if not args.output.endswith(".pkl"):
            raise Exception("output file does not end with .pkl")
        paths["out"] = args.output

    with open("files.path", "w") as f:
        for key in paths:
            f.write("%s=%s\n" % (key, paths[key]))


if __name__ == '__main__':
    main()
