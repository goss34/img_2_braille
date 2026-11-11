"""Virtual Canute: run the braille display logic on a laptop, with no ROS, Jetson or board.

    python3 tools/braille_sim                 # opens the simulator window
    python3 tools/braille_sim --instant       # refresh the board instantly instead of at real speed

See docs/simulator.md.
"""
import argparse
import sys

from world import SCENES


def main():
    ap = argparse.ArgumentParser(description='Virtual Canute braille display simulator')
    ap.add_argument('--scene', choices=list(SCENES), default='Two-box test')
    ap.add_argument('--instant', action='store_true', help='refresh the board instantly (default: real speed)')
    ap.add_argument('--line-seconds', type=float, default=1.3, help='seconds per changed line at real speed')
    args = ap.parse_args()
    try:
        import tkinter as tk
    except ImportError:
        sys.exit('This Python has no Tk. On macOS with Homebrew: brew install python-tk@3.15')
    from app import App

    root = tk.Tk()
    App(root, scene=args.scene, real_speed=not args.instant, line_seconds=args.line_seconds)
    root.focus_force()
    root.mainloop()


if __name__ == '__main__':
    main()
