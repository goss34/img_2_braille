#!/usr/bin/env python3
import os
import curses
import glob

class BrfViewer:
    def __init__(self, folder_path):
        self.folder_path = folder_path
        self.current_index = 0
        self.files = []
        self.update_file_list()

    def update_file_list(self):
        """Update the list of .brf files in the folder"""
        self.files = sorted(glob.glob(os.path.join(self.folder_path, "*.brf")))
        
    def get_current_content(self):
        """Read and return content of current file"""
        if not self.files:
            return "No .brf files found in directory"
        try:
            with open(self.files[self.current_index], 'r') as f:
                return f.read()
        except Exception as e:
            return f"Error reading file: {str(e)}"

    def next_file(self):
        """Switch to next file"""
        if self.files:
            self.current_index = (self.current_index + 1) % len(self.files)

    def prev_file(self):
        """Switch to previous file"""
        if self.files:
            self.current_index = (self.current_index - 1) % len(self.files)

def main(stdscr):
    # Initialize color pairs
    curses.start_color()
    curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_BLACK)
    
    # Hide the cursor
    curses.curs_set(0)
    
    # Initialize viewer with folder path
    viewer = BrfViewer("../Shapes/")  # Change this path
    
    # Main loop
    while True:
        # Clear screen
        stdscr.clear()
        
        # Get current file content
        content = viewer.get_current_content()
        
        # Display content
        stdscr.addstr(0, 0, content, curses.color_pair(1))
        
        # Refresh screen
        stdscr.refresh()
        
        # Get key input
        key = stdscr.getch()
        
        # Handle key input
        if key == curses.KEY_RIGHT:
            viewer.next_file()
        elif key == curses.KEY_LEFT:
            viewer.prev_file()
        elif key == ord('q'):  # Press 'q' to quit
            break
        elif key == ord('r'):  # Press 'r' to refresh file list
            viewer.update_file_list()

if __name__ == "__main__":
    # Initialize curses
    curses.wrapper(main)