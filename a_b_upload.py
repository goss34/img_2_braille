import requests
import time
from datetime import datetime
import os

class CanuteShapeSwitcher:
    def __init__(self):
        self.flashair_ip = '192.168.0.100'
        
    def read_canute_state(self):
        """Read the current canute_state.txt file"""
        try:
            response = requests.get(f'http://{self.flashair_ip}/canute_state.txt')
            if response.status_code == 200:
                return response.text
            return None
        except:
            return None
            
    def write_canute_state(self, state_content):
        """Write updated state file"""
        try:
            headers = {
                'Content-Type': 'text/plain',
                'Cache-Control': 'no-cache',
                'Pragma': 'no-cache'
            }
            response = requests.put(
                f'http://{self.flashair_ip}/canute_state.txt',
                data=state_content,
                headers=headers
            )
            return response.status_code in [200, 201]
        except:
            return False

    def switch_to_shape(self, shape_name):
        """Update state to point to a specific shape file"""
        print(f"\nAttempting to switch to {shape_name} at {datetime.now().strftime('%H:%M:%S')}")
        
        current_state = self.read_canute_state()
        if not current_state:
            print("! Could not read current state")
            return False

        # Create new state pointing to the shape file
        new_state = []
        state_updated = False
        for line in current_state.splitlines():
            if line.startswith('current_book'):
                new_state.append(f'current_book="sd-card/{shape_name}.brf"')
                state_updated = True
            else:
                new_state.append(line)
        
        if not state_updated:
            new_state.append(f'current_book="sd-card/{shape_name}.brf"')
            
        # Write temporary state to force refresh
        temp_state = '\n'.join(new_state).replace(shape_name, 'temp_trigger')
        if self.write_canute_state(temp_state):
            time.sleep(1)  # Give device time to process
            # Write actual desired state
            if self.write_canute_state('\n'.join(new_state)):
                print(f"✓ Successfully switched to {shape_name}")
                return True
                
        print(f"! Failed to switch to {shape_name}")
        return False

    def force_refresh(self):
        """Send multiple refresh commands"""
        commands = [
            'command.cgi?op=refresh',
            'upload.cgi?WRITEPROTECT=OFF',
            'command.cgi?op=sync',
            'command.cgi?op=cleanup'
        ]
        
        print("Sending refresh commands...")
        for cmd in commands:
            try:
                url = f'http://{self.flashair_ip}/{cmd}'
                response = requests.get(url, timeout=2)
                if response.status_code == 200:
                    print(f"✓ {cmd} successful")
            except:
                print(f"! {cmd} failed")

    def run_test_sequence(self):
        """Run through a test sequence of shape switches"""
        shapes = ['square', 'circle', 'star']
        
        print("Starting shape switching test sequence")
        print("Current shape files:", shapes)
        print("\nPress Ctrl+C to stop the sequence")
        
        try:
            while True:
                for shape in shapes:
                    self.switch_to_shape(shape)
                    self.force_refresh()
                    
                    # Wait for user input before continuing
                    input(f"\nPress Enter to switch to the next shape...")
                    
        except KeyboardInterrupt:
            print("\nTest sequence stopped by user")

def main():
    switcher = CanuteShapeSwitcher()
    switcher.run_test_sequence()

if __name__ == "__main__":
    main()