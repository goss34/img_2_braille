import requests
import time
import os
from datetime import datetime
import json

class CanuteFlashAirUploader:
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
        """Write updated state file to try triggering a reload"""
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

    def trigger_reload(self, filename):
        """Attempt to trigger a reload by manipulating state"""
        # Read current state
        current_state = self.read_canute_state()
        if not current_state:
            print("! Could not read current state")
            return False

        # Parse the current book path
        current_book = None
        for line in current_state.splitlines():
            if line.startswith('current_book'):
                current_book = line.split('=')[1].strip().strip('"')
                break

        if not current_book:
            print("! Could not determine current book from state")
            return False

        # Create temporary modified state to force reload
        temp_state = current_state.replace(
            current_book, 
            "sd-card/temp_trigger.brf"
        )
        
        # Write temporary state
        if self.write_canute_state(temp_state):
            time.sleep(1)  # Give device time to process
            # Write original state back
            return self.write_canute_state(current_state)
            
        return False

    def force_refresh(self):
        """Send multiple refresh commands with retries"""
        commands = [
            'command.cgi?op=refresh',
            'upload.cgi?WRITEPROTECT=OFF',
            'command.cgi?op=sync',
            'command.cgi?op=cleanup'
        ]
        
        for cmd in commands:
            for _ in range(3):
                try:
                    url = f'http://{self.flashair_ip}/{cmd}'
                    response = requests.get(url, timeout=2)
                    if response.status_code == 200:
                        break
                except requests.exceptions.RequestException:
                    time.sleep(1)
                    continue

    def upload_file(self, filename, mount_path=None):
        """Upload file and attempt to trigger reload"""
        upload_url = f'http://{self.flashair_ip}/{filename}'
        
        try:
            print(f"\nStarting upload process at {datetime.now().strftime('%H:%M:%S')}")
            
            # First verify we can read the state
            if not self.read_canute_state():
                print("! Cannot access Canute state file - check connection")
                return False
                
            # Upload the new file
            with open(filename, 'rb') as file:
                file_data = file.read()
                print(f"Uploading file ({len(file_data)} bytes)...")
                
                headers = {
                    'Content-Type': 'application/octet-stream',
                    'Cache-Control': 'no-cache',
                    'Pragma': 'no-cache'
                }
                
                response = requests.put(upload_url, data=file_data, headers=headers)
            
            if response.status_code in [200, 201]:
                print("✓ Upload successful")
                
                # Force refresh
                print("Sending refresh commands...")
                self.force_refresh()
                time.sleep(2)
                
                # Try to trigger reload through state manipulation
                print("Attempting to trigger reload...")
                if self.trigger_reload(filename):
                    print("✓ Reload triggered")
                else:
                    print("! Could not trigger reload")
                
                # Verify file
                try:
                    verify = requests.get(upload_url)
                    if verify.status_code == 200 and len(verify.content) == len(file_data):
                        print("✓ File verified on FlashAir")
                        return True
                except:
                    pass
                    
            print("! Upload verification failed")
            return False
                
        except Exception as e:
            print(f"! Error during upload: {e}")
            return False

def main():
    uploader = CanuteFlashAirUploader()
    filename = 'circle.brf'  # Update this to match your file
    uploader.upload_file(filename)

if __name__ == "__main__":
    main()