import requests
import time
import os
from datetime import datetime

class CanuteFlashAirUploader:
    def __init__(self):
        self.flashair_ip = '192.168.0.100'
        
    def check_web_interface(self, filename):
        """Verify file exists via web interface"""
        try:
            response = requests.get(f'http://{self.flashair_ip}/{filename}')
            if response.status_code == 200:
                print(f"✓ File visible on web interface")
                print(f"  Size: {len(response.content)} bytes")
                return True
            else:
                print("✗ File not found on web interface")
                return False
        except Exception as e:
            print(f"! Error checking web interface: {e}")
            return False
    
    def check_filesystem(self, mount_path, filename):
        """Check if file is visible in the OS filesystem"""
        full_path = os.path.join(mount_path, filename)
        if os.path.exists(full_path):
            print(f"✓ File visible in filesystem")
            print(f"  Size: {os.path.getsize(full_path)} bytes")
            return True
        else:
            print("✗ File not visible in filesystem")
            return False
    
    def upload_file(self, filename, mount_path=None):
        """Upload file and verify through both interfaces"""
        upload_url = f'http://{self.flashair_ip}/{filename}'
        
        try:
            print(f"\nStarting upload process at {datetime.now().strftime('%H:%M:%S')}")
            
            # Upload file
            with open(filename, 'rb') as file:
                file_data = file.read()
                file_size = len(file_data)
                print(f"Uploading file ({file_size} bytes)...")
                
                headers = {
                    'Content-Type': 'application/octet-stream',
                    'Content-Length': str(file_size)
                }
                response = requests.put(upload_url, data=file_data, headers=headers)
            
            if response.status_code in [200, 201]:
                print("✓ Upload request successful")
                
                # Wait briefly for processing
                time.sleep(2)
                
                # Check both interfaces
                print("\nVerifying file availability:")
                web_status = self.check_web_interface(filename)
                
                if mount_path:
                    fs_status = self.check_filesystem(mount_path, filename)
                    if not fs_status:
                        print("\nFile system may need refresh. Options:")
                        print("1. Eject and remount the SD card")
                        print("2. On macOS, try: diskutil unmount force /path/to/card")
                        print("   then: diskutil mount /path/to/card")
                
                # Send refresh commands
                refresh_commands = [
                    f'http://{self.flashair_ip}/command.cgi?op=refresh',
                    f'http://{self.flashair_ip}/upload.cgi?WRITEPROTECT=OFF',
                    f'http://{self.flashair_ip}/command.cgi?op=sync'
                ]
                
                print("\nSending refresh commands...")
                for cmd in refresh_commands:
                    try:
                        requests.get(cmd, timeout=1)
                    except requests.exceptions.RequestException:
                        continue
                
                return True
            else:
                print(f"! Upload failed with status code: {response.status_code}")
                return False
                
        except Exception as e:
            print(f"! Error during upload: {e}")
            return False

def main():
    uploader = CanuteFlashAirUploader()
    
    # Adjust these values for your setup
    filename = 'test5_output.brf'
    mount_path = '/Volumes/NO NAME'  # Update this to match your SD card's mount point
    
    uploader.upload_file(filename, mount_path)

if __name__ == "__main__":
    main()