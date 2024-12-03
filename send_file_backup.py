import requests
import time

def upload_and_refresh():
    # Hardcoded values
    file_name = 'test5_output.brf'
    flashair_ip = '192.168.0.100'
    upload_url = f'http://{flashair_ip}/{file_name}'
    
    try:
        # Upload file
        with open(file_name, 'rb') as file:
            response = requests.put(upload_url, data=file)
        
        if response.status_code in [200, 201]:
            print(f"Successfully uploaded {file_name}")
            
            # Try to force refresh using FlashAir's command system
            refresh_url = f'http://{flashair_ip}/command.cgi?op=refresh'
            requests.get(refresh_url)
            
            # Additional refresh attempts
            sync_url = f'http://{flashair_ip}/upload.cgi?WRITEPROTECT=OFF'
            requests.get(sync_url)
            
            print("Refresh commands sent")
            return True
        else:
            print(f"Upload failed with status code: {response.status_code}")
            return False
            
    except Exception as e:
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    upload_and_refresh()