# Current Process for Updating Canute:

1. Convert ```square.jpg``` into ```square.brf``` using the ```convert.py``` script
2. Convert ```circle.jpg``` into ```circle.brf``` using the ```convert.py``` script after changing the example usage section
3. Insert the FlashAir SD card into the Canute, it will reboot
4. Using the ```send_file.py``` script, send the ```square.brf``` file as the intial file we want to update, Canute reboot is necessary
5. Load the file on the Canute by selecting it from the library menu
6. Keep a backup of ```square.brf``` and rename ```circle.brf``` into ```square.brf```
6. Using the ```send_file.py``` script run the same command to send the updated contents via the same file name
7. In order to see updates, the Canute must be rebooted or ejecting and re-inserting the SD card.
8. Selecting the same ```square.brf``` file in the Canute will show the updated contents and should now be a circle.

# 1/14 Meeting
- Email Bristoll Braille ✓

- Utilize two files A and B, update one at a time, see if that makes a difference
    - This works perfectly fine. Automatically editing the status file to change what file we load on reboot allows us to use a two file setup. But is essentially the same as doing a single file and overwriting and rebooting. Again, even changing the status file while the Canute is booted, nothing happens until after the reboot.

- New firmware maybe allows for API access?

- Also try writing to the canute_state.txt to manually change the filename to change the location of what file is being read.
    - Reading the status file again only happens on reboot.

- Utilize multiple pages, append changes to new lines and attempt to reload in real time.
    - New pages would only be noticed on reboot again.

# 1/28 Meeting
- Develop an app with the Jetson to create a local server that displays the braille translated point cloud data
    - Will need to use a router
    - Keep the original aspect ratio and resolution of the Canute for the vitual display
    - Live updates, refresh or use an embeded application hosted via the Jetson to refresh in real-time

# 3/11 Meeting
- Develop a dynamic refresh based on how much data actually needs to be refreshed
- Tunnel vision theory:
    - Middle of Canute doesnt need to be refreshed as frequently as edges
    - Trees next to car are faster versus the road ahead
