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