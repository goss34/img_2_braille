# BRLTTY Build Notes on Jetson Orin

## Disabling TTY

```bash
cd /etc/systemd
sudo vim logind.conf

# Make the following changes:
# (You may have to uncomment these lines)

[Login]
NAutoVTs=0
ReserveVt=0
```

## Downloading and Builidng BRLTTY
The following is done after the ```tty``` consoles have been disabled. For now we dont install, just build the binaries. Installing and then rebooting has caused issues with getting into GDM (Ubuntu). This forces you to use a ```tty``` instance that does not give you any admin privliges. The ```install``` argument to the ```make``` command does create X11 and GDM startup scripts that seem to be the culprits for the lack of GUI access after doing this. Might be able to get into a X session if you were able to properly use ```sudo```, but as for the Tegra firmware, it is not allowed.

```bash
git clone https://github.com/brltty/brltty.git
cd brltty/
git checkout master
./autogen
sudo Tools/reqpkgs -i
./configure
make -s
```
