# BRLTTY Build Notes on Jetson Orin

## Initial Jetson Setup/Optimizations

### [Add User to Docker Group](https://github.com/dusty-nv/jetson-containers/blob/master/docs/setup.md#adding-user-to-docker-group)
Add the current user to the Docker group if necessary:

```bash
sudo usermod -aG docker $USER
```

### [Configure Docker Default Runtime](https://github.com/dusty-nv/jetson-containers/blob/master/docs/setup.md#docker-default-runtime)
Using Vim or nano, make the following change to the ```/etc/docker/daemon.json``` file:

```json
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },

    "default-runtime": "nvidia"
}
```

Then restart the Docker service or reboot:

```bash
sudo systemctl restart docker
```

Confirm the change with the following grep command:

```bash
sudo docker info | grep 'Default Runtime'
```

### [Increasing Swap Size w/ NVME SSD](https://github.com/dusty-nv/jetson-containers/blob/master/docs/setup.md#mounting-swap)
We use the following commands to increase the swap size on the Jetson and increase the amount of memory that the Jetson can utilize:

```bash
# disable ZRAM:
sudo systemctl disable nvzramconfig

# configure and create swap file:
sudo fallocate -l 16G /mnt/16GB.swap
sudo mkswap /mnt/16GB.swap
sudo swapon /mnt/16GB.swap
```

To make the swap file persistent, add the following line to the end of ```/etc/fstab``` using Vim or nano with sudo:

```bash
/mnt/16GB.swap  none  swap  sw 0  0
```

Reboot. Use the following command to check current sources for swap space:

```bash
swapon --show
```

Use the following command to view current memory usage:

```bash
free -m
```

### [Changing Power Mode](https://github.com/dusty-nv/jetson-containers/blob/master/docs/setup.md#setting-the-power-mode)
We change the power mode to MAXN for optimum performance available:

```bash
# check current power mode already set
sudo nvpmodel -q

# example:
NV Power Mode: 25W
1

# set mode to highest performance (MAXN SUPER)
sudo nvpmodel -m 2
```

Reboot to confirm change is persistent.

## Switch to bash Instead of dash

```bash
# Check current binding:
ls -l /bin/sh

# Re-configure and select 'No' when prompted:
sudo dpkg-reconfigure dash
```

## Downloading and Builidng BRLTTY
For now we dont install, just build the binaries. Installing and then rebooting has caused issues with getting into GDM (Ubuntu). This forces you to use a ```tty``` instance that does not give you any admin privliges. The ```install``` argument to the ```make``` command does create X11 and GDM startup scripts that seem to be the culprits for the lack of GUI access after doing this. Might be able to get into a X session if you were able to properly use ```sudo```, but as for the Tegra firmware, it is not allowed.

```bash
cd ~
git clone https://github.com/brltty/brltty.git
cd brltty/
git checkout master
sudo ./autogen
sudo Tools/reqpkgs -i
sudo ./configure
sudo make

# Add user to brlapi group
sudo usermod -aG brlapi $USER
```

## Configure BRLTTY
Uncomment the following lines inside of the ```/etc/brltty.conf``` file:

```bash

# BrlAPI Braille Driver Parameters
braille-parameters ba:Auth=/etc/brlapi.key
braille-parameters ba:Host=:0

# AtSpi2 Screen Driver Parameters
screen-parameters a2:Release=yes # [yes,no]
screen-parameters a2:Type=default # [default,all,{terminal,text}+...]
```

## Displaying Terminal Text on Canute

```bash
cd img_2_braille
chmod +x start_canute.sh
./start_canute
```

At this point a brltty process will initiate. If the terminal used to start the program is killed, brltty is still running in the background. Use the following to kill the process:

```bash
pkill brltty
```

