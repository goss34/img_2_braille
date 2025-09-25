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