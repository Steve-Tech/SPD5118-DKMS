# SPD-5118 DKMS

This is René's driver for the SPD-5118 DDR5 Temperature sensor, with a DKMS config added.

The original patches can be downloaded from [here](https://t2sde.org/packages/linux), if it works consider donating to the author: [Patreon](https://www.patreon.com/renerebe), [GitHub Sponsors](https://github.com/sponsors/rxrbln/).

The [it87 Makefile](https://github.com/a1wong/it87/blob/master/Makefile) was also used as an example.

## Installation

```sh
sudo make dkms
sudo modprobe spd5118
```

## Uninstallation

```sh
sudo modprobe -r spd5118
sudo make dkms_clean
```
