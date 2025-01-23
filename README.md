# devcontainer-esp-idf-clean

## Installation Guide

### Windows 10, Windows 11

1. Install [Docker Desktop](https://www.docker.com/products/docker-desktop/). Do not check *Enable integration with my default WSL distro* in settings.

2. Install [Remote Development](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack) extenstion in VS Code. Do not check *Execute In WSL* in DEV Container: settings

3. Install [Ubuntu 24.04 LTS](https://apps.microsoft.com/detail/9nz3klhxdjp5?hl=pl-pl&gl=PL)
> Note: kernel version must be higher than `5.10.60.1`, check by `uname -a` on Ubuntu or by `wsl --version` on CMD

4. Make Ubuntu default WSL in CMD: `wsl -l -v` and `wsl --set-default Ubuntu-24.04`

5. Install usbipd in CMD: `winget install usbipd`

6. In Ubuntu run commands:
- `sudo apt-get update && sudo apt-get -qqy upgrade`
- `sudo apt install linux-tools-virtual hwdata`
- `sudo update-alternatives --install /usr/local/bin/usbip usbip $(command -v ls /usr/lib/linux-tools/*/usbip | tail -n1) 20`
- `sudo udevadm control --reload`
- `sudo service udev restart`

7. Plug board to computer (on devkit use UART MicroUSB port).

8. In CMD run:
- `usbipd list`
    Look for BUSID with CP210x device and remember its `id-id`.

9. Open CMD as Administrator and run:
- `usbipd bind --busid id-id`

10. In user CMD run:
- `usbipd attach -w --busid id-id`

Now device should be listed as attached:
- `usbipd list`

> Note: more advanced instructions for steps 6-10 available [here](https://github.com/dorssel/usbipd-win/wiki/WSL-support)

9. Open folder in Dev Container:
- first use: `F1` + `Dev Containers: Rebuild Without Cache and Reopen in Container`
- further use: `F1` + `Dev Containers: Reopen in Container`

10. Wait for container to setup. Reload window after setup.

11. Finally run in terminal (although it is run at startup):
- `make setup`

12. See Makefile help by:
- `make help` or just by `make`

13. Steps that may help if USB Windows-WSL bridge does not work:
- If the device is not detected first run `lsusb` to see if it pops up.
- See if `/dev/ttyUSB0` pops up by: `ls /dev | grep USB`.
- You may retry `make setup` or rebuild the Dev Container.
- Rarely, you'll need to restart your computer (usually when doing this full instruction in one Windows session).
