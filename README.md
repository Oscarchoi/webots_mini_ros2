# webots_mini_ros2

## Prerequisite

For Ubuntu 20.04 or earlier, use **Webots R2023b** to ensure compatibility. Since this version is not available through Snap channels, you need to install it manually.

### 1. Uninstall existing Webots version (if any)

```bash
sudo snap remove webots
```

### 2. Install Webots R2023b manually

You can download the Snap package directly using:

```bash
wget https://github.com/cyberbotics/webots/releases/download/R2023b/webots_R2023b_amd64.snap
```

Since this is a local Snap package, you need to install it using the --dangerous flag:

```bash
sudo snap install webots_R2023b_amd64.snap --dangerous
```

### 3. Prevent auto-updates (Optional)

To avoid accidental upgrades via Snap:

```bash
sudo snap refresh webots --hold
```

Now, Webots R2023b should be installed and will remain at this version.

## How to start

```bash
ros2 launch webots_mini_ros2 webots_mini_launch.py
```
