| Supported Targets | STM32L496G-DISCO |
| ----------------- | ----- -----------| 

# B-CMAS-OMV in STM32L496G-DISCO

This project shows how to use the DCMI interface on the STM32L496G to display the image captured by the **ov5640** sensor on the board's LCD.

## How to use example

### Hardware Required

* STM32L496G-DISCO  <https://www.st.com/en/evaluation-tools/32l496gdiscovery.html>

* B-CAMS-OVM  <https://www.st.com/en/evaluation-tools/b-cams-omv.html>

### Configure the project

using **STMCubeProgrammer** load the [camera.elf](/bin/Camera.elf) file into the folder.

### Test

Use the **JOYSTICK** to enable and disable the camera and LCD.

* **JOY_LEFT:** Camera resume

* **JOY_UP:** LCD on

* **JOY_RIGHT:** Camera pause

* **JOY_DOWN:** LCD off

* **JOY_SEL:** LCD on and camera resume

## Example Output

![Output serial](/img/serial_monitor.PNG)

![Example](/img/example.jpg)

