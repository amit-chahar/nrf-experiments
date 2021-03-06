# nrf-experiments
Some experiments with nrf51 board


## RPA generator
*RPA = Resolvable Private Address*<br>
This is the project to generate RPA from our own code without using softdevice functions.

**How to run this project**
 
In examples folder of NRF SDK, replace the main.c file in ble\_app\_template project with the given main.c file. That will make sure that the hex file is built according to up-to-date libraries.
 
If you get some error like this:<br>
*.\_build\nrf51422_xxac.axf: Error: L6218E: Undefined symbol SEGGER_RTT_printf (referred from main.o).*<br>
Add **NRF_LOG_USES_RTT=1**  in pre-processor symbols as shown in the picture below. This symbol is for segger_RTT.

![](images/rpa_generation_err.PNG?raw=true "Error Screenshot")

**Functions explanation**<br>
There are two functions that can be used for RPA generation:

**1. RPA\_generation\_and\_set():** This function generates RPA without using softdevice functions and  according to BLE specifications.


**2. RPA_config():** This is an alternative function and it uses softdevice functions to set the RPA but the IRK(Identity Resolving key) and RPA refresh interval is given by us.
