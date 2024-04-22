# Speed control system PID STM32F103
The system inludes: sensor pressure RP-L, Motor with gearbox and encoder JGB37-520B 12V and Driver DRV8833.

# Pressure sensor RP-L
![image](https://github.com/Ngoc-Phu-Nguyen/Speed-control-system-PID-STM32F103/assets/167606858/90b0a555-34ee-4002-a115-f9aa3b8aa5d5)

Fig 1. Process of analyzing sensor

![image](https://github.com/Ngoc-Phu-Nguyen/Speed-control-system-PID-STM32F103/assets/167606858/e5059043-de73-4c75-a2da-323192986a01)

Fig 2. Characteristics of sensor (gram-Om)

![image](https://github.com/Ngoc-Phu-Nguyen/Speed-control-system-PID-STM32F103/assets/167606858/419ae8d1-b30e-4e95-8811-1ed52ad813dd)

Fig 3. Sensors circuit for MCU

![image](https://github.com/Ngoc-Phu-Nguyen/Speed-control-system-PID-STM32F103/assets/167606858/3278990f-a5d1-44e7-9f41-27335e84ec20)

Fig 4. Voltage output depends on R0

# Target - Reality speed 

![image](https://github.com/Ngoc-Phu-Nguyen/Speed-control-system-PID-STM32F103/assets/167606858/75d81b1f-717e-4536-896a-fad9a6f3bc27)
![image](https://github.com/Ngoc-Phu-Nguyen/Speed-control-system-PID-STM32F103/assets/167606858/59140a50-1702-42b8-8740-b1fb02b8e970)
![image](https://github.com/Ngoc-Phu-Nguyen/Speed-control-system-PID-STM32F103/assets/167606858/b8433459-0465-4601-9e9b-2cf74a35409c)

Fig 5. Signal output from encoder. Speed 930-730-300 rps. 

# PID controller

![image](https://github.com/Ngoc-Phu-Nguyen/Speed-control-system-PID-STM32F103/assets/167606858/e39fc099-b110-4133-8553-65ac58384b77)

Fig 6. Algorithm of PID controller

![image](https://github.com/Ngoc-Phu-Nguyen/Speed-control-system-PID-STM32F103/assets/167606858/4c8cbf7d-22ed-41c7-a2a4-421673206d2f)

Fig 7. Application for system - in this case, the y(t) is the speeds feedback from encoder (current speed). Accordingly, we have the target speed r(t). Hence, we can get the error e(t) = r(t) - y(t). Using PID, we can calculate the control value u(t) in order to keep the system run steadily.  

# Result 

![image](https://github.com/Ngoc-Phu-Nguyen/Speed-control-system-PID-STM32F103/assets/167606858/9162eea6-59e2-4c87-9f31-b8154d1bf17e)

Fig 8. The speed is kept stable while the load is being changed. 
