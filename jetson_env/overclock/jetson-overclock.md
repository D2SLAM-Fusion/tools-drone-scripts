## Default MAXN

```
SOC family:tegra234  Machine:NVIDIA Orin NX Developer Kit
Online CPUs: 0-7
cpu0: Online=1 Governor=schedutil MinFreq=729600 MaxFreq=1984000 CurrentFreq=1804800 IdleStates: WFI=1 c7=1 
cpu1: Online=1 Governor=schedutil MinFreq=729600 MaxFreq=1984000 CurrentFreq=1984000 IdleStates: WFI=1 c7=1 
cpu2: Online=1 Governor=schedutil MinFreq=729600 MaxFreq=1984000 CurrentFreq=1984000 IdleStates: WFI=1 c7=1 
cpu3: Online=1 Governor=schedutil MinFreq=729600 MaxFreq=1984000 CurrentFreq=1984000 IdleStates: WFI=1 c7=1 
cpu4: Online=1 Governor=schedutil MinFreq=729600 MaxFreq=1984000 CurrentFreq=1984000 IdleStates: WFI=1 c7=1 
cpu5: Online=1 Governor=schedutil MinFreq=729600 MaxFreq=1984000 CurrentFreq=1984000 IdleStates: WFI=1 c7=1 
cpu6: Online=1 Governor=schedutil MinFreq=729600 MaxFreq=1984000 CurrentFreq=1984000 IdleStates: WFI=1 c7=1 
cpu7: Online=1 Governor=schedutil MinFreq=729600 MaxFreq=1984000 CurrentFreq=1984000 IdleStates: WFI=1 c7=1 
GPU MinFreq=306000000 MaxFreq=918000000 CurrentFreq=306000000
EMC MinFreq=204000000 MaxFreq=3199000000 CurrentFreq=2133000000 FreqOverride=0
DLA0_CORE:   Online=1 MinFreq=0 MaxFreq=614400000 CurrentFreq=614400000
DLA0_FALCON: Online=1 MinFreq=0 MaxFreq=294400000 CurrentFreq=294400000
DLA1_CORE:   Online=1 MinFreq=0 MaxFreq=614400000 CurrentFreq=614400000
DLA1_FALCON: Online=1 MinFreq=0 MaxFreq=294400000 CurrentFreq=294400000
PVA0_VPS0: Online=1 MinFreq=0 MaxFreq=704000000 CurrentFreq=704000000
PVA0_AXI:  Online=1 MinFreq=0 MaxFreq=486400000 CurrentFreq=486400000
FAN Dynamic Speed control=active hwmon2_pwm1=255
NV Power Mode: MAXN
```

single core benchmark

```
sysbench cpu --threads=1 run
```

```
CPU speed:
    events per second:  2360.31

General statistics:
    total time:                          10.0003s
    total number of events:              23608

Latency (ms):
         min:                                    0.42
         avg:                                    0.42
         max:                                    1.35
         95th percentile:                        0.42
         sum:                                 9995.97

Threads fairness:
    events (avg/stddev):           23608.0000/0.00
    execution time (avg/stddev):   9.9960/0.00
```

# OverClock(CPU: 2.3GHz & GPU: 1.3GHz)

```
SOC family:tegra234  Machine:NVIDIA Orin NX Developer Kit
Online CPUs: 0-7
cpu0: Online=1 Governor=schedutil MinFreq=2304000 MaxFreq=2304000 CurrentFreq=2304000 IdleStates: WFI=0 c7=0 
cpu1: Online=1 Governor=schedutil MinFreq=2304000 MaxFreq=2304000 CurrentFreq=2304000 IdleStates: WFI=0 c7=0 
cpu2: Online=1 Governor=schedutil MinFreq=2304000 MaxFreq=2304000 CurrentFreq=2304000 IdleStates: WFI=0 c7=0 
cpu3: Online=1 Governor=schedutil MinFreq=2304000 MaxFreq=2304000 CurrentFreq=2304000 IdleStates: WFI=0 c7=0 
cpu4: Online=1 Governor=schedutil MinFreq=2304000 MaxFreq=2304000 CurrentFreq=2304000 IdleStates: WFI=0 c7=0 
cpu5: Online=1 Governor=schedutil MinFreq=2304000 MaxFreq=2304000 CurrentFreq=2304000 IdleStates: WFI=0 c7=0 
cpu6: Online=1 Governor=schedutil MinFreq=2304000 MaxFreq=2304000 CurrentFreq=2304000 IdleStates: WFI=0 c7=0 
cpu7: Online=1 Governor=schedutil MinFreq=2304000 MaxFreq=2304000 CurrentFreq=2304000 IdleStates: WFI=0 c7=0 
GPU MinFreq=1287750000 MaxFreq=1287750000 CurrentFreq=1287750000
EMC MinFreq=204000000 MaxFreq=3199000000 CurrentFreq=3199000000 FreqOverride=1
DLA0_CORE:   Online=1 MinFreq=0 MaxFreq=614400000 CurrentFreq=614400000
DLA0_FALCON: Online=1 MinFreq=0 MaxFreq=294400000 CurrentFreq=294400000
DLA1_CORE:   Online=1 MinFreq=0 MaxFreq=614400000 CurrentFreq=614400000
DLA1_FALCON: Online=1 MinFreq=0 MaxFreq=294400000 CurrentFreq=294400000
PVA0_VPS0: Online=1 MinFreq=0 MaxFreq=704000000 CurrentFreq=704000000
PVA0_AXI:  Online=1 MinFreq=0 MaxFreq=486400000 CurrentFreq=486400000
FAN Dynamic Speed control=active hwmon2_pwm1=255
NV Power Mode: MAXN
```



## BPMP Frimware Configuration：

CPU Clock Parameter

```
clock@cluster0 {
			clk-id = <0x118>;
			max-rate-maxn = <0x00 0x94c5f000>;
		};

		clock@cluster1 {
			clk-id = <0x119>;
			max-rate-maxn = <0x00 0x94c5f000>;
		};

		clock@cluster2 {
			clk-id = <0x11a>;
			max-rate-maxn = <0x00 0x94c5f000>;
		};

```

GPU Clock Parameter

```
clock@gpc0 {
			clk-id = <0x1aa>;
			max-rate-maxn = <0x00 0x4d7c6d00>;
		};

		clock@gpc1 {
			clk-id = <0x1ab>;
			max-rate-maxn = <0x00 0x4d7c6d00>;
		};

		clock@gpusys {
			clk-id = <0x1ac>;
			max-rate-maxn = <0x00 0x4d7c6d00>;
		};
```

Regulators Parameter

```
regulators {

		vdd_aon {
			rail-id = <0x05>;
			dev = <0x01>;
		};

		vdd_cpu {
			rail-id = <0x00 0x02 0x03>;
			dev = <0x02>;
			regulator-ramp-delay-linear = <0xdac>;
			regulator-enable-ramp-delay = <0xbb8>;
			regulator-disable-ramp-delay = <0x4e20>;
			regulator-min-microvolt = <0xb2f48>;
			regulator-max-microvolt = <0x155cc0>;
		};

		vdd_core {
			rail-id = <0x01>;
			dev = <0x03>;
			regulator-ramp-delay-linear = <0xdac>;
			regulator-min-microvolt = <0xbb030>;
			regulator-max-microvolt = <0x155cc0>;
		};
	};
```

## Linux Kernel Device Tree

node: ina3221

bypass the power summation

```
ina3221@40 {
			compatible = "ti,ina3221";
			reg = <0x40>;
			#address-cells = <0x01>;
			#size-cells = <0x00>;
			#io-channel-cells = <0x01>;
			phandle = <0x31b>;

			channel@0 {
				reg = <0x00>;
				label = "VDD_IN";
				shunt-resistor-micro-ohms = <0x1388>;
				summation-bypass;
			};

			channel@1 {
				reg = <0x01>;
				label = "VDD_CPU_GPU_CV";
				shunt-resistor-micro-ohms = <0x1388>;
				summation-bypass;
			};

			channel@2 {
				reg = <0x02>;
				label = "VDD_SOC";
				shunt-resistor-micro-ohms = <0x1388>;
				summation-bypass;
			};
		};
```

Linux PowerLimit Configuration

```
sudo echo 7000 > /sys/bus/i2c/drivers/ina3221/1-0040/hwmon/hwmon3/curr1_crit
sudo echo 6500 > /sys/bus/i2c/drivers/ina3221/1-0040/hwmon/hwmon3/curr1_max
```

**or** write a service file to run this at boot time

overclock.service

```
[Unit]
Description=OverClock
After=multi-user.target

[Service]
ExecStart= /bin/bash -c "echo 7000 > /sys/bus/i2c/drivers/ina3221/1-0040/hwmon/hwmon3/curr1_crit && echo 6500 > /sys/bus/i2c/drivers/ina3221/1-0040/hwmon/hwmon3/curr1_max"

[Install]
WantedBy=multi-user.target
```

```
sudo cp overclock.service /etc/systemd/system/overclock.service
sudo systemctl enable overclock.service
sudo systemctl start overclock.service
```



# Test

GPU：

```
 /usr/src/tensorrt/bin/trtexec --onnx=./crestereo_combined_iter2_240x320.onnx --loadEngine=./crestereo_combined_iter2_240x320_onnx.trt --int8 --fp16  --iterations=100 --streams=4 --useSpinWait
```



## Single core benchmark

```
sysbench cpu --threads=1 run
```

```
CPU speed:
    events per second:  2744.10

General statistics:
    total time:                          10.0003s
    total number of events:              27445

Latency (ms):
         min:                                    0.36
         avg:                                    0.36
         max:                                    0.41
         95th percentile:                        0.37
         sum:                                 9995.76

Threads fairness:
    events (avg/stddev):           27445.0000/0.00
    execution time (avg/stddev):   9.9958/0.00
```

SpeedUp: （27445 - 23608) / 23608 = 16.25%

## Burn CPU&GPU

![image-20230912202108867](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/Gallery/PicGo/202309122021916.png)

![image-20230912202324657](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/Gallery/PicGo/202309122023675.png)

## GeekBench6

登顶Jetson跑分榜

![image-20230913171938543](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/Gallery/PicGo/202309131719604.png)

## NVPmodel

Boost CPU 

```
CPU: 2.3GHz
GPU: 900MHz
```

Boost GPU

```
CPU: 1.8GHz
GPU: 1100MHz
```

