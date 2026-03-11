---
layout: default
title: Home
permalink: /
---

## Introduction

I'm a Computer Science graduate student at Boston University and NYU CS alumnus.

***

### llm-bench

*Python, Ollama API* &nbsp;·&nbsp; [GitHub](https://github.com/alessandrolandi/llm-bench)

A CLI tool for benchmarking local LLM inference. Measures tokens/sec, time-to-first-token, and prompt processing speed across different models and hardware configurations.

- Warmup runs and multi-run averaging ensure reproducible results
- Exports to JSON and CSV for cross-hardware comparison and analysis

***

### Autonomous Room Mapping Drone

*Python, OpenCV, Radio Communication*

Built an autonomous drone system for indoor room mapping using a monocular camera and radio communication. Implemented 3D room reconstruction from a live video feed using computer vision and depth estimation, along with a pathfinding algorithm for autonomous navigation and obstacle avoidance.

***

### GPU-Accelerated DCT Implementations for JPEG Compression

*CUDA, C*

Implemented and benchmarked four Discrete Cosine Transform (DCT) algorithms for JPEG compression on an NVIDIA V100 GPU, analyzing the performance tradeoffs of each approach. Work was benchmarked on a Red Hat OpenShift GPU cluster, with infrastructure findings shared directly with Red Hat engineers.


***

### 910x Turning Point Worlds Robot

*C++, PROS* &nbsp;·&nbsp; [GitHub](https://github.com/alessandrolandi/910x-vrc-tp-worlds)

<img src="assets/img/910Auton.gif" width="100%">

Turning Point was a VRC robotics season where teams built robots to target flags, flip caps, and park on platforms. Our robot used a single-flywheel launcher and a dual-function roller mechanism that could both collect balls and flip caps.

Each match consisted of an autonomous period (where robots acted entirely on pre-programmed instructions) and a driver-controlled period. 

For the drivetrain, we used an S-curve profile rather than a linear voltage ramp. Abrupt voltage changes caused the wheels to break traction; sinusoidal shaping smoothed both the acceleration and deceleration phases, giving the robot more consistent travel distances.

```c++
//abbreviated snippet of drive code
if(unitsTraveled < accelUnits)
    baseVoltage = (maxSpeed - 10) * sin(1.5708 * unitsTraveled / accelUnits) + 10;
else if(unitsLeft < decelUnits)
    baseVoltage = (maxSpeed - 10) * sin(1.5708 * unitsLeft / decelUnits) + 10;
```

The cap scraper needed to hit precise angles repeatedly. A PID loop on the scraper motor gave us the closed-loop position control to do that reliably.

```c++
int PID(Pid * pid, double setPoint, double Sensor){
  // Calculate error
  pid->error = setPoint - Sensor;
  pid->errorTotal += pid->error;
  pid->errorLast = pid->error;

    // Find proportional term
  double pTerm = pid->kP * pid->error;
    // Find integral term
  double iTerm = pid->kI * pid->errorTotal;
    // Find derivative term
  double dTerm = pid->kD * (pid->error - pid->errorLast);
  if (pid->error < pid->errorZone) {
    pid->errorTotal += pid->error;
  } else {
    pid->errorTotal = 0;
  }
    // Compute output
  double power = pTerm + iTerm + dTerm;

    // return val
  return power;
}
```

Flywheel velocity control required a different approach. Standard PID accumulated integral error during spin-up, causing the wheel to overshoot and oscillate around the target velocity. A [Take Back Half (TBH)](https://wiki.purduesigbots.com/software/control-algorithms/take-back-half-tbh-controller) controller addressed this by halving the running total whenever the error changed sign, converging to the setpoint without windup.

```c++
void TBH() {
  fwError = targetVel - avgFlywheelEnc();
  output += gain * fwError;
  if(abs(fwError) / fwError != abs(previousFwError) / previousFwError) {
    if(firstCross) {
      output = 118;
      firstCross = false;
    }
    else {
      output = (tbh + output) / 2;
      tbh = output;
   }
  }
  output = output > 127 ? 127 : output;
  previousFwError = fwError;
  setFlywheel(output);
  pros::lcd::set_text(5, "front tbh engaged");
}
```

Full source code is on [GitHub.](https://github.com/alessandrolandi/910x-vrc-tp-worlds)

We won 4 of 5 regional tournaments, won the Florida State Championship, and finished as a Division Finalist at the World Championship.

<img src="assets/img/IMG_1151.gif" width="100%">
<img src="assets/img/IMG_1181.gif" width="100%">
