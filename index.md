---
layout: default
title: Home
permalink: /
---

## Introduction

Hello and welcome to my projects showcase! My name is Alessandro Landi and I'm an NYU Computer Science graduate and aspiring Software Engineer.

***

### 910x Turning Point World's Robot

<img src="assets/img/910Auton.gif" width="100%"> 

This robot was the culmination of my teammate and I's development during the VRC Turning Point season. The objective of the game was to build a robot capable of targetting flags, manipulating caps, and navigating field obstacles. We created a platform featuring a single flywheel paired with a roller and scraper combo to accomplish these objectives efficiently.

Each match consisted of two periods: an autonomous period, during which each team executed pre-programmed instructions to score as many points as possible, and a driver control period. 

To maximize our robot's performance during the autonomous period, we implemented:

an s-curve motion profile for the drive
```c++
//abbreviated snippet of drive code
if(unitsTraveled < accelUnits)
    baseVoltage = (maxSpeed - 10) * sin(1.5708 * unitsTraveled / accelUnits) + 10;
else if(unitsLeft < decelUnits)
    baseVoltage = (maxSpeed - 10) * sin(1.5708 * unitsLeft / decelUnits) + 10;
```

a PID controller for the scraper
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
and a modified I-controller called [TBH](https://wiki.purduesigbots.com/software/control-algorithms/take-back-half-tbh-controller) for the flywheel.

```c++
void tbh() {
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
Our full implementations along with the rest of the necessary source code can be found in [this GitHub repository.](https://github.com/alessandrolandi/910x-vrc-tp-worlds)

Throughout the season, we achieved remarkable success, winning 4 out of 5 of the regional tournaments we attended, claiming victory at the Florida State Championship, and finishing as Division Finalists at the World Championship.

<img src="assets/img/IMG_1151.gif" width="100%"> 
<img src="assets/img/IMG_1181.gif" width="100%">