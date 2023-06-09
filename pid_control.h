/*
 *  pcr-controller
 *
 *
 *  Copyright (C) 2013-2014 Bernhard Tittelbach <xro@realraum.at>
 *
 *  This code is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  any later version.
 *
 *  This code is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with pcr-controller. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PID_CONTROLLER_INCLUDE_H_
#define PID_CONTROLLER_INCLUDE_H_

// #include <stdio.h>
#include <stdint.h>

#define PID_SCALE 1024L
#define PID_DISABLED (1<<15)

void pid_setP(int16_t p);
void pid_setI(int16_t i);
void pid_setD(int16_t d);
void pid_printVars(void);
void pid_setTargetValue(int16_t v);
void pid_setLimits(int16_t min, int16_t max);
int pid_isEnabled(void);
int16_t pid_getTargetValue(void);
int16_t pid_calc(int16_t current_value);
// void pid_loadFromEEPROM(void);
// void pid_saveToEEPROM(void);

#endif
