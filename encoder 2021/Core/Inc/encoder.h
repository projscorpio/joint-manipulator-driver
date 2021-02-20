/*
 * encoder.h
 *
 *  Created on: Feb 19, 2021
 *      Author: HP
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#pragma once

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "math.h"

#define ENC_MAX 4096
#define ENC_MIN 0

float getAngle(void);
void InitSPIRec(void);
#endif /* INC_ENCODER_H_ */
