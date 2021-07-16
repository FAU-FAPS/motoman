/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, Institute for Factory Automation and Production Systems (FAPS)
 * All rights reserved.
 *
 * Redistribution and use in binary form, with or without modification,
 * is permitted provided that the following conditions are met:
 *
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Yaskawa America, Inc., nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VELOCITYCONTROL_H
#define VELOCITYCONTROL_H

// Conversion factors
#define DEG_TO_RAD   0.017453292519943295
#define RAD_TO_DEG   57.29577951308232
#define DEG_TO_INC   1e4
#define RAD_TO_INC   (RAD_TO_DEG * DEG_TO_INC)
#define METER_TO_INC 1e6

// Default values
#define VEL_CTRL_FILTER_SIZE       0                  // Size of moving average filter
#define VEL_CTRL_MEMORY_SIZE       3                   // Size of memory buffer for storing the last executed velocity commands
#define VEL_CTRL_MAX_LINEAR_VEL    0.250               // Maximum linear velocity in m/s
#define VEL_CTRL_MAX_LINEAR_ACCEL  5.0                 // Maximum linear acceleration in m/s^2
#define VEL_CTRL_MAX_ANGULAR_VEL   (150 * DEG_TO_RAD)   // Maximum angular velocity in rad/s
#define VEL_CTRL_MAX_ANGULAR_ACCEL (300 * DEG_TO_RAD)  // Maximum angular acceleration in rad/s^2

// Special sequence number
#define VEL_CTRL_SPECIAL_SEQ_STOP_MOTION -1

// Lock timeout
#if (DX100 || DX200 || FS100)
#define VEL_CTRL_LOCK_TIMEOUT 1000
#else
#define VEL_CTRL_LOCK_TIMEOUT 5000  // YRC1000 tick period is 0.2 ms
#endif

typedef struct
{
	int sequence;
	float vector[ROS_MAX_JOINT];
} VelocityCommand;

typedef struct
{
	int size;
	int index;
	VelocityCommand* commands;
} VelocityCommandBuffer;

typedef struct
{
	SEM_ID lock;
	CtrlGroup* ctrlGrp;               // Pointer reference to associated control group
	UINT16 interpolPeriod;            // Interpolation period of the controller in milliseconds
	float maxLinearVel;               // Maximum linear velocity in m/s
	float maxLinearDeltaVel;          // Maximum linear delta velocity in m/s
	float maxAngularVel;              // Maximum angular velocity in rad/s
	float maxAngularDeltaVel;         // Maximum angular delta velocity in rad/s
	UCHAR toolFileNum;                // Tool file number (for MP_POS_TAG.data[2])
	UCHAR userCoordNum;               // User coordinate number (for MP_POS_TAG.data[4])
	MotoVelocityCmdType commandType;  // Command type (ANGLE / BASE_FRAME / ROBOT_FRAME / TOOL_FRAME)
	VelocityCommandBuffer memory;     // Buffer for executed commands (command history)
	VelocityCommandBuffer filter;     // Buffer for moving average filter
	VelocityCommand rawCommand;       // Raw command as received
	VelocityCommand filtCommand;      // Filtered command (output of the moving average filter)
	VelocityCommand actualCommand;    // Actual command to execute (after filtering and enforcing of pos/vel/accel limits)
	BOOL isStopped;                   // Flag if velocity control is stopped (e.g. after a StopMotion request)
} VelocityControl;

extern void VelocityControl_Init(VelocityControl* ctrl, CtrlGroup* ctrlGrp, UINT16 interpolPeriod_ms);
extern void VelocityControl_Reset(VelocityControl* ctrl);
extern void VelocityControl_SetFilterSize(VelocityControl* ctrl, int size);
extern BOOL VelocityControl_PushCommand(VelocityControl* ctrl, VelocityCommand* cmd);
extern void VelocityControl_PopCommand(VelocityControl* ctrl, VelocityCommand* cmd);
extern BOOL VelocityControl_ConvertToIncPosInfo(VelocityControl* ctrl, VelocityCommand* cmd, MP_GRP_POS_INFO* posInfo);

#endif
