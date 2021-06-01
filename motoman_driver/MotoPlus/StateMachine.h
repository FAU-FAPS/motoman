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

#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#if (DX100 || DX200 || FS100)
#define SMACH_LOCK_TIMEOUT 1000
#else
#define SMACH_LOCK_TIMEOUT 5000  // YRC1000 tick period is 0.2 ms
#endif

typedef enum
{
	SMACH_EVENT_UNDEFINED = 0,
	SMACH_EVENT_CLIENT_CONNECTED,
	SMACH_EVENT_CLIENT_DISCONNECTED,
	SMACH_EVENT_JOINT_TRAJECTORY_START,
	SMACH_EVENT_VELOCITY_CONFIG
} SmachEvent;

typedef enum
{
	MOTION_CTRL_MODE_IDLE = 0,
	MOTION_CTRL_MODE_JOINT_TRAJECTORY_STREAMING,
	MOTION_CTRL_MODE_VELOCITY_CONTROL
} MotionCtrlMode;

struct _StateMachine
{
	SEM_ID lock;
	MSG_Q_ID eventQueue;
	MotionCtrlMode motionCtrlMode;
};

// following typedef is in Controller.h
// typedef struct _StateMachine StateMachine; 

void StateMachine_Init(Controller* ctrl);
void StateMachine_Loop(Controller* ctrl);
BOOL StateMachine_SendEvent(Controller* ctrl, SmachEvent evt);

#endif