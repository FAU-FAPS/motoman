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

#include "MotoROS.h"

// names for MotionCtrlMode enum
static const char * const MOTION_CTRL_MODE_NAMES[] = {
	"IDLE",
	"JOINT_TRAJECTORY_STREAMING",
	"VELOCITY_CONTROL"
};

// forward declarations
void OnClientConnected(Controller* ctrl);
void OnClientDisconnected(Controller* ctrl);
void OnJointTrajPt(Controller* ctrl);
void OnVelocityConfig(Controller* ctrl);
void StartJointTrajectoryStreaming(Controller* ctrl);
void StartVelocityControl(Controller* ctrl);
BOOL StartMotionControlTask(Controller* ctrl, FUNCPTR func, const char* taskName);
void StopMotionControlTask(Controller* ctrl);
void SetMotionCtrlMode(Controller* ctrl, MotionCtrlMode mode);
void LogError(const char* format, ...);
void LogInfo(const char* format, ...);

// motion control tasks
void Ros_MotionServer_IncMoveLoopStart(Controller* controller);
void Ros_MotionServer_VelocityControlLoop(Controller* controller);

void StateMachine_Init(Controller* ctrl)
{
	ctrl->stateMachine = (StateMachine*)mpMalloc(sizeof(StateMachine));
	ctrl->stateMachine->lock = mpSemBCreate(SEM_Q_FIFO, SEM_FULL);
	ctrl->stateMachine->eventQueue = mpMsgQCreate(10, sizeof(int), MSG_Q_FIFO);
	ctrl->stateMachine->motionCtrlMode = MOTION_CTRL_MODE_IDLE; // initial state
}

void StateMachine_Loop(Controller* ctrl)
{
	int ret;
	int evt;

	FOREVER
	{
		// wait for "event"
		ret = mpMsgQReceive(ctrl->stateMachine->eventQueue, (char*)&evt, sizeof(int), WAIT_FOREVER);

		if (mpSemTake(ctrl->stateMachine->lock, SMACH_LOCK_TIMEOUT) == OK)
		{
			if (ret == sizeof(int))
			{
				// process event
				switch (evt)
				{
				case SMACH_EVENT_UNDEFINED:
					LogError("Received UNDEFINED event"); // possible bug
					break;

				case SMACH_EVENT_CLIENT_CONNECTED:
					OnClientConnected(ctrl);
					break;

				case SMACH_EVENT_CLIENT_DISCONNECTED:
					OnClientDisconnected(ctrl);
					break;

				case SMACH_EVENT_VELOCITY_CONFIG:
					OnVelocityConfig(ctrl);
					break;

				case SMACH_EVENT_JOINT_TRAJECTORY_START:
					OnJointTrajPt(ctrl);
					break;

				default:
					LogError("Received unknown event: %d", evt); // possible bug
					break;
				}
			}
			else
			{
				if (ret == ERROR)
				{
					const char* errDesc;
					switch (mpErrMsgQRcv())
					{
					case E_TIME_OUT:
						errDesc = "Timeout";
						break;
					case E_NONEXIST:
						errDesc = "The message queue was deleted";
						break;
					case E_OBJECTS_ID:
						errDesc = "Object ID error";
						break;
					default:
						errDesc = "Unknown error";
						break;
					}
					LogError("Failed to receive event: %s", errDesc);
				}
				else
					LogError("Received %d bytes for event data. Expected %d bytes", ret, sizeof(int));
			}

			// unlock
			mpSemGive(ctrl->stateMachine->lock);

			// throttle loop on error
			if (ret != sizeof(int))
				Ros_Sleep(1000);
		}
		else
		{
			LogError("Failed to process event. The state machine is locked up");
		}
	}
}

BOOL StateMachine_SendEvent(Controller* ctrl, SmachEvent evt)
{
	int evtValue = (int)evt;
	int ret = mpMsgQSend(ctrl->stateMachine->eventQueue, (char*)&evtValue, sizeof(int), NO_WAIT, MSG_PRI_NORMAL);

	if (ret == ERROR)
	{
		const char* errDesc;
		switch (mpErrMsgQSnd())
		{
		case E_TIME_OUT:
			errDesc = "Timeout";
			break;
		case E_NONEXIST:
			errDesc = "The message queue was deleted";
			break;
		case E_LIMIT_EXCEEDED:
			errDesc = "Limit exceeded";
			break;
		case E_OBJECTS_ID:
			errDesc = "Object ID error";
			break;
		default:
			errDesc = "Unknown error";
			break;
		}

		LogError("Failed to send state machine event: %s", errDesc);
		return FALSE;
	}

	return TRUE;
}

void OnClientConnected(Controller* ctrl)
{
	LogInfo("Received CLIENT_CONNECTED event");

	switch (ctrl->stateMachine->motionCtrlMode)
	{
	case MOTION_CTRL_MODE_IDLE:
		StartJointTrajectoryStreaming(ctrl);  // default
		break;

	case MOTION_CTRL_MODE_VELOCITY_CONTROL:
	case MOTION_CTRL_MODE_JOINT_TRAJECTORY_STREAMING:
		LogError("The state machine should be in IDLE state but the active motion control mode is: %s)",
			MOTION_CTRL_MODE_NAMES[ctrl->stateMachine->motionCtrlMode]);  // possible bug
		break;

	default:
		LogError("State machine in undefined state (motion control mode: %d)", ctrl->stateMachine->motionCtrlMode);  // possible bug
		break;
	}
}

void OnClientDisconnected(Controller* ctrl)
{
	LogInfo("Received CLIENT_DISCONNECTED event");

	switch (ctrl->stateMachine->motionCtrlMode)
	{
	case MOTION_CTRL_MODE_IDLE:
		LogError("The state machine should not be in IDLE state");  // possible bug
		break;

	case MOTION_CTRL_MODE_VELOCITY_CONTROL:
	case MOTION_CTRL_MODE_JOINT_TRAJECTORY_STREAMING:
		StopMotionControlTask(ctrl);
		break;

	default:
		LogError("State machine in undefined state (motion control mode: %d)", ctrl->stateMachine->motionCtrlMode);  // possible bug
		break;
	}
}

void OnJointTrajPt(Controller* ctrl)
{
	LogInfo("Received JOINT_TRAJECTORY_START event");

	switch (ctrl->stateMachine->motionCtrlMode)
	{
	case MOTION_CTRL_MODE_IDLE:
		StartJointTrajectoryStreaming(ctrl);
		break;

	case MOTION_CTRL_MODE_VELOCITY_CONTROL:
		StopMotionControlTask(ctrl);
		StartJointTrajectoryStreaming(ctrl);
		break;

	case MOTION_CTRL_MODE_JOINT_TRAJECTORY_STREAMING:
		LogInfo("Already in JOINT_TRAJECTORY_STREAMING mode. Nothing to do");
		break;

	default:
		LogError("State machine in undefined state (motion control mode: %d)", ctrl->stateMachine->motionCtrlMode);  // possible bug
		break;
	}
}

void OnVelocityConfig(Controller* ctrl)
{
	LogInfo("Received VELOCITY_CONFIG event");

	switch (ctrl->stateMachine->motionCtrlMode)
	{
	case MOTION_CTRL_MODE_IDLE:
		StartVelocityControl(ctrl);
		break;

	case MOTION_CTRL_MODE_JOINT_TRAJECTORY_STREAMING:
		StopMotionControlTask(ctrl);
		StartVelocityControl(ctrl);
		break;

	case MOTION_CTRL_MODE_VELOCITY_CONTROL:
		LogInfo("Already in VELOCITY_CONTROL mode. Nothing to do");
		break;

	default:
		LogError("State machine in undefined state (motion control mode: %d)", ctrl->stateMachine->motionCtrlMode);  // possible bug
		break;
	}
}

void StartJointTrajectoryStreaming(Controller* ctrl)
{
	LogInfo("Starting JOINT_TRAJECTORY_STREAMING");

	if (StartMotionControlTask(ctrl, (FUNCPTR)Ros_MotionServer_IncMoveLoopStart, "IncMoveLoopStart"))
	{
		SetMotionCtrlMode(ctrl, MOTION_CTRL_MODE_JOINT_TRAJECTORY_STREAMING);
	}
}

void StartVelocityControl(Controller* ctrl)
{
	LogInfo("Starting VELOCITY_CONTROL");

	if (StartMotionControlTask(ctrl, (FUNCPTR)Ros_MotionServer_VelocityControlLoop, "VelocityControlLoop"))
	{
		SetMotionCtrlMode(ctrl, MOTION_CTRL_MODE_VELOCITY_CONTROL);
	}
}

BOOL StartMotionControlTask(Controller* ctrl, FUNCPTR task, const char* taskName)
{
	// Start the motion control task only if there is no task running already
	if (ctrl->tidIncMoveThread == INVALID_TASK)
	{
		LogInfo("Creating new task: %s", taskName);

		ctrl->tidIncMoveThread = mpCreateTask(MP_PRI_IP_CLK_TAKE, MP_STACK_SIZE, task, (int)ctrl, 0, 0, 0, 0, 0, 0, 0, 0, 0);

		if (ctrl->tidIncMoveThread == ERROR)
		{
			LogError("Failed to create task. Check robot parameters.");
			ctrl->tidIncMoveThread = INVALID_TASK;
			Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
			mpSetAlarm(8004, "MOTOROS FAILED TO CREATE TASK", 4);
			return FALSE;
		}
	}
	else
	{
		LogError("Failed to start motion control task. There is already a motion task running");
		return FALSE;
	}

	return TRUE;
}

void StopMotionControlTask(Controller* ctrl)
{
	LogInfo("Stopping the active motion control task");
	int tid = ctrl->tidIncMoveThread;
	ctrl->tidIncMoveThread = INVALID_TASK;
	mpDeleteTask(tid);
	SetMotionCtrlMode(ctrl, MOTION_CTRL_MODE_IDLE);
}

void SetMotionCtrlMode(Controller* ctrl, MotionCtrlMode mode)
{
	MotionCtrlMode last_mode = ctrl->stateMachine->motionCtrlMode;
	ctrl->stateMachine->motionCtrlMode = mode;
	LogInfo("State transition: %s -> %s", MOTION_CTRL_MODE_NAMES[last_mode], MOTION_CTRL_MODE_NAMES[mode]);
}

void Log(const char* loglevel, const char* format, va_list args)
{
	char buf[1024];
	vsnprintf(buf, 1024, format, args);
	printf("[state_machine] [%s]: %s\r\n", loglevel, buf);
}

void LogError(const char* format, ...)
{
	va_list arglist;
	va_start(arglist, format);
	Log("ERROR", format, arglist);
	va_end(arglist);
}

void LogInfo(const char* format, ...)
{
	va_list arglist;
	va_start(arglist, format);
	Log("INFO ", format, arglist);
	va_end(arglist);
}
