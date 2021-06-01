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

//-----------------------
// Function declarations
//-----------------------

// Extern functions
void VelocityControl_Init(VelocityControl* ctrl, CtrlGroup* ctrlGrp, UINT16 interpolPeriod_ms);
void VelocityControl_Reset(VelocityControl* ctrl);
void VelocityControl_SetFilterSize(VelocityControl* ctrl, int size);
BOOL VelocityControl_PushCommand(VelocityControl* ctrl, VelocityCommand* cmd);
void VelocityControl_PopCommand(VelocityControl* ctrl, VelocityCommand* cmd);
BOOL VelocityControl_ConvertToIncPosInfo(VelocityControl* ctrl, VelocityCommand* cmd, MP_GRP_POS_INFO* posInfo);

// Buffer functions
void VelocityControl_InitBuffer(VelocityCommandBuffer* buffer, int size);
void VelocityControl_ResizeBuffer(VelocityCommandBuffer* buffer, int newSize);
void VelocityControl_AddToBuffer(VelocityCommandBuffer* buffer, VelocityCommand* cmd);

// Helper function
double VelocityControl_GetMagnitude(float vec[3]);

// Command processing functions
void VelocityControl_FilterCommand(VelocityCommandBuffer* filter, VelocityCommand* in, VelocityCommand* out);
BOOL VelocityControl_EnforceLimits(VelocityControl* ctrl, VelocityCommand* in, VelocityCommand* out);
void VelocityControl_EnforceCartesianLimits(VelocityControl* ctrl, VelocityCommand* in, VelocityCommand* out);
void VelocityControl_EnforceJointLimits(VelocityControl* ctrl, VelocityCommand* in, VelocityCommand* out);

// Conversion functions (velocity command -> increment)
BOOL VelocityControl_ConvertToIncDataType(VelocityControl* ctrl, UCHAR* incDataType);
void VelocityControl_ConvertToCartInc(VelocityControl* ctrl, VelocityCommand* cmd, long cartInc[MP_GRP_AXES_NUM]);


//-------------------------
// Function implementation
//-------------------------

// Initialize the velocity control data
void VelocityControl_Init(VelocityControl* ctrl, CtrlGroup* ctrlGrp, UINT16 interpolPeriod_ms)
{
	ctrl->lock = mpSemBCreate(SEM_Q_FIFO, SEM_FULL);
	ctrl->ctrlGrp = ctrlGrp;
	ctrl->interpolPeriod = interpolPeriod_ms;
	ctrl->maxLinearVel = VEL_CTRL_MAX_LINEAR_VEL;
	ctrl->maxLinearDeltaVel = VEL_CTRL_MAX_LINEAR_ACCEL * ctrl->interpolPeriod * 1e-3;
	ctrl->maxAngularVel = VEL_CTRL_MAX_ANGULAR_VEL;
	ctrl->maxAngularDeltaVel = VEL_CTRL_MAX_ANGULAR_ACCEL * ctrl->interpolPeriod * 1e-3;
	ctrl->toolFileNum = 0;
	ctrl->userCoordNum = 0;
	ctrl->commandType = VELOCITY_CMD_TYPE_UNDEFINED;
	VelocityControl_InitBuffer(&ctrl->memory, VEL_CTRL_MEMORY_SIZE);
	VelocityControl_InitBuffer(&ctrl->filter, VEL_CTRL_FILTER_SIZE);
	memset(&ctrl->rawCommand, 0x00, sizeof(VelocityCommand));
	memset(&ctrl->filtCommand, 0x00, sizeof(VelocityCommand));
	memset(&ctrl->actualCommand, 0x00, sizeof(VelocityCommand));
	ctrl->isStopped = FALSE;
}

// Reset the command data and buffers
void VelocityControl_Reset(VelocityControl* ctrl)
{
	memset(ctrl->memory.commands, 0x00, ctrl->memory.size * sizeof(VelocityCommand));
	memset(ctrl->filter.commands, 0x00, ctrl->filter.size * sizeof(VelocityCommand));
	memset(&ctrl->rawCommand, 0x00, sizeof(VelocityCommand));
	memset(&ctrl->filtCommand, 0x00, sizeof(VelocityCommand));
	memset(&ctrl->actualCommand, 0x00, sizeof(VelocityCommand));
}

// Set size of moving average filter
void VelocityControl_SetFilterSize(VelocityControl* ctrl, int size)
{
	VelocityControl_ResizeBuffer(&ctrl->filter, size);
}

BOOL VelocityControl_PushCommand(VelocityControl* ctrl, VelocityCommand* cmd)
{
	// store as raw command
	memcpy(&ctrl->rawCommand, cmd, sizeof(VelocityCommand));

	// filter command: RAW_CMD->|FILTER|->FILT_CMD
	VelocityControl_FilterCommand(&ctrl->filter, &ctrl->rawCommand, &ctrl->filtCommand);

	// enforce limits according to the command type: FILT_CMD->|LIMITS|->ACTUAL_CMD
	if (!VelocityControl_EnforceLimits(ctrl, &ctrl->filtCommand, &ctrl->actualCommand))
	{
		VelocityControl_Reset(ctrl);
		return FALSE;
	}

	return TRUE;
}

void VelocityControl_PopCommand(VelocityControl* ctrl, VelocityCommand* cmd)
{
	// copy actual command into output variable
	memcpy(cmd, &ctrl->actualCommand, sizeof(VelocityCommand));

	// copy command into memory buffer (command history)
	VelocityControl_AddToBuffer(&ctrl->memory, &ctrl->actualCommand);

	// clear command (to avoid repeated execution if there are no new commands)
	memset(&ctrl->rawCommand, 0x00, sizeof(VelocityCommand));
	memset(&ctrl->filtCommand, 0x00, sizeof(VelocityCommand));
	memset(&ctrl->actualCommand, 0x00, sizeof(VelocityCommand));
}

// Convert velocity command into increment position info according to its type
BOOL VelocityControl_ConvertToIncPosInfo(VelocityControl* ctrl, VelocityCommand* cmd, MP_GRP_POS_INFO* posInfo)
{
	int i;
	UCHAR incDataType;

	if (!VelocityControl_ConvertToIncDataType(ctrl, &incDataType))
		return FALSE;

	if (incDataType == MP_INC_PULSE_DTYPE)
	{
		Ros_CtrlGroup_ConvertToMotoPos(ctrl->ctrlGrp, cmd->vector, posInfo->pos);
		for (i = 0; i < MP_GRP_AXES_NUM; i++)
		{
			posInfo->pos[i] *= ctrl->interpolPeriod * 1e-3;
		}
	}
	else if (incDataType == MP_INC_BF_DTYPE || incDataType == MP_INC_RF_DTYPE || incDataType == MP_INC_TF_DTYPE)
	{
		VelocityControl_ConvertToCartInc(ctrl, cmd, posInfo->pos);
	}
	else
	{
		printf("[velocity_control] Error: Increment data type not supported: %d\r\n", incDataType);
		return FALSE;
	}

	posInfo->pos_tag.data[2] = ctrl->toolFileNum;
	posInfo->pos_tag.data[3] = incDataType;
	posInfo->pos_tag.data[4] = ctrl->userCoordNum;

	return TRUE;
}

// Allocate memory and init buffer
void VelocityControl_InitBuffer(VelocityCommandBuffer* buffer, int size)
{
	buffer->size = size;
	buffer->index = 0;
	buffer->commands = (VelocityCommand*)mpMalloc(buffer->size * sizeof(VelocityCommand));
	memset(buffer->commands, 0x00, buffer->size * sizeof(VelocityCommand));
}

// Resize buffer (memory allocation if newSize >0 and not equal to the current size)
void VelocityControl_ResizeBuffer(VelocityCommandBuffer* buffer, int newSize)
{
	if (buffer->size == newSize)
		return;

	if (newSize <= 0)
	{
		// empty buffer
		mpFree(buffer->commands);
		buffer->commands = NULL;
		buffer->size = 0;
		buffer->index = 0;
	}
	else
	{
		mpFree(buffer->commands);
		VelocityControl_InitBuffer(buffer, newSize);
	}
}

// Add velocity command to buffer
void VelocityControl_AddToBuffer(VelocityCommandBuffer* buffer, VelocityCommand* cmd)
{
	memcpy(&buffer->commands[buffer->index], cmd, sizeof(VelocityCommand));

	// increment index of ring buffer
	buffer->index = (buffer->index + 1) % buffer->size;
}

// Compute magnitude of a 3D vector
double VelocityControl_GetMagnitude(float vec[3])
{
	return sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

// Filter command (compute moving average): IN->|FILTER|->OUT
void VelocityControl_FilterCommand(VelocityCommandBuffer* filter, VelocityCommand* in, VelocityCommand* out)
{
	int i, j;

	if (filter->commands != NULL && filter->size > 0)
	{
		// add input command to the filter
		VelocityControl_AddToBuffer(filter, in);

		// clear output command vector
		memset(out->vector, 0x00, sizeof(float) * ROS_MAX_JOINT);

		// take sequence number from input command
		out->sequence = in->sequence;

		// compute the average
		for (i = 0; i < ROS_MAX_JOINT; i++)
		{
			for (j = 0; j < filter->size; j++)
			{
				out->vector[i] += filter->commands[j].vector[i];
			}

			out->vector[i] /= filter->size;
		}
	}
	else
	{
		// filter disabled (copy input to output)
		memcpy(out, in, sizeof(VelocityCommand));
	}
}

// Enforce limits according to the command type: IN->|LIMITS|->OUT
BOOL VelocityControl_EnforceLimits(VelocityControl* ctrl, VelocityCommand* in, VelocityCommand* out)
{
	if (ctrl->commandType == VELOCITY_CMD_TYPE_JOINT)
	{
		VelocityControl_EnforceJointLimits(ctrl, in, out);
	}
	else if (ctrl->commandType == VELOCITY_CMD_TYPE_BASE_FRAME || ctrl->commandType == VELOCITY_CMD_TYPE_ROBOT_FRAME || ctrl->commandType == VELOCITY_CMD_TYPE_TOOL_FRAME)
	{
		VelocityControl_EnforceCartesianLimits(ctrl, in, out);
	}
	else if (ctrl->commandType == VELOCITY_CMD_TYPE_UNDEFINED)
	{
		printf("[velocity_control] Error: Failed to enforce limits. Undefined velocity command type\r\n");
		return FALSE;
	}
	else
	{
		printf("[velocity_control] Error: Failed to enforce limits. Unknown velocity command type: %d\r\n", ctrl->commandType);
		return FALSE;
	}

	return TRUE;
}

// Enforce cartesian velocity/acceleration limits: IN->|LIMITS|->OUT
void VelocityControl_EnforceCartesianLimits(VelocityControl* ctrl, VelocityCommand* in, VelocityCommand* out)
{
	int i;
	double scale;

	// copy input command into output command
	memcpy(out, in, sizeof(VelocityCommand));

	// compute absolute linear/angular velocity
	double linearVel = VelocityControl_GetMagnitude(out->vector);
	double angularVel = VelocityControl_GetMagnitude(&out->vector[3]);

	// scale linear velocity if necessary
	if (linearVel > ctrl->maxLinearVel)
	{
		scale = ctrl->maxLinearVel / linearVel;
		for (i = 0; i < 3; i++)
		{
			out->vector[i] *= scale;
		}
	}

	// scale angular velocity if necessary
	if (angularVel > ctrl->maxAngularVel)
	{
		scale = ctrl->maxAngularVel / angularVel;
		for (i = 3; i < 6; i++)
		{
			out->vector[i] *= scale;
		}
	}

	// compute resulting delta velocity
	float* prevCmdVec = ctrl->memory.commands[Q_OFFSET_IDX(ctrl->memory.index, -1, ctrl->memory.size)].vector;
	float deltaCmdVec[6];
	for (i = 0; i < 6; i++)
	{
		deltaCmdVec[i] = out->vector[i] - prevCmdVec[i];
	}

	// compute absolute linear/angular delta velocity
	double linearDeltaVel = VelocityControl_GetMagnitude(deltaCmdVec);
	double angularDeltaVel = VelocityControl_GetMagnitude(&deltaCmdVec[3]);

	// scale linear acceleration if necessary
	if (linearDeltaVel > ctrl->maxLinearDeltaVel)
	{
		scale = ctrl->maxLinearDeltaVel / linearDeltaVel;
		for (i = 0; i < 3; i++)
		{
			out->vector[i] = prevCmdVec[i] + deltaCmdVec[i] * scale;
		}
	}

	// scale angular acceleration if necessary
	if (angularDeltaVel > ctrl->maxAngularDeltaVel)
	{
		scale = ctrl->maxAngularDeltaVel / angularDeltaVel;
		for (i = 3; i < 6; i++)
		{
			out->vector[i] = prevCmdVec[i] + deltaCmdVec[i] * scale;
		}
	}
}

// Enforce joint velocity limits: IN->|LIMITS|->OUT
void VelocityControl_EnforceJointLimits(VelocityControl* ctrl, VelocityCommand* in, VelocityCommand* out)
{
	int i;
	double scale = 1.0;                                 // scaling factor for all axes
	int maxAxes = min(ROS_MAX_JOINT, MP_GRP_AXES_NUM);  // maximum count of (controlled) axes
	float deltaPos[MP_GRP_AXES_NUM];                    // delta position in radians
	long deltaPulsePos[MP_GRP_AXES_NUM];                // delta pulse position

	// copy input command into output command
	memcpy(out, in, sizeof(VelocityCommand));

	//=============================
	// enforce velocity limits
	//=============================

	// determine scale factor to satisfy velocity limits for each axis
	for (i = 0; i < maxAxes; i++)
	{
		// prioritize the maximum velocity parameter stored in the velocity control data
		// but ensure that the maximum velocity for each axis does not exceed its hard limit
		float maxVel = min(ctrl->maxAngularVel, ctrl->ctrlGrp->maxSpeed[i]);

		if (fabs(out->vector[i]) > maxVel)
		{
			scale = min(scale, maxVel / fabs(out->vector[i]));
		}
	}

	// finally apply the actual scale on each axis
	for (i = 0; i < maxAxes; i++)
	{
		out->vector[i] *= scale;
	}

	//=============================
	// enforce acceleration limits
	//=============================

	scale = 1.0;

	// compute resulting delta velocity and determine scale factor to satisfy acceleration limits (max delta velocity) for each axis
	float* prevCmdVec = ctrl->memory.commands[Q_OFFSET_IDX(ctrl->memory.index, -1, ctrl->memory.size)].vector;
	float deltaCmdVec[ROS_MAX_JOINT];

	for (i = 0; i < maxAxes; i++)
	{
		deltaCmdVec[i] = out->vector[i] - prevCmdVec[i];

		if (fabs(deltaCmdVec[i]) > ctrl->maxAngularDeltaVel)
		{
			scale = min(scale, ctrl->maxAngularDeltaVel / fabs(deltaCmdVec[i]));
		}
	}

	// finally apply the actual scale on each axis
	for (i = 0; i < maxAxes; i++)
	{
		out->vector[i] = prevCmdVec[i] + deltaCmdVec[i] * scale;
	}

	//=============================
	// enforce position limits
	//=============================

	// fetch current (commanded) position
	if (Ros_CtrlGroup_GetPulsePosCmd(ctrl->ctrlGrp, ctrl->ctrlGrp->prevPulsePos))
	{
		// convert velocity command into delta position
		for (i = 0; i < maxAxes; i++)
		{
			deltaPos[i] = out->vector[i] * ctrl->interpolPeriod * 1e-3;
		}

		// convert delta position into delta pulse position
		Ros_CtrlGroup_ConvertToMotoPos(ctrl->ctrlGrp, deltaPos, deltaPulsePos);

		// check if pulse limits are satisfied
		for (i = 0; i < maxAxes; i++)
		{
			// multiply delta pulse position with a small factor to be able to stop in time
			if (ctrl->ctrlGrp->prevPulsePos[i] + deltaPulsePos[i] * 3 < ctrl->ctrlGrp->jointPulseLimits.minLimit[i] ||
				ctrl->ctrlGrp->prevPulsePos[i] + deltaPulsePos[i] * 3 > ctrl->ctrlGrp->jointPulseLimits.maxLimit[i])
			{
				// exceeding limit -> stop motion
				memset(out->vector, 0x00, sizeof(float) * ROS_MAX_JOINT);
				
				// print throttled warning message
				static int counter = 0;
				if (counter == 0)
				{
					printf("[velocity_control] Warning: Exceeding pulse limit in joint[%d] (motoman joint order)! Stopping Motion!\r\n", i);
					counter = (counter + 1) % (1000 / ctrl->interpolPeriod);
				}
				
				return;
			}
		}
	}
	else
	{
		printf("[velocity_control] Error: Failed to get commanded position. Skipped checking position limits");
	}
}

// Convert velocity command type into increment data type
BOOL VelocityControl_ConvertToIncDataType(VelocityControl* ctrl, UCHAR* incDataType)
{
	BOOL ret = TRUE;

	switch (ctrl->commandType)
	{
	case VELOCITY_CMD_TYPE_UNDEFINED:
		printf("[velocity_control] Error: Failed to convert into increment data type. Velocity command type is undefined\r\n");
		ret = FALSE;
		break;
	case VELOCITY_CMD_TYPE_JOINT:
		*incDataType = MP_INC_PULSE_DTYPE;
		break;
	case VELOCITY_CMD_TYPE_BASE_FRAME:
		*incDataType = MP_INC_BF_DTYPE;
		break;
	case VELOCITY_CMD_TYPE_ROBOT_FRAME:
		*incDataType = MP_INC_RF_DTYPE;
		break;
	case VELOCITY_CMD_TYPE_TOOL_FRAME:
		*incDataType = MP_INC_TF_DTYPE;
		break;
	default:
		printf("[velocity_control] Error: Failed to convert into increment data type. Unknown velocity command type: %d\r\n", ctrl->commandType);
		ret = FALSE;
		break;
	}

	return ret;
}

// Convert cartesian velocity command into cartesian increments
void VelocityControl_ConvertToCartInc(VelocityControl* ctrl, VelocityCommand* cmd, long cartInc[MP_GRP_AXES_NUM])
{
	double conversion;

	// clear output array
	memset(cartInc, 0x00, sizeof(long) * MP_GRP_AXES_NUM);

	// [x, y, z] in m/s -> increment per interpolation period expressed in micrometers
	conversion = METER_TO_INC * ctrl->interpolPeriod * 1e-3;
	cartInc[0] = (long)(cmd->vector[0] * conversion);
	cartInc[1] = (long)(cmd->vector[1] * conversion);
	cartInc[2] = (long)(cmd->vector[2] * conversion);

	// [rx, ry, rz] in rad/s -> increment per interpolation period expressed in 0.0001 degrees
	conversion = RAD_TO_INC * ctrl->interpolPeriod * 1e-3;
	cartInc[3] = (long)(cmd->vector[3] * conversion);
	cartInc[4] = (long)(cmd->vector[4] * conversion);
	cartInc[5] = (long)(cmd->vector[5] * conversion);
}

