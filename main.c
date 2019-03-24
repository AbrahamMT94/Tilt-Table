#include "derivative.h" /* include peripheral declarations */
#include "TFC\TFC.h"
#include "pid.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
PID* InitPID()
{
	PID*		pPID = malloc(sizeof(PID));
	
	if (pPID)
		memset(pPID, 0, sizeof(PID));
	
	pPID->startup = 1;
	
	return pPID;
}

void SetPIDSetpoint(PID* pPID, float setPt)
{
	if (! pPID)
		return;
	
	pPID->setpoint = setPt;
}

void SetPIDLimits(PID* pPID, float lwrLmt, float uprLmt)
{
	if (! pPID)
		return;
	
	pPID->lowerLimit = lwrLmt;
	pPID->upperLimit = uprLmt;
}

void SetPIDGain(PID* pPID, float p, float i, float d)
{
	if (! pPID)
		return;

	pPID->Kp = p;
	pPID->Ki = i;
	pPID->Kd = d;
}

float GetPIDOutput(PID* pPID, float input, float dTime)
{
	float		output = 0;
	float		error;
	
	if (! pPID)
		return 0;

	if (pPID->startup)
	{
		pPID->startup = 0;
		return pPID->setpoint;
	}
	
	error = pPID->setpoint - input;
	pPID->errSum += pPID->Ki*(error * dTime);
	
	if (pPID->errSum > pPID->upperLimit)
		pPID->errSum = pPID->upperLimit;
	else if (pPID->errSum < pPID->lowerLimit)
		pPID->errSum = pPID->lowerLimit;
	
	pPID->dError = fabs(dTime) < 0.00001 ? pPID->lastError : error/dTime - pPID->lastError;
	pPID->lastError = error;
	
	// Compute PID Output
	output = pPID->Kp*error + pPID->errSum + pPID->Kd*pPID->dError;
	
	// check limits
	if (output > pPID->upperLimit)
		output = pPID->upperLimit;
	else if (output < pPID->lowerLimit)
		output = pPID->lowerLimit;
	
	return output;
}



int main(void)
    {	
	//PID declaration and set up
	PID* pid1=InitPID();
	PID* pid2=InitPID();
	SetPIDSetpoint(pid1,-.6);
	SetPIDSetpoint(pid2,0);
	TFC_Init();	
	SetPIDLimits(pid1,-.7,.7);//y
	SetPIDLimits(pid2,-.7,.7);//x
	SetPIDGain(pid1,4,0.00000005,3.8);
	SetPIDGain(pid2,4,0.00000005,3.8);

	float X_pos, Y_pos, X_servo, Y_servo;
	for(;;)
	{	   
		        //TFC_Task must be called in your main loop.  This keeps certain processing happy (I.E. Serial port queue check)
			    TFC_Task();				    
				if(TFC_Ticker[0]==20) {
					 TFC_Ticker[0] = 0; //reset the Ticker			
					 //Every 20 mSeconds, update the Servos
				     X_pos = Get_X();			     			
			         Y_pos = Get_Y();
			     		    
//********************************* PID controller code ***************				    
			         Y_servo = GetPIDOutput(pid1, Y_pos, 20);
			         X_servo = GetPIDOutput(pid2, X_pos, 20);
			         TERMINAL_PRINTF("X= %.2f      Y= %.2f  \r",X_servo, Y_servo);
			         
			         TERMINAL_PRINTF("poX= %.2f      posY= %.2f  \r\n",X_pos, Y_pos);

			         //Set the servo positions
					TFC_SetServo(0,-X_servo);
					TFC_SetServo(1,Y_servo);
				}
	}
	return 0;
}
