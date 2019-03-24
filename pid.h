#ifndef PID_H_
#define PID_H_

struct PIDStruct
{
	float	errSum;
	float	dError;
	float	lastError;
	float	Kp, Ki, Kd;
	float	upperLimit, lowerLimit;
	float	setpoint;
	int		startup;
};

struct PIDStructI
{
	int		feedbackGain;
	int		errSum;
	int		dError;
	int		lastError;
	int		Kp, Ki, Kd;
	int		Fp, Fi, Fd;
	int		upperLimit, lowerLimit;
	int		setpoint;
	int		startup;
};


// float version
typedef struct PIDStruct	PID;
PID* InitPID();

float GetPIDOutput(PID* pPID, float input, float dTime);

void SetPIDGain(PID* pPID, float p, float i, float d);
void SetPIDLimits(PID* pPID, float lwrLmt, float uprLmt);
void SetPIDSetpoint(PID* pPID, float setPt);


// integer version
typedef struct PIDStructI	PIDI;
PIDI* InitPIDI();

int GetPIDIOutput(PIDI* pPIDI, int input, int dTime);

void SetPIDIFeedbackGain(PIDI* pPIDI, int Gfb);
void SetPIDIGain(PIDI* pPIDI, int p, int i, int d);
void SetPIDILimits(PIDI* pPIDI, int lwrLmt, int uprLmt);
void SetPIDISetpoint(PIDI* pPIDI, int setPt);
void SetPIDIGainFactors(PIDI* pPIDI, int Fp, int Fi, int Fd);

#endif /* PID_H_ */