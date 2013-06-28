#ifndef XAP_H
#define XAP_H
typedef struct 
{
	float t;
	float x;
	float y;
	float psi;
	float vx;
	float vy;
	float omega;
	float cam_1_raw_x;
	float cam_1_raw_y;
	float cam_1_raw_psi;
	float cam_1_seen;
	float cam_2_raw_x;
	float cam_2_raw_y;
	float cam_2_raw_psi;
	float cam_2_seen;
} states;

typedef struct {
	states car[8];
	bool on;
	bool padding [3];
} PI_IN;

typedef struct {
	bool reply;
	bool padding [3];
} PI_OUT;

#endif /* XAP_H */
