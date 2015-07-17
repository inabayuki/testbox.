//libraries
#include <math.h>

//application

//controller

//base
#include "system.h"
#include "mcutime.h"

//board
#include "pin.hpp"

//circuit
class three{
    public:
		int sw=0;
        int time=0;
        float pwmp[3]={1,1,1};
        float pwmrock[3]={0,0,0};
        float rad=0;
        float degree=0;
        float od=0;
        float l=90;//enc distance from the center
        float encDistance0=0;
        float encDistance1=0;
        float encDistance2=0;
        float Distance=0;
        int divide=0;
        int i=0;
        float j=0;
        float p=1.0/180.0;
        float d=0.6;
        float oldEnc0=0;
        float oldEnc1=0;
        float oldEnc2=0;
        float newEnc0=0;
        float newEnc1=0;
        float newEnc2=0;
        float encx0=0;
        float encx1=0;
        float encx2=0;
        float ency0=0;
        float ency1=0;
        float ency2=0;
        float fenc0=0;
        float fenc1=0;
        float fenc2=0;
        float x=0;
        float y=0;
        float integralx=0;
        float integraly=0;
        float radm=0;
        float tmp=0;
        float rad1=0;
        float oed0=0;
        float oed1=0;
        float oed2=0;
        float ned0=0;
        float ned1=0;
        float ned2=0;
        float ds=0;
        float pm=0;
        float distance=0;
        float mokux=0;
        float mokuy=0;
        float tmp1=0;
        float deg=0;
        float a=0;
		float b=0;
		int c=0;
		float up=0;
		float dg=1.0/1000.0;
		float dx=0.8;
		float dy=0.8;
		float x1=0;
		float x2=0;
		float x3=0;
		float y1=0;
		float y2=0;
		float y3=0;
		int k=0;
        Serial0 serial;
        CCW0 ccw0;
        CCW1 ccw1;
        CCW2 ccw2;
        CW0 cw0;
        CW1 cw1;
        CW2 cw2;
        Pwm0 pwm0;
        Pwm1 pwm1;
        Pwm2 pwm2;
        Enc0 enc0;
        Enc1 enc1;
        Enc2 enc2;
        Sw0 sw0;
        three();
        void degree1();
        void jkit();
        void jkit1();
        void mota();
        void degreerock();
        void test();
        void xy();
        void degmota();
        void mota1();
        void final();
        void indication();
        void switch0();
};

three::three(){
	ccw0.setupDigitalOut();
	ccw1.setupDigitalOut();
	ccw2.setupDigitalOut();

	cw0.setupDigitalOut();
	cw1.setupDigitalOut();
	cw2.setupDigitalOut();

	pwm0.setupPwmOut(80000,1.0);
	pwm1.setupPwmOut(80000,1.0);
	pwm2.setupPwmOut(80000,1.0);

	serial.setup(115200);
	enc0.setup();
	enc1.setup();
	enc2.setup();
	sw0.setupDigitalIn();
}

void three::switch0(){
	if(sw0.digitalRead()==0&&sw==0){
		sw=1;
		c=millis();
		return;
	}
	else if(sw0.digitalRead()==0&&sw==1){
		sw=0;
		pwm0.pwmWrite(1);
		pwm1.pwmWrite(1);
		pwm2.pwmWrite(1);
		cw0.digitalWrite(0);
		ccw0.digitalWrite(0);
		cw1.digitalWrite(0);
		ccw1.digitalWrite(0);
		cw2.digitalWrite(0);
		ccw2.digitalWrite(0);
		wait(3000);
		return;
	}
}

void three::xy(){

	if(k==0){
		a=250;
		b=250;
	}
	mokux=a-integralx;
	mokuy=b-integraly;
	distance=hypotf(mokuy,mokux);
	if(distance<=0.5){
		k++;
	}
	if(k==1){
		a=0;
		b=500;
	}

	if(k==2){
		a=-250;
		b=250;
	}

	if(k==3){
		a=0;
		b=0;
	}
	if(k==4){
		a=250;
		b=500;
	}/*
	if(k==5){
		a=-500;
		b=500;
	}
	/*
	a=0;
	b=2000;
	*/
	return;
}

void three::degree1(){

	fenc0=enc0.count();
	fenc1=enc1.count();
	fenc2=enc2.count();

	newEnc0=enc0.count()-oldEnc0;
	newEnc1=enc1.count()-oldEnc1;
	newEnc2=enc2.count()-oldEnc2;
	oed0=encDistance0;
	oed1=encDistance1;
	oed2=encDistance2;
	encDistance0=40*M_PI*enc0.count()/200.0;
	encDistance1=40*M_PI*enc1.count()/200.0;
	encDistance2=40*M_PI*enc2.count()/200.0;

	rad=(encDistance0+encDistance1+encDistance2)/(l*3);
	degree=rad/M_PI*180;

//****************degree 180 Indication*************************//
	divide=degree/180;
	if(divide>0){
		if(divide%2!=0){
			divide++;
		}
		degree=degree-180*divide;
	}
	if(divide<0){
		if(divide%2!=0){
			divide--;
		}
		degree=degree-180*divide;
	}
	if((int)degree%180==0&&divide%2!=0){
		degree=180;
	}
//**************************************************************//
	return;
}

void three::jkit1(){
	ned0=encDistance0-oed0;
	ned1=encDistance1-oed1;
	ned2=encDistance2-oed2;

	y1=((ned1+ned2)/(2*cos(30*M_PI/180)))-ned1/cos(30*M_PI/180);
	y2=-tan(30*M_PI/180)*ned0-(ned1/cos(30*M_PI/180));
	y3=tan(30*M_PI/180)*ned0+(ned2/cos(30*M_PI/180));

	x1=(ned1+ned2)/(2*cos(30*M_PI/180)*tan(30*M_PI/180));
	x2=-ned0;
	x3=-ned0;

	x=(x1+x2+x3)/3.0;
	y=(y1+y2+y3)/3.0;

	integralx+=x*cos(rad)-y*sin(rad);
	integraly+=y*cos(rad)+x*sin(rad);

	mokux=a-integralx;
	mokuy=b-integraly;
	distance=hypotf(mokuy,mokux);
	return;
}

void three::degmota(){
	deg=atan2(mokuy,mokux);
	pwmp[1]=-1*sin(deg-30*M_PI/180);
	pwmp[0]=-1*cos(deg);
	pwmp[2]=sin(30*M_PI/180+deg);

	tmp=fabsf(pwmp[0]);

	for(i=1;i<=2;i++){
		if(tmp<fabsf(pwmp[i])){
			tmp=fabsf(pwmp[i]);
		}
	}
	for(i=0;i<=2;i++){
		pwmp[i]=pwmp[i]/fabsf(tmp);
	}

	return;
}

void three::degreerock(){
	ds+=(degree-od)*d;
	for(i=0;i<=2;i++){
		pwmrock[i]=degree*p+ds;
	}
	tmp=fabsf(pwmrock[0]);
	for(i=1;i<=2;i++){
		if(tmp<fabsf(pwmrock[i])){
			tmp=fabsf(pwmrock[i]);
		}
	}
	for(i=0;i<=2;i++){
		pwmrock[i]=pwmrock[i]/fabsf(tmp);
	}
	for(i=0;i<=2;i++){
		pwmrock[i]=pwmrock[i]/5.5;
	}
	od=degree;
	return;
}

void  three::final(){
	for(i=0;i<=2;i++){
		pwmp[i]=pwmp[i]-pwmrock[i];
	}
	tmp1=fabsf(pwmp[0]);

	for(i=1;i<=2;i++){
		if(tmp1<fabsf(pwmp[i])){
			tmp1=fabsf(pwmp[i]);
		}
	}

	for(i=0;i<=2;i++){
		pwmp[i]=pwmp[i]*(distance/100.0);
	}

	for(i=0;i<=2;i++){
		pwmp[i]=pwmp[i]/fabsf(tmp1);
	}

	cw0.digitalWrite(0);
	ccw0.digitalWrite(1);
	cw1.digitalWrite(0);
	ccw1.digitalWrite(1);
	cw2.digitalWrite(0);
	ccw2.digitalWrite(1);

	for(i=0;i<=2;i++){
		if(pwmp[i]<0){
			if(i==0){
				pwmp[i]=pwmp[i]*-1;
				cw0.digitalWrite(1);
				ccw0.digitalWrite(0);
			}
			if(i==1){
				pwmp[i]=pwmp[i]*-1;
				cw1.digitalWrite(1);
				ccw1.digitalWrite(0);
			}
			if(i==2){
				pwmp[i]=pwmp[i]*-1;
				cw2.digitalWrite(1);
				ccw2.digitalWrite(0);
			}
		}
	}

	for(i=0;i<=2;i++){
		pwmp[i]=1-pwmp[i];
	}

	pwm0.pwmWrite(pwmp[0]);
	pwm1.pwmWrite(pwmp[1]);
	pwm2.pwmWrite(pwmp[2]);
	return;

}


void three::indication(){
	serial.printf("\r%.2f,%.2f,%.2f,%d,%d,%d,%.2f,%.2f,%.2f\n",degree,integralx,integraly,enc0.count(),enc1.count(),enc2.count(),pwmp[0],pwmp[1],pwmp[2]);
	return;
}

void three::test(){


	for(i=0;i<=2;i++){
		pwmp[i]=0;
	}

		pwm0.pwmWrite(pwmp[0]);
		pwm1.pwmWrite(pwmp[1]);
		pwm2.pwmWrite(pwmp[2]);
		cw0.digitalWrite(0);
		ccw0.digitalWrite(1);
		cw1.digitalWrite(0);
		ccw1.digitalWrite(1);
		cw2.digitalWrite(0);
		ccw2.digitalWrite(1);
		wait(10);


	serial.printf("%d,%d,%d\n\r",enc0.count(),enc1.count(),enc2.count());

	return;

}
int main(){
	three t;
	while(1){
	if(t.sw==0){
		t.switch0();
	}
	else if(t.sw==1){
	t.xy();
	t.degree1();
	t.jkit1();
	t.degmota();
	t.degreerock();
	t.final();
	t.indication();

	if(millis()-t.c>500){
		t.switch0();
	}
	}

	//t.test();

	}
	return 0;
}
