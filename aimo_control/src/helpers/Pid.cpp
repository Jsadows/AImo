#include "aimo_control/helpers/PIDRegulator.hpp"

PIDRegulator::PIDRegulator(float t, float k=0.0f, float ti=10000000.0f, float td=0.0f): 
k_(k),
ti_(ti),
td_(td),
t(t){}

float PIDRegulator::pid(float y, float yTarget){
	float e = yTarget - y;
	float ui = uPast_ + k_/ti_*t_*(ePast_ + e)/2.0f;
	float u = calcUdUp(e) + ui;
	uPast_ = ui;
	ePast_ = e;
	return u;
}


float PIDRegulator::pidAntyWindup(float y, float yTarget,float tv, float max){
        float uw;
        if(uPast_ > max ){
            uw = max;
        }
        else if (uPast_ < -max)
        {
            uw = -max;
        }
        else{
            uw = uPast_;
        }
        
        float e = yTarget - y;
        float ui = uPast_ + k_/ti_*t_*(ePast_ + e)/2.0f +t_/tv * (uw - uPast_);
        float u = calcUdUp(e) + ui;
        uPast_ = ui;
        ePast_ = e;
        return u;
    }

float PIDRegulator::calcUdUp(float e) const{
	float up = k_*e;
	float ud = k_*td_*(e-ePast_)/t_;
	return up + ud;
}

void PIDRegulator::clear(){
        this->ePast_ = 0.0f;
        this->uPast_ = 0.0f;
    }

float PIDRegulator::getK() const{
    return this->k_;
}
float PIDRegulator::getTi() const{
    return this->ti_;
}
float PIDRegulator::getTd() const{
    return this->td_;
}
void PIDRegulator::setK(float newK){
    this->k_ = newK;
}
void PIDRegulator::setTi(float newTi){
    this->ti_ = newTi;
}
void PIDRegulator::setTd(float newTd){
    this->td_ = newTd;
}