#pragma once

#include <chrono>

class PIDRegulator {
public:
	PIDRegulator(float t, float k, float ti, float td);
	float getK() const;
	float getTi() const;
	float getTd() const;
	void setK(float newK);
	void setTi(float newTi);
	void setTd(float newTd);
	float pid(float y, float yTarget);
	float pidAntyWindup(float y, float yTarget,float tv, float max);
	void clear();

private:
	float k_;
    float ti_;
    float td_;
	float t_;
    float ePast_ = 0.0f;
    float uPast_ = 0.0f;
	float calcUdUp(float e) const;
};