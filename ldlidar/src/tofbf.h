#ifndef __TOFBF_H_
#define __TOFBF_H_

#include <stdint.h>
#include <vector>
#include "lipkg.h"

class Tofbf
{
private:
	const int CONFIDENCE_LOW = 15;         /* Low confidence threshold */
    const int CONFIDENCE_SINGLE = 220;     /* Discrete points require higher confidence */
    const int SCAN_FRE = 4500;             /* Default scan frequency, to change, read according to radar protocol */
    double offset_x, offset_y;
    double curr_speed;
    Tofbf() = delete;
    Tofbf(const Tofbf &) = delete;
    Tofbf &operator=(const Tofbf &) = delete;
public:
    Tofbf(int speed);
	std::vector<PointData> NearFilter(const std::vector<PointData> &tmp) const;
    ~Tofbf();
};

#endif
