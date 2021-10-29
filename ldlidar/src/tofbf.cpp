
#include "tofbf.h"
#include <math.h>
#include <iostream>
#include <algorithm>

/*!
	\brief      Select the radar version number and set the current speed 
	\param[in]
	  \arg  version    Version number of current radar
	  \arg  speed      Current radar speed
	\param[out] none   A circle of radar data packed
	\retval     none
*/
Tofbf::Tofbf(int speed)
{
	curr_speed = speed;
}

Tofbf::~Tofbf()
{
}

/*!
    \brief       Filter within 5m to filter out unreasonable data points
	\param[in]
	  \arg  tmp 
	\param[out] none
	\retval     Normal data
*/
std::vector<PointData> Tofbf::NearFilter(const std::vector<PointData> &tmp) const
{
	std::vector<PointData> normal, pending, item;
	std::vector<std::vector<PointData>> group;

    // Remove points within 5m
	for (auto n : tmp)
	{
        if (n.distance < 5000)
		{
			pending.push_back(n);
		}
		else
		{
			normal.push_back(n);
		}
	}

	if (tmp.empty())
		return normal;

	double angle_delta_up_limit = curr_speed / SCAN_FRE * 2;

	// std::cout <<angle_delta_up_limit << std::endl; test code

	//sort 
	std::sort(pending.begin(), pending.end(), [](PointData a, PointData b) { return a.angle < b.angle; });

	PointData last(-10, 0, 0);
	//group
	for (auto n : pending)
	{
		if (abs(n.angle - last.angle) > angle_delta_up_limit || abs(n.distance - last.distance) > last.distance*0.03)
		{
			if(item.empty() == false)
			{
				group.push_back(item);
				item.clear();
			}
		}
		item.push_back(n);
		last = n;
	}
	// push back last item
	if (item.empty() == false)
		group.push_back(item);

	if (group.empty())
		return normal;

	// Connection 0 degree and 359 degree
	auto first_item = group.front().front();
	auto last_item = group.back().back();
	if (abs(first_item.angle + 360.f - last_item.angle) < angle_delta_up_limit && abs(first_item.distance - last_item.distance) < last.distance*0.03)
	{
		group.front().insert(group.front().begin(), group.back().begin(), group.back().end());
		group.erase(group.end() - 1);
	}
	//selection
	for (auto n : group)
	{
		if (n.size() == 0)
			continue;
		//No filtering if there are many points
		if (n.size() > 15)
		{
			normal.insert(normal.end(), n.begin(), n.end());
			continue;
		}

		//Filter out those with few points
        if (n.size() < 3)
		{
            int c = 0;
            for(auto m : n)
            {
                c += m.confidence;
            }
            c /= n.size();
            if(c < CONFIDENCE_SINGLE)
                continue;
		}

		//Calculate the mean value of distance and confidence
		double confidence_avg = 0;
        double dis_avg = 0;
		for (auto m : n)
		{
			confidence_avg += m.confidence;
			dis_avg += m.distance;
		}
		confidence_avg /= n.size();
		dis_avg /= n.size();

		//High confidence, no filtering
		if (confidence_avg > CONFIDENCE_LOW)
		{
			normal.insert(normal.end(), n.begin(), n.end());
			continue;
		}

	}

	return normal;
}

