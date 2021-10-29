/**
* @file         lipkg.cpp
* @author       LD Robot
* @version      V01
* @brief         
* @note          
* @attention    COPYRIGHT LDROBOT
**/

#include "lipkg.h"
#include <math.h>
#include <algorithm>
#include "tofbf.h"

 
static const uint8_t CrcTable[256] =
{
	0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
	0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
	0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
	0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
	0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
	0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
	0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
	0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
	0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
	0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
	0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
	0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
	0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
	0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
	0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
	0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
	0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
	0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
	0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
	0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
	0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
	0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};


LiPkg::LiPkg():
	mTimestamp(0),
	mSpeed(0),
	mErrorTimes(0),
	mFrameReady(false),
	mIsPkgReady(false)
{
}

double LiPkg::GetSpeed(void)
{
	return mSpeed;
}

bool LiPkg::Parse(const uint8_t * data, long len)
{
	for (int i = 0; i < len; i++)
	{
		mDataTmp.push_back(*(data + i));
	}

	if (mDataTmp.size() < sizeof(LiDARFrameTypeDef))
		return false;

	if (mDataTmp.size() > sizeof(LiDARFrameTypeDef) * 100)
	{
		mErrorTimes++;
		mDataTmp.clear();
		return false;
	}

	uint16_t start = 0;

	while (start < mDataTmp.size() - 2)
	{
		start = 0;
		while (start < mDataTmp.size() - 2)
		{
			if ((mDataTmp[start] == PKG_HEADER) && (mDataTmp[start + 1] == PKG_VER_LEN))
			{
				break;
			}

			if ((mDataTmp[start] == PKG_HEADER) && (mDataTmp[start + 1] == (PKG_VER_LEN | (0x07 << 5))))
			{
				break;
			}
			start++;
		}

		if (start != 0)
		{
			mErrorTimes++;
			for (int i = 0; i < start; i++)
			{
				mDataTmp.erase(mDataTmp.begin());
			}
		}

		if (mDataTmp.size() < sizeof(LiDARFrameTypeDef))
			return false;

	
		LiDARFrameTypeDef* pkg = (LiDARFrameTypeDef *)mDataTmp.data();
		uint8_t crc = 0;

		for (uint32_t i = 0; i < sizeof(LiDARFrameTypeDef) - 1; i++)
		{
			crc = CrcTable[(crc ^ mDataTmp[i]) & 0xff];
		}

		if (crc == pkg->crc8)
		{
			double diff = (pkg->end_angle / 100 - pkg->start_angle / 100 + 360) % 360;
			if (diff > (double)pkg->speed*POINT_PER_PACK / 4500 * 3 / 2)
			{
				mErrorTimes++;
			}
			else
			{
				mSpeed = pkg->speed;
				mTimestamp = pkg->timestamp;
				uint32_t diff = ((uint32_t)pkg->end_angle + 36000 - (uint32_t)pkg->start_angle) % 36000;
				float step = diff / (POINT_PER_PACK - 1) / 100.0;
				float start = (double)pkg->start_angle / 100.0;
				float end = (double)(pkg->end_angle % 36000) / 100.0;
				PointData data;
				for (int i = 0; i < POINT_PER_PACK; i++)
				{
					data.distance = pkg->point[i].distance;
					data.angle = start + i * step;
					if (data.angle >= 360.0)
					{
						data.angle -= 360.0;
					}
					data.confidence = pkg->point[i].confidence;
					mOnePkg[i] = data;
					mFrameTmp.push_back(PointData(data.angle, data.distance, data.confidence));
				}
				//prevent angle invert
				mOnePkg.back().angle = end;

				mIsPkgReady = true;
			}

			for (uint32_t i = 0; i < sizeof(LiDARFrameTypeDef); i++)
			{
				mDataTmp.erase(mDataTmp.begin());
			}

			if (mDataTmp.size() < sizeof(LiDARFrameTypeDef))
			{
				break;
			}
		}
		else
		{
			mErrorTimes++;
			/*only remove header,not all frame,because lidar data may contain head*/
			for (int i = 0; i < 2; i++)
			{
				mDataTmp.erase(mDataTmp.begin());
			}
		}
	}

	return true;
}

bool LiPkg::AssemblePacket()
{
	float last_angle = 0;
	std::vector<PointData> tmp;
	int count = 0;
	for (auto n : mFrameTmp)
	{
		/*wait for enough data, need enough data to show a circle*/
		if (n.angle - last_angle < (-350.f)) /* enough data has been obtained */
		{
			Tofbf tofbfLd06(GetSpeed()); 
			//std::cout << GetSpeed() << std::endl;
			tmp = tofbfLd06.NearFilter(tmp);
			std::sort(tmp.begin(), tmp.end(), [](PointData a, PointData b) {return a.angle < b.angle; });
			if(tmp.size()>0)
			{
				ToLaserscan(tmp);
				mFrameReady = true;
				for(auto i=0;i<count;i++)
				{
					mFrameTmp.erase(mFrameTmp.begin());
				}
				return true;
			}
		}
		else
		{
			tmp.push_back(n);  /* getting data */
		}

		count++;
		last_angle = n.angle;
	}

	return false;
}

const std::array<PointData, POINT_PER_PACK>& LiPkg::GetPkgData(void)
{
	mIsPkgReady = false;
	return mOnePkg;
}

void LiPkg::ToLaserscan(std::vector<PointData> src)
{
  float angle_min, angle_max, range_min, range_max, angle_increment;
  
  /*Adjust the parameters according to the demand*/
  angle_min = 0 ;
  angle_max =  3.14159*2;
  range_min = 0.02;
  range_max = 12;
  /*Angle resolution, the smaller the resolution, the smaller the error after conversion*/
  angle_increment = ANGLE_TO_RADIAN(mSpeed/4500);
  /*Calculate the number of scanning points*/
  unsigned int beam_size = ceil((angle_max - angle_min) / angle_increment);
  output.header.stamp = ros::Time::now();
  output.header.frame_id = "lidar_frame";
  output.angle_min = angle_min;
  output.angle_max = angle_max;
  output.range_min = range_min;
  output.range_max = range_max;
  output.angle_increment = angle_increment;
  output.time_increment = 0.0;
  output.scan_time = 0.0;
  
  /*First fill all the data with Nan*/
  output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
  output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

  for (auto point : src)
  {
	float range = point.distance/1000.f;
    float angle = ANGLE_TO_RADIAN(point.angle);
	
    int index = (int)((angle - output.angle_min) / output.angle_increment);
    if (index >= 0 && index < beam_size)
    {
      /*If the current content is Nan, it is assigned directly*/
      if (std::isnan(output.ranges[index]))
      {
        output.ranges[index] = range;
      }   
      else
      {/*Otherwise, only when the distance is less than the current value, it can be re assigned*/
        if (range < output.ranges[index])
        {
          output.ranges[index] = range;
        }
      }
      output.intensities[index] = point.confidence;
    }
  }
}

/********************* (C) COPYRIGHT LD Robot *******END OF FILE ********/