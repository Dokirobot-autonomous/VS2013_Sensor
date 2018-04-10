#pragma once

#include "GoogleEarth.hpp"
#include "Coor.hpp"
#include "Position.hpp"
#include "TransMatrix.hpp"
#include "GoogleEarth.hpp"
#include <math.h>

class GlobalToLocal
{
public:
	GlobalToLocal(){};
	~GlobalToLocal(){};
	//	原点をset
	void setOriginal(double latitude, double longtitude, double elevation, double direction)
	{
		org_ge.setIniPos(latitude, longtitude, elevation);
		org_direction = direction;
	};
	//	Coor型に座標変換
	Coor<> getCoor(double latitude, double longtitude, double elevation)
	{
		org_ge.setData(latitude, longtitude, elevation);
		std::vector<double> tmp = org_ge.get_ENU();	//	ENU座標変換
		Coor<> out(tmp[0], tmp[1]);
		out = TransMatrix(0.0, 0.0, (double)(org_direction - 90.0)*M_PI / 180.0).Transform(out);
		return out;
	};
	//	Position型に座標変換
	Position<> getPosition(double latitude, double longtitude, double elevation, double direction)
	{
		Coor<> tmp = getCoor(latitude, longtitude, elevation);
		Position<> out = Position<>(tmp.x, tmp.y, (org_direction - direction)*M_PI / 180);
		return out;
	}

private:
	GoogleEarth org_ge;		//	基準となるGlobal座標(GPSの座標系)
	double org_direction;	//	基準となる角度（北を0degとして時計回りを正）
};