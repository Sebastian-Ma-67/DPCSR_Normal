/**
 * Copyright (C), 2015.XXXX公司,All rights reserved.
 * @file
 * @brief
 * @author
 * @email sebastian.xq.ma@gmail.com
 * @version
 * @date 2015-XX-XX
 * @update 2022-04-27
 * @note
 * @warning
 * @todo
 * @history:
 */

#ifndef FUSION_H
#define FUSION_H

#include <cmath>
#include <vector>
#include <iostream>

// 一维滤波器信息结构体
typedef struct
{
	double filterValue; // k-1时刻的滤波值，即是k-1时刻的值
	double kalmanGain;	//   Kalamn增益
	double A;			// x(n)=A*x(n-1)+u(n),u(n)~N(0,Q)
	double H;			// z(n)=H*x(n)+w(n),w(n)~N(0,R)
	double Q;			//预测过程噪声偏差的方差
	double R;			//测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
	double P;			//估计误差协方差
} KalmanInfo;

/**
 * @brief Init_KalmanInfo   初始化滤波器的初始值
 * @param info  滤波器指针
 * @param Q 预测噪声方差 由系统外部测定给定
 * @param R 测量噪声方差 由系统外部测定给定
 */
// void Init_KalmanInfo(KalmanInfo* info, double Q, double R)
//{
//	info->A = 1;  //标量卡尔曼
//	info->H = 1;  //
//	info->P = 10;  //后验状态估计值误差的方差的初始值（不要为0问题不大）
//	info->Q = Q;    //预测（过程）噪声方差 影响收敛速率，可以根据实际需求给出
//	info->R = R;    //测量（观测）噪声方差 可以通过实验手段获得
//	info->filterValue = 0;// 测量的初始值
// };

// double KalmanFilter(KalmanInfo* kalmanInfo, double lastMeasurement);

/**
* @class Fusion 
* @brief brief description
* @author pengdi_huang
* @note
* detailed description
*/
class Fusion
{
public:
	Fusion();
	~Fusion();

	// set node weight for kinect fusion mode
	void SetKinectMode(const int &iNodeNum, float fRawWeight = 1.0f);

	// set map size
	void SetAccDisSize(const int &iNodeNum, float fRawDis = 0.0f);

	// fusion using kinect fusion mode - Weighted average
	// if vWeights and fRawWeight is the constant 1, respectively
	// then it is a mean least squares
	void KinectFusion(const std::vector<float> &vCurrentDis, 
					  std::vector<float> &vWeights);

	/** @brief 
    * @param[in]  name brief
	* @param[out] name brief
    */
	void ConvexBasedFusion(const std::vector<float> &vCurrentDis, 
						   std::vector<float> &vDisMap);
    
	/** @brief 
    * @param[in]  name brief
	* @param[out] name brief
    */
	void UnionMinimalFusion(const std::vector<float> &vCurrentDis);

	//*******data********
	std::vector<float> m_vAccDis;

private:
	std::vector<float> m_vKinectWeight;
};

#endif