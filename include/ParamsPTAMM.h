/*
 * Params.h
 *
 *  Created on: May 8, 2013
 *      Author: Giuseppe Loianno
 */

#ifndef PARAMS_H_
#define PARAMS_H_

#include <PTAMM_RGBD_cooperative/PtammParamsConfig.h>
#include <string.h>
#include <ros/ros.h>
#include <XmlRpcValue.h>

#include <dynamic_reconfigure/server.h>
#include <ParamsPTAMM.h>

typedef dynamic_reconfigure::Server<PTAMM_RGBD_cooperative::PtammParamsConfig> PtammParamsReconfigureServer;
typedef PTAMM_RGBD_cooperative::PtammParamsConfig VarParams;

class FixParams
{
	public:
	int DR;
	int DC;
	int STEP_R_MAX;
	int STEP_R_MIN;
	int STEP_C_MAX;
	int STEP_C_MIN;
	int Z_MAX;
	int Z_MIN;
	int image_width;
	int image_height;
	double fx_rgb;
	double fy_rgb;
	double cx_rgb;
	double cy_rgb;
	int ARBuffer_width;
	int ARBuffer_height;
	std::string BundleMEstimator;
	std::string TrackerMEstimator;
	double MinTukeySigma;
	int CandidateMinShiTomasiScore;
	//double Calibrator_BlurSigma;
	//double Calibrator_MeanGate;
	//int Calibrator_MinCornersForGrabbedImage;
	//bool Calibrator_Optimize;
	//bool Calibrator_Show;
	//bool Calibrator_NoDistortion;
	//double CameraCalibrator_MaxStepDistFraction;
	//int CameraCalibrator_CornerPatchSize;
	bool mgvnEnabled;
	int mgvnMenuItemWidth;
	int mgvnMenuTextOffset;
	int InitLevel;
	std::string parent_frame;
	bool gui;
	void readFixParams();
};

class ParamsAccess
{
public:
	ParamsAccess(){};
	ParamsAccess(PTAMM_RGBD_cooperative::PtammParamsConfig* _varParams, FixParams* _fixParams){


		varParams = _varParams;
		fixParams = _fixParams;
	};


	static PTAMM_RGBD_cooperative::PtammParamsConfig* varParams;
	static FixParams* fixParams;
};

class PtammParameters{
private:
	PTAMM_RGBD_cooperative::PtammParamsConfig mVarParams;
        FixParams mFixParams;

        PtammParamsReconfigureServer *mpPtammParamsReconfigureServer;

        void ptammParamsConfig(PTAMM_RGBD_cooperative::PtammParamsConfig & config, uint32_t level){
                mVarParams = config;
        };
public:
        PtammParameters()
        {
                mpPtammParamsReconfigureServer = new PtammParamsReconfigureServer(ros::NodeHandle("~"));
                PtammParamsReconfigureServer::CallbackType PtammParamCall = boost::bind(&PtammParameters::ptammParamsConfig, this, _1, _2);
                mpPtammParamsReconfigureServer->setCallback(PtammParamCall);

                mFixParams.readFixParams();

                ParamsAccess pAccess(&mVarParams, &mFixParams);
        }
};
#endif /* PARAMS_H_ */
