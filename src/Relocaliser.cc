// Copyright 2009 Isis Innovation Limited

/****************************************
  PTAMM: The relocalizer has been modified
  to allow it to search all maps for a match.
  If it recovers to the current map then
  behaviour is as before.
  If it recovers to another map then it signals
  for the Tracker, MapMaker, MapViewer and
  System to change maps.

****************************************/

#include "Relocaliser.h"
#include "SmallBlurryImage.h"
#include <cvd/utility.h>
#include <gvars3/instances.h>

namespace PTAMM {

using namespace CVD;
using namespace std;
using namespace GVars3;

/**
 * Create the relocalizer
 * @param maps The reference to all of the maps for searching. 
 * @param camera The camera model
 */
Relocaliser::Relocaliser(std::vector<Map*> &maps, ATANCamera &camera)
  : mvpMaps(maps),
    mpBestMap(NULL),
    mbNewRun(true),
    mCamera(camera)
{
}

/**
 * Return the best pose found
 * @return Best camera pose as an SE3.
 */
SE3<> Relocaliser::BestPose()
{
  return mse3Best;
}


/**
 * Attempt to recover the camera pose.
 * Searches all maps for a best match.
 * @param currentMap The current map
 * @param kCurrent The current camera image
 * @return sucess or failure
 */
bool Relocaliser::AttemptRecovery(Map & currentMap, KeyFrame &kCurrent)
{
  mbNewRun = true;
  
  // Ensure the incoming frame has a SmallBlurryImage attached
  if(!kCurrent.pSBI)
    kCurrent.pSBI = new SmallBlurryImage(kCurrent);
  else
    kCurrent.pSBI->MakeFromKF(kCurrent);


  // Find the best ZMSSD match from all keyframes in all maps
  std::vector<Map*>::iterator it;
  for( it = mvpMaps.begin(); it != mvpMaps.end(); it++)
  {
cerr<<"num_maps:"<<mvpMaps.size()<<endl;
    ScoreKFs((*it), kCurrent);
    mbNewRun = false;
  }
  
  // And estimate a camera rotation from a 3DOF image alignment
  pair<SE2<>, double> result_pair = kCurrent.pSBI->IteratePosRelToTarget(*mpBestMap->vpKeyFrames[mnBest]->pSBI, 6);
  mse2 = result_pair.first;
  double dScore =result_pair.second;
       //cerr << "dScore:" << dScore <<endl;

  SE3<> se3KeyFramePos = mpBestMap->vpKeyFrames[mnBest]->se3CfromW;
  mse3Best = SmallBlurryImage::SE3fromSE2(mse2, mCamera) * se3KeyFramePos; 
//if(dScore <   GV2.GetDouble("Reloc2.MaxScore", 9e6, SILENT))
  //////////////////////////////////////modified Loianno/////////////////////////////////////////
  double score_temp = GV2.GetDouble("Reloc2.MaxScore", ParamsAccess::varParams->RelocMaxScore, SILENT);
  /////////////////////////////////////////////////////////////////
  if(dScore < score_temp && dScore>0)
  {

    //are we in the same map?
    if (mpBestMap == &currentMap) {
     cerr << "dScore:" << dScore <<endl;
      return true;
    }
    else {
      //switch
      ostringstream os;
      os << "SwitchMap " << mpBestMap->MapID();
      GUI.ParseLine(os.str());
      //remain lost until switch complete
      return false;
    }
  }
  else {
    return false;
  }
}



/**
 * Compare current KF to all KFs stored in map by
 * Zero-mean SSD
 * @param pMap the map to search
 * @param kCurrent the current camera frame
 */
void Relocaliser::ScoreKFs(Map * pMap, KeyFrame &kCurrent)
{
  if(mbNewRun) //only reset on a new attempt at reloc.
  {
    mdBestScore = 99999999999999.9;
    mnBest = -1;
    mpBestMap = NULL;
  }

  
  for( unsigned int i = 0; i < pMap->vpKeyFrames.size(); i++ )
  {
    double dSSD = kCurrent.pSBI->ZMSSD( *pMap->vpKeyFrames[i]->pSBI);
    if(dSSD < mdBestScore)
    {
      mdBestScore = dSSD;
      mnBest = i;
      mpBestMap = pMap;
    }
  }
}

}
