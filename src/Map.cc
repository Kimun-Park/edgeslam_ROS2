/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"

#include<mutex>
#include<iostream>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
    
    // Multi-robot SLAM: Add to robot-specific container
    int robotId = pKF->GetRobotId();
    mspRobotKeyFrames[robotId].insert(pKF);
}

// Edge-SLAM
KeyFrame* Map::RetrieveKeyFrame(long int id)
{
    if(id >= 0)
    {
        for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        {
            if ((*sit)->mnId == (unsigned)id)
                return *sit;
        }
    }

    return NULL;
}

// Edge-SLAM
MapPoint* Map::RetrieveMapPoint(long int id, bool isTracking)
{
    if(id >= 0)
    {
        for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        {
            unsigned long int current_id = (isTracking)?(*sit)->mnId:(*sit)->lmMnId;
            if (current_id == (unsigned)id)
                return *sit;
        }
    }

    return NULL;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
    
    // Multi-robot SLAM: Add to robot-specific container
    int robotId = pMP->GetRobotId();
    mspRobotMapPoints[robotId].insert(pMP);
    
    // Check if it's a shared MapPoint (observed by multiple robots)
    bool isShared = false;
    for(const auto& robot : mspRobotMapPoints)
    {
        if(robot.first != robotId && pMP->IsObservedByRobot(robot.first))
        {
            isShared = true;
            break;
        }
    }
    if(isShared)
    {
        mspSharedMapPoints.insert(pMP);
    }
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);

    // This only erase the pointer
    mspMapPoints.erase(pMP);
    
    // Multi-robot SLAM: Remove from robot-specific containers
    int robotId = pMP->GetRobotId();
    if(mspRobotMapPoints.count(robotId))
    {
        mspRobotMapPoints[robotId].erase(pMP);
    }
    mspSharedMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);

    // This only erase the pointer
    mspKeyFrames.erase(pKF);
    
    // Multi-robot SLAM: Remove from robot-specific container
    int robotId = pKF->GetRobotId();
    if(mspRobotKeyFrames.count(robotId))
    {
        mspRobotKeyFrames[robotId].erase(pKF);
    }

    // TODO: This only erase the pointer.
    // Delete the KeyFrame
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
    
    // Multi-robot SLAM: Clear robot-specific containers
    mspRobotKeyFrames.clear();
    mspRobotMapPoints.clear();
    mspSharedMapPoints.clear();
}

// Multi-robot SLAM: Get KeyFrames for specific robot
std::vector<KeyFrame*> Map::GetRobotKeyFrames(int robotId)
{
    unique_lock<mutex> lock(mMutexMap);
    if(mspRobotKeyFrames.count(robotId))
        return vector<KeyFrame*>(mspRobotKeyFrames[robotId].begin(), mspRobotKeyFrames[robotId].end());
    return vector<KeyFrame*>();
}

// Multi-robot SLAM: Get MapPoints for specific robot
std::vector<MapPoint*> Map::GetRobotMapPoints(int robotId)
{
    unique_lock<mutex> lock(mMutexMap);
    if(mspRobotMapPoints.count(robotId))
        return vector<MapPoint*>(mspRobotMapPoints[robotId].begin(), mspRobotMapPoints[robotId].end());
    return vector<MapPoint*>();
}

// Multi-robot SLAM: Get shared MapPoints (observed by multiple robots)
std::vector<MapPoint*> Map::GetSharedMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspSharedMapPoints.begin(), mspSharedMapPoints.end());
}

// Multi-robot SLAM: Get list of active robots
std::set<int> Map::GetActiveRobots()
{
    unique_lock<mutex> lock(mMutexMap);
    std::set<int> activeRobots;
    for(const auto& robot : mspRobotKeyFrames)
    {
        if(!robot.second.empty())
            activeRobots.insert(robot.first);
    }
    return activeRobots;
}

// Multi-robot SLAM: Count MapPoints for specific robot
long unsigned int Map::RobotMapPointsInMap(int robotId)
{
    unique_lock<mutex> lock(mMutexMap);
    if(mspRobotMapPoints.count(robotId))
        return mspRobotMapPoints[robotId].size();
    return 0;
}

// Multi-robot SLAM: Count KeyFrames for specific robot
long unsigned int Map::RobotKeyFramesInMap(int robotId)
{
    unique_lock<mutex> lock(mMutexMap);
    if(mspRobotKeyFrames.count(robotId))
        return mspRobotKeyFrames[robotId].size();
    return 0;
}

// Multi-robot SLAM: Test function for robot data separation
void Map::TestRobotDataSeparation()
{
    try {
        std::cout << "=== Multi-Robot SLAM Data Separation Test ===" << std::endl;
        
        unique_lock<mutex> lock(mMutexMap);
        
        std::cout << "Total KeyFrames: " << mspKeyFrames.size() << std::endl;
        std::cout << "Total MapPoints: " << mspMapPoints.size() << std::endl;
        std::cout << "Robot containers size: " << mspRobotKeyFrames.size() << std::endl;
        
        std::cout << "=============================================" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "ERROR in TestRobotDataSeparation: " << e.what() << std::endl;
    }
}

} //namespace ORB_SLAM
