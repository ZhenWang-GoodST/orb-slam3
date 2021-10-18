/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/



#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

#include "orb_utils.h"
#include "glog/logging.h"

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{  
    // if(argc < 5)
    // {
    //     cerr << endl << "Usage: ./mono_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) (trajectory_file_name)" << endl;
    //     return 1;
    // }

    // const int num_seq = (argc-3)/2;
    int num_seq;
    int type = 0;
    std::string root_path = "";
    std::string yaml = "";
    std::string vocabulary = "";
    std::string start_stamp = "";
    if (argc > 3) {
        type = atoi(argv[1]);
        root_path = argv[2];
        start_stamp = argv[3];
        yaml = argv[4];
    } else {
        readParameter<int>("type", type);
        readParameter<std::string>("root_path", root_path);
        readParameter<std::string>("start_stamp", start_stamp);
        readParameter<std::string>("yaml", yaml);
    }
    // readParameter<int>("num_seq", num_seq);
    num_seq = 1;
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = "/home/tonglu/VO-LOAM/github/orb-slam3/KeyFrameTrajectory.txt";
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq = 1;
    vector< vector<string> > vstrImageFilenames;
    vector< vector<double> > vTimestampsCam;
    vector<int> nImages;

    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    nImages.resize(num_seq);

    int tot_images = 0;
    std::string imagepath = "";
    std::string stamppath = "";
    if (type == 0) {//realsense 
        // yaml = "/home/wz/VO-LOAM/github/orb-slam3/conf/myrealsense.yaml";
        imagepath = root_path + "/camera/color/image_raw";
        stamppath = root_path + "/camera/color/image_rawinfo.txt";
    } else if(type == 1) {//usb相机
        // yaml = "/home/wz/VO-LOAM/github/orb-slam3/conf/nondistor.yaml";
        imagepath = root_path + "/camera/color/image_raw1";
        stamppath = root_path + "/camera/color/image_raw1info.txt";
    } else if(type == 2) {//Euroc
        // yaml = "/home/tonglu/VO-LOAM/github/orb-slam3/Examples/Monocular/EuRoC.yaml";
        readParameter<std::string>("imagepath", imagepath);
        readParameter<std::string>("stamppath", stamppath);
    }
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";
        LoadImages(imagepath, stamppath, vstrImageFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageFilenames[seq].size();
        tot_images += nImages[seq];
    }
    int start_id = 0;
    for (int i = 0; i < vTimestampsCam.size(); ++i) {
        for (int j = 0; j < vTimestampsCam[i].size(); ++j) {
            if (strcmp(std::to_string(vTimestampsCam[i][j]).c_str(), start_stamp.c_str()) == 0) {
                start_id = j;
                break;
            }
        }
        if (start_id == 0) continue;
        vTimestampsCam[i].erase(vTimestampsCam[i].begin(), vTimestampsCam[i].begin() + start_id);
        vstrImageFilenames[i].erase(vstrImageFilenames[i].begin(), vstrImageFilenames[i].begin() + start_id);
    }
    

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // readParameter<std::string>("vocabulary", vocabulary);
    vocabulary = "/home/wz/VO-LOAM/github/orb-slam3/Vocabulary/ORBvoc.bin";
    ORB_SLAM3::System SLAM(vocabulary,yaml,ORB_SLAM3::System::MONOCULAR, true);

    log_dir = root_path + "/log/camera" + std::to_string(type) + "/";
    std::string pangolin_dir = log_dir + "pangolin";
    std::string tracking_dir = log_dir + "tracking";
    createFolders(pangolin_dir.c_str());
    createFolders(tracking_dir.c_str());
    for (seq = 0; seq<num_seq; seq++)
    {

        // Main loop
        cv::Mat im;
        int proccIm = 0;
        for(int ni=0; ni<vstrImageFilenames[seq].size(); ni++, proccIm++)
        {
            // Read image from file
            im = cv::imread(vstrImageFilenames[seq][ni],cv::IMREAD_UNCHANGED);
            double tframe = vTimestampsCam[seq][ni];

            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  vstrImageFilenames[seq][ni] << endl;
                return 1;
            }
            global_timestamp = std::to_string(tframe);
            glog_dir = log_dir + "trackinginfo/" + std::to_string(tframe);
            createFolders(glog_dir.c_str());
            google::InitGoogleLogging("orb-log");
            FLAGS_log_dir = glog_dir; 

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the image to the SLAM system
            SLAM.TrackMonocular(im,tframe);
            google::ShutdownGoogleLogging();

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

#ifdef REGISTER_TIMES
            double t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6);
        }

        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }

    }
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    std::string mappath = log_dir + "map/";
    createFolders(mappath.c_str());
    const string f_file =  log_dir + "trajectory.txt";
    const string kf_file = log_dir + "keyframe.txt";
    if (bFileName)
    {
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".jpg");
            double t;
            ss >> t;
            vTimeStamps.push_back(t);

        }
    }
}
