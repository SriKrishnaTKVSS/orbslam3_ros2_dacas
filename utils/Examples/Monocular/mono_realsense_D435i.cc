/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <condition_variable>

#include <opencv2/core/core.hpp>

#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"

#include <System.h>

//---------coming from Mocap code--------
#include "RTProtocol.h"
#include "RTPacket.h"

#ifdef _WIN32
#define sleep Sleep
#else
#include <unistd.h>
#endif
//----------------------
//------For post processing code in python------
#include <pybind11/embed.h> //python header for embedding
namespace py=pybind11;

//-----------------


using namespace std;

bool b_continue_session;

void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;

}

rs2_vector interpolateMeasure(const double target_time,
                              const rs2_vector current_data, const double current_time,
                              const rs2_vector prev_data, const double prev_time);

static rs2_option get_sensor_option(const rs2::sensor& sensor)
{
    // Sensors usually have several options to control their properties
    //  such as Exposure, Brightness etc.

    std::cout << "Sensor supports the following options:\n" << std::endl;

    // The following loop shows how to iterate over all available options
    // Starting from 0 until RS2_OPTION_COUNT (exclusive)
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)
    {
        rs2_option option_type = static_cast<rs2_option>(i);
        //SDK enum types can be streamed to get a string that represents them
        std::cout << "  " << i << ": " << option_type;

        // To control an option, use the following api:

        // First, verify that the sensor actually supports this option
        if (sensor.supports(option_type))
        {
            std::cout << std::endl;

            // Get a human readable description of the option
            const char* description = sensor.get_option_description(option_type);
            std::cout << "       Description   : " << description << std::endl;

            // Get the current value of the option
            float current_value = sensor.get_option(option_type);
            std::cout << "       Current Value : " << current_value << std::endl;

            //To change the value of an option, please follow the change_sensor_option() function
        }
        else
        {
            std::cout << " is not supported" << std::endl;
        }
    }

    uint32_t selected_sensor_option = 0;
    return static_cast<rs2_option>(selected_sensor_option);
}

int main(int argc, char **argv) {


    //---------------- Create the file for Mocap pose---------
    // ofstream mocap_data("Mocap_data.txt",'a');
    const string mocap_file_name="Mocap_data.txt";
    remove(mocap_file_name.c_str());
    ofstream mocap_data {mocap_file_name,ios_base::app};

    if (argc < 3 || argc > 4) {
        cerr << endl
             << "Usage: ./mono_realsense_D435i path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name;

    if (argc == 4) {
        file_name = string(argv[argc - 1]);
    }

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    double offset = 0; // ms

    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    rs2::device selected_device;
    if (devices.size() == 0)
    {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
        return 0;
    }
    else
        selected_device = devices[0];

    std::vector<rs2::sensor> sensors = selected_device.query_sensors();
    int index = 0;
    // We can now iterate the sensors and print their names
    for (rs2::sensor sensor : sensors)
        if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
            ++index;
            if (index == 1) {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
                sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT,5000);
                sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0); // switch off emitter
            }
            // std::cout << "  " << index << " : " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
            get_sensor_option(sensor);
            if (index == 2){
                // RGB camera (not used here...)
                sensor.set_option(RS2_OPTION_EXPOSURE,100.f);
            }
        }

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);

    // IMU callback
    std::mutex imu_mutex;
    std::condition_variable cond_image_rec;

    cv::Mat imCV;
    int width_img, height_img;
    double timestamp_image = -1.0;
    bool image_ready = false;
    int count_im_buffer = 0; // count dropped frames

    auto imu_callback = [&](const rs2::frame& frame)
    {
        std::unique_lock<std::mutex> lock(imu_mutex);

        if(rs2::frameset fs = frame.as<rs2::frameset>())
        {
            count_im_buffer++;

            double new_timestamp_image = fs.get_timestamp()*1e-3;
            if(abs(timestamp_image-new_timestamp_image)<0.001){
                // cout << "Two frames with the same timeStamp!!!\n";
                count_im_buffer--;
                return;
            }

            rs2::video_frame ir_frameL = fs.get_infrared_frame(1);
            imCV = cv::Mat(cv::Size(width_img, height_img), CV_8U, (void*)(ir_frameL.get_data()), cv::Mat::AUTO_STEP);

            timestamp_image = fs.get_timestamp()*1e-3;
            image_ready = true;

            lock.unlock();
            cond_image_rec.notify_all();
        }
    };

    rs2::pipeline_profile pipe_profile = pipe.start(cfg, imu_callback);

    rs2::stream_profile cam_left = pipe_profile.get_stream(RS2_STREAM_INFRARED, 1);

    rs2_intrinsics intrinsics_left = cam_left.as<rs2::video_stream_profile>().get_intrinsics();
    width_img = intrinsics_left.width;
    height_img = intrinsics_left.height;
    cout << "Left camera: \n";
    std::cout << " fx = " << intrinsics_left.fx << std::endl;
    std::cout << " fy = " << intrinsics_left.fy << std::endl;
    std::cout << " cx = " << intrinsics_left.ppx << std::endl;
    std::cout << " cy = " << intrinsics_left.ppy << std::endl;
    std::cout << " height = " << intrinsics_left.height << std::endl;
    std::cout << " width = " << intrinsics_left.width << std::endl;
    std::cout << " Coeff = " << intrinsics_left.coeffs[0] << ", " << intrinsics_left.coeffs[1] << ", " <<
        intrinsics_left.coeffs[2] << ", " << intrinsics_left.coeffs[3] << ", " << intrinsics_left.coeffs[4] << ", " << std::endl;
    std::cout << " Model = " << intrinsics_left.model << std::endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true, 0, file_name);
    float imageScale = SLAM.GetImageScale();

    double timestamp;
    cv::Mat im;

    double t_resize = 0.f; SLAM.TrackMonocular(im, timestamp);
    double t_track = 0.f;

    CRTProtocol rtProtocol;

    while (!SLAM.isShutDown())
    {
        std::vector<rs2_vector> vGyro;
        std::vector<double> vGyro_times;
        std::vector<rs2_vector> vAccel;
        std::vector<double> vAccel_times;

        {
            std::unique_lock<std::mutex> lk(imu_mutex);
            if(!image_ready)
                cond_image_rec.wait(lk);

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point time_Start_Process = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point time_Start_Process = std::chrono::monotonic_clock::now();
#endif

            if(count_im_buffer>1)
                cout << count_im_buffer -1 << " dropped frs\n";
            count_im_buffer = 0;

            timestamp = timestamp_image;
            im = imCV.clone();

            image_ready = false;
        }

        if(imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));

#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_Start_Track = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t_Start_Track = std::chrono::monotonic_clock::now();
    #endif
#endif
//------------Mocap code---------------
    try
    {
        // CRTProtocol rtProtocol;

        //Example code for how to use discovery calls.
        //if (rtProtocol.DiscoverRTServer(4534, false))
        //{
        //    sleep(1);
        //    const auto numberOfResponses = rtProtocol.GetNumberOfDiscoverResponses();
        //    for (auto index = 0; index < numberOfResponses; index++)
        //    {
        //        unsigned int addr;
        //        unsigned short basePort;
        //        std::string message;
        //        if (rtProtocol.GetDiscoverResponse(index, addr, basePort, message))
        //        {
        //            printf("%2d - %d.%d.%d.%d:%d\t- %s\n", index, 0xff & addr, 0xff & (addr >> 8), 0xff & (addr >> 16), 0xff & (addr >> 24), basePort, message.c_str());
        //        }
        //    }
        //}
        //else
        //{
        //    printf("%s", rtProtocol.GetErrorString());
        //}

        const char           serverAddr[] = "10.176.60.155";
        const unsigned short basePort = 22222;
        const int            majorVersion = 1;
        const int            minorVersion = 19;
        const bool           bigEndian = false;

        bool dataAvailable = false;
        bool streamFrames = false;
        unsigned short udpPort = 6734;
        // while (true)
        // while(!rtProtocol.Connected())
        // {
        if (!rtProtocol.Connected())
        {
            if (!rtProtocol.Connect(serverAddr, basePort, &udpPort, majorVersion, minorVersion, bigEndian))
            {
                printf("rtProtocol.Connect: %s\n\n", rtProtocol.GetErrorString());
                sleep(1);
                continue;
            }
        }

        if (!dataAvailable)
        {
            if (!rtProtocol.Read6DOFSettings(dataAvailable))
            {
                printf("rtProtocol.Read6DOFSettings: %s\n\n", rtProtocol.GetErrorString());
                sleep(1);
                continue;
            }
        }

        if (!streamFrames)
        {
            if (!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort, NULL, CRTProtocol::cComponent6d))
            {
                printf("rtProtocol.StreamFrames: %s\n\n", rtProtocol.GetErrorString());
                sleep(1);
                continue;
            }
            streamFrames = true;

            printf("Starting to streaming 6DOF data\n\n");
        }

        CRTPacket::EPacketType packetType;

        if (rtProtocol.Receive(packetType, true) == CNetwork::ResponseType::success)
        {
            if (packetType == CRTPacket::PacketData)
            {
                float fX, fY, fZ;
                float rotationMatrix[9];

                CRTPacket* rtPacket = rtProtocol.GetRTPacket();

                printf("Frame %d\n", rtPacket->GetFrameNumber());
                printf("======================================================================================================================\n");

                for (unsigned int i = 0; i < rtPacket->Get6DOFBodyCount(); i++)
                {
                    if (rtPacket->Get6DOFBody(i, fX, fY, fZ, rotationMatrix))
                    {
                        const char* pTmpStr = rtProtocol.Get6DOFBodyName(i);
                        if (pTmpStr)
                        {
                            printf("%-12s ", pTmpStr);
                        }
                        else
                        {
                            printf("Unknown     ");
                        }
                        printf("Pos: %9.3f %9.3f %9.3f    Rot: %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n",
                            fX, fY, fZ, rotationMatrix[0], rotationMatrix[1], rotationMatrix[2],
                            rotationMatrix[3], rotationMatrix[4], rotationMatrix[5], rotationMatrix[6], rotationMatrix[7], rotationMatrix[8]);
                        mocap_data << std::fixed <<  timestamp*(1e9) << " "<< fX << " "<< fY<< " " << fZ << " "<< rotationMatrix[0] << " "<<rotationMatrix[1]<< " "<<rotationMatrix[2]<< " "<<rotationMatrix[3]<< " "<<rotationMatrix[4]<< " "<<rotationMatrix[5]<< " "<<rotationMatrix[6]<< " "<<rotationMatrix[7]<< " "<<rotationMatrix[8]<<endl;
                        
                    }
                }
                printf("\n");
            }
        }
        // }

    }
    catch (std::exception& e)
    {
        printf("%s\n", e.what());
    }
    //--------------------------
        // Stereo images are already rectified.
        SLAM.TrackMonocular(im, timestamp);
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_End_Track = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t_End_Track = std::chrono::monotonic_clock::now();
    #endif
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Track - t_Start_Track).count();
        SLAM.InsertTrackTime(t_track);
#endif
    }
    rtProtocol.StopCapture();
    rtProtocol.Disconnect();
    mocap_data.close();
    cout<< "Post processing and plotting...\n";
    //------------ Post processing using process_data.py-------------
    // py::scoped_interpreter guard{};

//     try {
//         py::exec(R"(
// import sys
// sys.path.append('/home/srikrishna/Desktop/Projects/scale_est/Krishna_pybind_orsbslam3/src/ORB_SLAM3/Examples/Monocular
// /processing_data.py')  // Replace with the actual path to the folder containing process_data.py
//     )");
//         py::module_ processing_data=py::module_::import("processing_data");

//         processing_data.attr("main")();
//     }
//     catch (const py::error_already_set& e) {
//         std::cerr << "Error calling Python function: " << e.what() << std::endl;
//     }
    py::scoped_interpreter guard{}; // Start the Python interpreter

    try {
        // Correctly formatted Python code inside exec() with properly escaped path
        py::exec(R"(
import sys
sys.path.append('/home/srikrishna/Desktop/Projects/scale_est/Krishna_pybind_orsbslam3/src/ORB_SLAM3/Examples/Monocular/')
)");

        // Import the 'process_data' module (without the .py extension)
        py::module_ processing_data = py::module_::import("processing_data");

        // Call the 'main' function from process_data.py
        processing_data.attr("main")();  // This will call the main() function in process_data.py
    } catch (const py::error_already_set& e) {
        std::cerr << "Error calling Python function: " << e.what() << std::endl;
    }


    cout << "System shutdown!\n";
}