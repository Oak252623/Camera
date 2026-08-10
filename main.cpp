#include <new>      
#include <limits>   
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>
#include <chrono>
#include <string>
#include <thread>
#include <atomic>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

// ---------------------------------------------------------
// THREAD-SAFE TELEMETRY BRIDGE
// ---------------------------------------------------------
std::atomic<double> shared_tx(0.0), shared_ty(0.0), shared_tz(0.0);
std::atomic<bool> pipeline_active(true);
std::atomic<bool> fresh_data(false); // The Cryptographic Handshake

// Background UDP Thread: strictly bounded by CV performance
void udp_telemetry_streamer(std::string target_ip, int target_port) {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(target_port);
    inet_pton(AF_INET, target_ip.c_str(), &server_addr.sin_addr);

    std::cout << "[NET] UDP Telemetry Thread spawned. Target: " << target_ip << ":" << target_port << "\n";

    while (pipeline_active) {
        if (fresh_data.load()) {
            std::string payload = std::to_string(shared_tx.load()) + "," + 
                                  std::to_string(shared_ty.load()) + "," + 
                                  std::to_string(shared_tz.load());
            
            sendto(sockfd, payload.c_str(), payload.length(), 0, 
                  (const struct sockaddr *) &server_addr, sizeof(server_addr));
            
            // Instantly lock the thread again until the CV loop calculates a new pose
            fresh_data.store(false); 
        } else {
            // 5ms micro-sleep to prevent CPU starvation on the single-core Pi
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    close(sockfd);
}

// ---------------------------------------------------------
// EXECUTION ENTRY POINT
// ---------------------------------------------------------
int main() {
    cv::setNumThreads(1); 
    std::string ubuntu_ip = "10.42.0.1"; 
    std::thread network_thread(udp_telemetry_streamer, ubuntu_ip, 5005);

    // Hardware camera pipeline with automatic frame dropping to prevent lag
    cv::VideoCapture cap("libcamerasrc ! videoconvert ! appsink drop=true max-buffers=1", cv::CAP_GSTREAMER);
    if (!cap.isOpened()) return -1;

    cv::Mat frame_raw, frame, gray;
    cv::Mat descriptors, prev_descriptors;
    std::vector<cv::KeyPoint> keypoints, prev_keypoints;

    cv::Ptr<cv::ORB> orb = cv::ORB::create(100, 1.2f, 2, 31, 0, 2, cv::ORB::FAST_SCORE, 31, 25);
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

    double K_array[9] = {250.0, 0.0, 160.0, 0.0, 250.0, 100.0, 0.0, 0.0, 1.0};
    cv::Mat K = cv::Mat(3, 3, CV_64F, K_array);

    std::cout << "[SYS] Booting ARMv6l Multithreaded 3D Odometry..." << std::endl;

    while (true) {
        cap >> frame_raw;
        if (frame_raw.empty()) continue;

        cv::resize(frame_raw, frame, cv::Size(320, 200));
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        orb->detectAndCompute(gray, cv::noArray(), keypoints, descriptors);

        if (!prev_descriptors.empty() && !descriptors.empty()) {
            std::vector<cv::DMatch> matches;
            matcher->match(prev_descriptors, descriptors, matches);
            
            std::vector<cv::Point2f> good_new, good_old;
            for (const auto& match : matches) {
                if (match.distance < 50.0f) {
                    good_new.push_back(keypoints[match.queryIdx].pt);
                    good_old.push_back(prev_keypoints[match.trainIdx].pt);
                }
            }

            if (good_new.size() >= 5) {
                cv::Mat mask;
                cv::Mat E = cv::findEssentialMat(good_new, good_old, K, cv::RANSAC, 0.999, 1.0, mask);
                
                if (!E.empty() && E.rows == 3 && E.cols == 3) {
                    cv::Mat R, t;
                    cv::recoverPose(E, good_new, good_old, K, R, t, mask);
                    
                    shared_tx.store(t.at<double>(0));
                    shared_ty.store(t.at<double>(1));
                    shared_tz.store(t.at<double>(2));
                    
                    // Trigger the network thread to fire the payload
                    fresh_data.store(true); 
                }
            }
        }
        if (!descriptors.empty()) descriptors.copyTo(prev_descriptors);
        prev_keypoints = keypoints;
    }

    pipeline_active = false;
    network_thread.join();
    cap.release();
    return 0;
}
