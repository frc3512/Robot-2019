// Copyright (c) 2019-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Vision.hpp"

#include <iostream>

using namespace frc3512;

Vision::Vision() : PublishNode("Vision") {
    m_thread = std::thread(&Vision::RetrieveMatrices, this);
    m_socket.bind(Constants::kRpiPort);
}

Vision::~Vision() {
    m_isRunning = false;
    m_thread.join();
}

void Vision::RetrieveMatrices() {
    while (m_isRunning) {
        std::unique_lock<std::mutex> lock(m_mutex);
        size_t count = 0;
        size_t bytesReceived;
        uint32_t rpiIP;
        uint16_t port;
        while (count < sizeof(PnP)) {
            m_socket.receive(&m_pnp + count, sizeof(PnP) - count, bytesReceived,
                             rpiIP, port);
            count += bytesReceived;
        }
        /*
                cv::Rodrigues(
                    m_pnp.rotation,
                    m_rotationMatrix);  // 3x1 to 3x3 target in camera centered
           coords m_rotationMatrix = m_rotationMatrix.t();  // transpose the 3x3
                m_translationVec =
                    m_pnp.translation *
                    -m_rotationMatrix;  // 3x1 target in camera centered coords

                // 4x4 transformation matrix from world coordinates to camera
                // coordinates (centered on camera)
                m_transformationMat = cv::Mat::eye(4, 4,
           m_rotationMatrix.type()); m_transformationMat(cv::Range(0, 2),
           cv::Range(0, 2)) = m_rotationMatrix; m_transformationMat(cv::Range(0,
           2), cv::Range(3, 3)) = m_translationVec; cv::Mat placeholderVec{1, 4,
           m_transformationMat.type()}; m_transformationMat(cv::Range(3, 3),
           cv::Range(0, 3)) = placeholderVec;

                m_transformationMat =
                    m_transformationMat
                        .inv();  // world centered coords where col 4 is
           location
                                 // of camera in world coordinates, and the 3x3
                                 // submatrix is the rotation

                // std::cout << m_transformationMat << std::endl;*/
    }
}

void Vision::SubsystemPeriodic() {}
