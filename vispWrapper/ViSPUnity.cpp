/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2020 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Unity plugin that wraps some ViSP functionalities.
 *
 *****************************************************************************/
#include "ViSPUnity.h"

#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

/*!
  \file
  \brief ViSPUnity plugin functions definition.
 */

extern "C" {

/*!
 * Global variables for debug
 */
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
static vpDisplay *m_debug_display = nullptr; //!< Display associated to internal image m_I.
#else
static vpDisplay *m_debug_display = NULL; //!< Display associated to image m_I.
#endif
static bool m_debug_enable_display = false; //!< Flag used to enable/disable display associated to internal image m_I.
static bool m_debug_display_is_initialized = false; //!< Flag used to know if display associated to internal image m_I is initialized.

/*!
 * Global variables that are common.
 */
static vpImage<unsigned char> m_I; //!< Internal image updated using Visp_ImageUchar_SetFromColor32Array().
static vpCameraParameters m_cam; //!< Internal camera parameters updated using Visp_CameraParameters_Init().

/*!
 * Global variables for vpDetectorAprilTag
 */
static vpDetectorAprilTag m_detector; //!< Internal AprilTag detector instance initialized using Visp_DetectorAprilTag_Init().
static float m_detector_quad_decimate = 1.0; //!< Internal parameter associated to AprilTag detector instance modified using Visp_DetectorAprilTag_Init().
static int m_detector_nthreads = 1; //!< Internal parameter associated to AprilTag detector instance modified using Visp_DetectorAprilTag_Init().
static double m_tagSize;

/*!
 * Global variables for PBVS
 */
static vpServo task;
static vpHomogeneousMatrix cdMc, cMo, oMo, cdMo;
static vpFeatureTranslation t((vpFeatureTranslation::cdMc));
static vpFeatureThetaU tu((vpFeatureThetaU::cdRc));
double convergence_threshold_t = 0.0005, convergence_threshold_tu = vpMath::rad(0.5);
bool display_tag = true;
bool opt_verbose = false;
bool opt_plot = false;
bool opt_adaptive_gain = false;
bool opt_task_sequencing = false;
double t_init_servo;

bool final_quit = false;
bool has_converged = false;
bool send_velocities = false;
bool servo_started = false;
std::vector<vpImagePoint> *traj_vip = nullptr; // To memorize point trajectory


void Visp_EnableDisplayForDebug(bool enable_display)
{
    m_debug_enable_display = enable_display;
}

void Visp_WrapperFreeMemory()
{
    if (m_debug_display) {
        delete m_debug_display;
        m_debug_enable_display = false;
        m_debug_display_is_initialized = false;
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
        m_debug_display = nullptr;
#else
        m_debug_display = NULL;
#endif
    }
}

void Visp_CameraParameters_Init(double cam_px, double cam_py, double cam_u0, double cam_v0)
{
    m_cam.initPersProjWithoutDistortion(cam_px, cam_py, cam_u0, cam_v0);
}

int Visp_Init(double tag_size, float *desired_trans_quat, float *initial_trans_quat,
              float quad_decimate, int nthreads) {
    // Initialize AprilTag detector
    m_detector_quad_decimate = quad_decimate;
    m_detector_nthreads = nthreads;
    m_detector.setAprilTagFamily(vpDetectorAprilTag::TAG_36h11);
    // TODO: m_detector.setAprilTagPoseEstimationMethod(vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
    m_detector.setAprilTagQuadDecimate(m_detector_quad_decimate);
    m_detector.setAprilTagNbThreads(m_detector_nthreads);
    m_detector.setDisplayTag(m_debug_enable_display, vpColor::none, 3);

    m_tagSize = tag_size;

    // Desired camera position with respect to the object
    vpQuaternionVector quat_d(desired_trans_quat[3], desired_trans_quat[4], desired_trans_quat[5], desired_trans_quat[6]);
    vpTranslationVector trans_d(desired_trans_quat[0], desired_trans_quat[1], desired_trans_quat[2]);
    cdMo.insert(trans_d);
    cdMo.insert(quat_d);

    cdMc = cdMo * cMo.inverse();
    t.buildFrom(cdMc);
    tu.buildFrom(cdMc);

    vpFeatureTranslation td((vpFeatureTranslation::cdMc));
    vpFeatureThetaU tud((vpFeatureThetaU::cdRc));

    task.addFeature(t, td);
    task.addFeature(tu, tud);
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);

    if (opt_adaptive_gain) {
        vpAdaptiveGain lambda(1.5, 0.4, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
        task.setLambda(lambda);
    } else {
        task.setLambda(0.5);
    }

    t_init_servo = vpTime::measureTimeMs();

    return 0;
}

/*!
 * Set vpImage from Unity Color32 array image.
 * \param bitmap : Bitmap color 32 array that contains the color RGBA [height x width] image.
 * \param height : Image height.
 * \param width : Image width.
 */
bool Visp_Process(unsigned char *bitmap, int height, int width, double *velocity_skew,
                  float *tag_cog, float *tag_length, float *tag_cMo, double *detection_time,
                  double *cd_t_c_out, double *cd_tu_c_out) {
    // 1. Copy image
    m_I.resize(static_cast<unsigned int>(height), static_cast<unsigned int>(width));
    vpImageConvert::RGBaToGrey(bitmap, m_I.bitmap, static_cast<unsigned int>(width * height));
    vpImageTools::flip(m_I);

    if (m_debug_enable_display && ! m_debug_display_is_initialized) {
#if defined(VISP_HAVE_X11)
        m_debug_display = new vpDisplayX(m_I);
    m_debug_display_is_initialized = true;
#elif defined VISP_HAVE_GDI
        m_debug_display = new vpDisplayGDI(m_I);
    m_debug_display_is_initialized = true;
#elif defined VISP_HAVE_OPENCV
        m_debug_display = new vpDisplayOpenCV(m_I);
    m_debug_display_is_initialized = true;
#endif
    }

    // 2. Detect AprilTag
    double t_start = vpTime::measureTimeMs();

    if (m_debug_enable_display && m_debug_display_is_initialized) {
        vpDisplay::display(m_I);
    }
    // Detection
    std::vector<vpHomogeneousMatrix> cMo_v;
    bool tag_detected = m_detector.detect(m_I, m_tagSize, m_cam, cMo_v);

    vpColVector v_c(6);

    if (tag_detected) {
        // If the image contains an aprilTag we pick the first one
        unsigned int tag_id = 0;
        // Tag characteristics
        vpImagePoint cog = m_detector.getCog(tag_id);
        tag_cog[0] = static_cast<float>(cog.get_u());
        tag_cog[1] = static_cast<float>(cog.get_v());

        std::vector <vpImagePoint> corners = m_detector.getPolygon(tag_id);
        tag_length[0] = static_cast<float>(vpImagePoint::distance(corners[0], corners[1])); // side1
        tag_length[1] = static_cast<float>(vpImagePoint::distance(corners[1], corners[2])); // side2
        tag_length[2] = static_cast<float>(vpImagePoint::distance(corners[2], corners[3])); // side3
        tag_length[3] = static_cast<float>(vpImagePoint::distance(corners[3], corners[0])); // side4
        tag_length[4] = static_cast<float>(vpImagePoint::distance(corners[0], corners[2])); // diagonal1
        tag_length[5] = static_cast<float>(vpImagePoint::distance(corners[1], corners[3])); // diagonal2

        cMo = cMo_v[tag_id];

        // Update output pose array
        for (unsigned int i = 0; i < 16; i++) {
            tag_cMo[i] = static_cast<float>(cMo.data[i]);
        }

        static bool first_time = true;
        if (first_time) {
            // Introduce security wrt tag positionning in order to avoid PI rotation
            std::vector<vpHomogeneousMatrix> v_oMo(2), v_cdMc(2);
            v_oMo[1].buildFrom(0, 0, 0, 0, 0, M_PI);
            for (size_t i = 0; i < 2; i++) {
                v_cdMc[i] = cdMo * v_oMo[i] * cMo.inverse();
            }
            if (std::fabs(v_cdMc[0].getThetaUVector().getTheta()) < std::fabs(v_cdMc[1].getThetaUVector().getTheta())) {
                oMo = v_oMo[0];
            } else {
                std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl;
                oMo = v_oMo[1]; // Introduce PI rotation
            }
        }

        // Update visual features
        cdMc = cdMo * oMo * cMo.inverse();
        t.buildFrom(cdMc);
        tu.buildFrom(cdMc);

        try {
            if (opt_task_sequencing) {
                if (!servo_started) {
                    if (send_velocities) {
                        servo_started = true;
                    }
                    t_init_servo = vpTime::measureTimeMs();
                }
                // TODO: v_c = task.computeControlLaw((vpTime::measureTimeMs() - t_init_servo) / 1000.);
            } else {
                // TODO: v_c = task.computeControlLaw();
            }
			v_c = 0;
        } catch (const vpException& e) {
            Debug::Log(e.getStringMessage(), Color::Yellow);
            Debug::Log(e.getCode(), Color::Yellow);
            return has_converged;
        }

        if (m_debug_enable_display && m_debug_display_is_initialized) {
            vpDisplay::displayFrame(m_I, cMo, m_cam, m_tagSize / 2, vpColor::none, 3);
            vpDisplay::displayFrame(m_I, cdMo * oMo, m_cam, m_tagSize / 1.5, vpColor::yellow, 2);
            // Get the tag cog corresponding to the projection of the tag frame in the image
            corners.push_back(cog);
            // Display the trajectory of the points
            if (first_time) {
                traj_vip = new std::vector<vpImagePoint>[corners.size()];
            }
            // TODO: display_point_trajectory(m_I, corners, traj_vip);
        }

        vpTranslationVector cd_t_c = cdMc.getTranslationVector();
        vpThetaUVector cd_tu_c = cdMc.getThetaUVector();
        double error_tr = sqrt(cd_t_c.sumSquare());
        double error_tu = vpMath::deg(sqrt(cd_tu_c.sumSquare()));

        for (int i = 0; i < 3; i++) {
            cd_t_c_out[i] = cd_t_c[i];
        }

        for (int i = 0; i < 3; i++) {
            cd_tu_c_out[i] = cd_tu_c[i];
        }

        if (error_tr < convergence_threshold_t && error_tu < convergence_threshold_tu) {
            has_converged = true;
            std::cout << "Servo task has converged" << std::endl;;
            if (m_debug_enable_display && m_debug_display_is_initialized) {
                vpDisplay::displayText(m_I, 100, 20, "Servo task has converged", vpColor::red);
            }
        }

        if (first_time) {
            first_time = false;
        }
    } else {
        v_c = 0;
    }

    *detection_time = vpTime::measureTimeMs() - t_start;
    if (m_debug_enable_display && m_debug_display_is_initialized) {
        std::stringstream ss;
        ss << "Loop time: " << *detection_time << std::endl;
        vpDisplay::displayText(m_I, 20, 20, ss.str(), vpColor::red);
        vpDisplay::flush(m_I);
    }

    for (int i = 0; i < 6; i++) {
        velocity_skew[i] = v_c[i];
    }

    return has_converged;
}
}

//-------------------------------------------------------------------
void  Debug::Log(const char* message, Color color) {
    if (callbackInstance != nullptr)
        callbackInstance(message, (int)color, (int)strlen(message));
}

void  Debug::Log(const std::string message, Color color) {
    const char* tmsg = message.c_str();
    if (callbackInstance != nullptr)
        callbackInstance(tmsg, (int)color, (int)strlen(tmsg));
}

void  Debug::Log(const int message, Color color) {
    std::stringstream ss;
    ss << message;
    send_log(ss, color);
}

void  Debug::Log(const char message, Color color) {
    std::stringstream ss;
    ss << message;
    send_log(ss, color);
}

void  Debug::Log(const float message, Color color) {
    std::stringstream ss;
    ss << message;
    send_log(ss, color);
}

void  Debug::Log(const double message, Color color) {
    std::stringstream ss;
    ss << message;
    send_log(ss, color);
}

void Debug::Log(const bool message, Color color) {
    std::stringstream ss;
    if (message)
        ss << "true";
    else
        ss << "false";

    send_log(ss, color);
}

void Debug::send_log(const std::stringstream &ss, const Color &color) {
    const std::string tmp = ss.str();
    const char* tmsg = tmp.c_str();
    if (callbackInstance != nullptr)
        callbackInstance(tmsg, (int)color, (int)strlen(tmsg));
}
//-------------------------------------------------------------------

//Create a callback delegate
void RegisterDebugCallback(FuncCallBack cb) {
    callbackInstance = cb;
}
