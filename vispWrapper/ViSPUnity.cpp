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

/*!
 * Global variables for IBVS
 */
static vpServo task;
static vpHomogeneousMatrix cMo;
static vpPoint point[4];
static vpFeaturePoint p[4];

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

/*!
 * Set vpImage from Unity Color32 array image.
 * \param bitmap : Bitmap color 32 array that contains the color RGBA [height x width] image.
 * \param height : Image height.
 * \param width : Image width.
 */
void Visp_ImageUchar_SetFromColor32Array(unsigned char *bitmap, int height, int width)
{
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
}

void Visp_CameraParameters_Init(double cam_px, double cam_py, double cam_u0, double cam_v0)
{
  m_cam.initPersProjWithoutDistortion(cam_px, cam_py, cam_u0, cam_v0);
}

void Visp_IBVS_Init(float *desired_trans_quat, float *initial_trans_quat)
{
    // Desired camera position with respect to the object
    vpQuaternionVector qd(desired_trans_quat[3], desired_trans_quat[4], desired_trans_quat[5], desired_trans_quat[6]);
    vpTranslationVector td(desired_trans_quat[0], desired_trans_quat[1], desired_trans_quat[2]);
    vpHomogeneousMatrix cdMo(td, qd);

    // Initial camera position with respect to the object
    vpQuaternionVector q(initial_trans_quat[3], initial_trans_quat[4], initial_trans_quat[5], initial_trans_quat[6]);
    vpTranslationVector t(initial_trans_quat[0], initial_trans_quat[1], initial_trans_quat[2]);
    cMo.insert(q);
    cMo.insert(t);

    // Then we define four 3D points that represent the corners of a 20cm by 20cm square.
    point[0].setWorldCoordinates(-0.1, -0.1, 0);
    point[1].setWorldCoordinates(0.1, -0.1, 0);
    point[2].setWorldCoordinates(0.1, 0.1, 0);
    point[3].setWorldCoordinates(-0.1, 0.1, 0);

    // The instantiation of the visual servo task is done with the next lines
    task.setServo(vpServo::EYEINHAND_CAMERA); // We initialize the task as an eye in hand visual servo. Resulting velocities computed by the controller are those that should be applied in the camera frame
    task.setInteractionMatrixType(vpServo::CURRENT); // The interaction matrix will be computed from the current visual features. Thus they need to be updated at each iteration of the control loop
    task.setLambda(0.5); // Finally, the constant gain lambda is set to 0.5

    // It is now time to define four visual features as points in the image-plane.
    // To this end we instantiate the vpFeaturePoint class.
    // The current point feature "s" is implemented in p[i]. The desired point feature "s*" is implemented in pd[i].
    vpFeaturePoint pd[4];

    // Each feature is obtained by computing the position of the 3D points in the corresponding camera frame,
    // and then by applying the perspective projection. Once current and desired features are created,
    // they are added to the visual servo task.
    for (unsigned int i = 0; i < 4; i++) {
        point[i].track(cdMo);
        vpFeatureBuilder::create(pd[i], point[i]);
        point[i].track(cMo);
        vpFeatureBuilder::create(p[i], point[i]);
        task.addFeature(p[i], pd[i]);
    }

    // For the simulation we need first to create two homogeneous transformations wMc and wMo,
    // respectively to define the position of the camera, and the position of the object in the world frame
    // vpHomogeneousMatrix wMo;
    /*vpHomogeneousMatrix wMc(wMc_trans_quat[0], wMc_trans_quat[1], wMc_trans_quat[2],
                            wMc_trans_quat[3], wMc_trans_quat[4], wMc_trans_quat[5], wMc_trans_quat[6]);*/

    // Secondly. we create an instance of our free flying camera. Here we also specify the sampling time to 0.040 seconds.
    // When a velocity is applied to the camera, this time will be used by the exponential map to determine the next position of the camera.
    // vpSimulatorCamera robot;
    // robot.setSamplingTime(0.040);

    // Finally, from the initial position wMc of the camera and the position of the object previously fixed in the
    // camera frame cMo, we compute the position of the object in the world frame wMo. Since in our simulation the
    // object is static, wMo will remain unchanged.
    // robot.getPosition(wMc);
    // wMo = wMc * cMo;
}

bool Visp_IBVS_Process(float *cMo_flat, float *velocity_skew)
{
    // velocity will be asisgned the computed vc

    // Now we can enter in the visual servo loop. When a velocity is applied to our free flying camera,
    // the position of the camera frame wMc will evolve wrt the world frame. From this position we compute
    // the position of object in the new camera frame.
    for (int i = 0; i < 16; i++) {
        cMo.data[i] = cMo_flat[i];
    }

    // The current visual features are then updated by projecting the 3D points in the image-plane associated
    // to the new camera location cMo.
    for (unsigned int i = 0; i < 4; i++) {
        point[i].track(cMo);
        vpFeatureBuilder::create(p[i], point[i]);
    }
    // Finally, the velocity skew vc is computed.
    vpColVector v = task.computeControlLaw();
    for (int i = 0; i < 6; i++) {
        velocity_skew[i] = static_cast<float>(v.data[i]);
    }
    return true;
}

void Visp_DetectorAprilTag_Init(float quad_decimate, int nthreads)
{
  // Initialize AprilTag detector
  m_detector_quad_decimate = quad_decimate;
  m_detector_nthreads = nthreads;
  m_detector.setAprilTagFamily(vpDetectorAprilTag::TAG_36h11);
  m_detector.setAprilTagQuadDecimate(m_detector_quad_decimate);
  m_detector.setAprilTagNbThreads(m_detector_nthreads);
  m_detector.setDisplayTag(m_debug_enable_display, vpColor::none, 3);
}

bool Visp_DetectorAprilTag_Process(double tag_size, float *tag_cog, float *tag_length, float *tag_cMo, double *detection_time)
{
  double t_start = vpTime::measureTimeMs();

  if (m_debug_enable_display && m_debug_display_is_initialized) {
    vpDisplay::display(m_I);
  }
  // Detection
  std::vector<vpHomogeneousMatrix> cMo_v;
  bool tag_detected = m_detector.detect(m_I, tag_size, m_cam, cMo_v);

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

    // Update output pose array
    for (unsigned int i = 0; i < 16; i++) {
      tag_cMo[i] = static_cast<float>(cMo_v[tag_id].data[i]);
    }

    if (m_debug_enable_display && m_debug_display_is_initialized) {
      vpDisplay::displayFrame(m_I, cMo_v[tag_id], m_cam, tag_size / 2, vpColor::none, 3);
    }
  }

  *detection_time = vpTime::measureTimeMs() - t_start;

  if (m_debug_enable_display && m_debug_display_is_initialized) {
    std::stringstream ss;
    ss << "Loop time: " << *detection_time << std::endl;
    vpDisplay::displayText(m_I, 20, 20, ss.str(), vpColor::red);
    vpDisplay::flush(m_I);
  }

  return tag_detected;
}
}
