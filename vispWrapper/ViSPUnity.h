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
#ifndef VISPUnity_h
#define VISPUnity_h

/*!
  \file
  \brief ViSPUnity plugin functions declaration.
 */
#include <visp3/visp_core.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/blob/vpDot2.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <ios>

#  ifdef ViSPUnity_EXPORTS
#    define VISP_UNITY_EXPORT VISP_DLLEXPORT
#  else
#    define VISP_UNITY_EXPORT VISP_DLLIMPORT
#  endif

extern "C" {

/*!
 * Allows to enable ViSP display for debugging.
 * \param enable_display : true to enable displaying images during process.
 *
 * \warning When set to true, it opens a window that will show the image that is processed.
 * The focus comes on that windows and freezes Unity. To unfreeze Unity, just click on Unity
 * to recover focus.
 */
VISP_UNITY_EXPORT
void Visp_EnableDisplayForDebug(bool enable_display=false);

/*!
 * Free memory allocated by the wrapper.
 */
VISP_UNITY_EXPORT
void Visp_WrapperFreeMemory();

/*!
 * Initialize camera parameters
 *
 * \param cam_px, cam_py : Intrinsic camera parameter corresponding to the ratio between the focal length of the lens
 * in meters and the size of the pixel in meters.
 * \param cam_u0, cam_v0 : Coordinates of the principal point (the intersection of the optical axes with the image plane)
 * that is usually near the image center.
 */
VISP_UNITY_EXPORT
void Visp_CameraParameters_Init(double cam_px=600., double cam_py=600., double cam_u0=340., double cam_v0=240.);

/*!
 * Initialize.
 *
 * \param tag_size : Tag size in [m]. This is the lenght of the black external shape of the tag.
 * \param desired_trans_quat
 * \param initial_trans_quat
 * \param quad_decimate : Detection of tags can be done on a lower-resolution image, improving speed
 * at a cost of pose accuracy and a slight decrease in detection rate. Decoding the binary payload
 * is still done at full resolution. Default is 1.0, increase this value to reduce the computation time.
 * \param nthreads : Set the number of threads for tag detection (default is 1).
 */
VISP_UNITY_EXPORT
int Visp_Init(double tag_size, float *desired_trans_quat, float *initial_trans_quat,
              float quad_decimate = 1.f, int nthreads = 1);

/*!
 * Process vpImage from Unity Color32 array image.
 * \param bitmap : Bitmap color 32 array that contains the color RGBA [height x width] image.
 * \param height : Image height.
 * \param width : Image width.
 * \param velocity_skew : used to store computed velocity
 * \param tag_cog : 2-dim array that contains tag center of gravity coordinates (u, v) along horizontal
 * and vertical axis respectively.
 * \param tag_length : 6-dim array that contains the length in pixel of the 4 tag sides and the length of the tag diagonal.
 * \param tag_cMo : 16-dim array corresponding to the tag pose as an [4 by 4] homogeneous matrix in row-major.
 * \param detection_time : Detection time in [ms].
 * \return true when a tag is detected, false otherwise.
 */
VISP_UNITY_EXPORT
bool Visp_Process(unsigned char *bitmap, int height, int width, double *velocity_skew,
                  float *tag_cog, float *tag_length, float *tag_cMo, double *detection_time,
                  double *cd_t_c_out, double *cd_tu_c_out);


/*!
 *
 * The following pseudo-code shows how to use this function in an Unity project:
 *
 * \code
 * void Start()
 * {
 *   Visp_EnableDisplayForDebug();
 *   Visp_CameraParameters_Init();
 *   Visp_DetectorAprilTag_Init();
 * }
 * void Update()
 * {
 *   Visp_ImageUchar_SetFromColor32Array()
 *   Visp_DetectorAprilTag_Process();
 * }
 * void OnApplicationQuit()
 * {
 *   Visp_WrapperFreeMemory();
 * }
 * \endcode
 */

/*
 * DEBUG
 */
//Create a callback delegate
typedef void(*FuncCallBack)(const char* message, int color, int size);
static FuncCallBack callbackInstance = nullptr;
VISP_UNITY_EXPORT void RegisterDebugCallback(FuncCallBack cb);
}

//Color Enum
enum class Color { Red, Green, Blue, Black, White, Yellow, Orange };

class  Debug
{
public:
    static void Log(const char* message, Color color = Color::Black);
    static void Log(const std::string message, Color color = Color::Black);
    static void Log(const int message, Color color = Color::Black);
    static void Log(const char message, Color color = Color::Black);
    static void Log(const float message, Color color = Color::Black);
    static void Log(const double message, Color color = Color::Black);
    static void Log(const bool message, Color color = Color::Black);

private:
    static void send_log(const std::stringstream &ss, const Color &color);
};

#endif
