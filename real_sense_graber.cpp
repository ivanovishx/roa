// =============================================================================
//
//  ROA
//  Main
//
//  Created by Ivan Lozano 6/20/17.
//
//  For research purposes using Realsense SR300 Camera
//
// =============================================================================
#include <iostream>
using namespace std;
//----------Server-----------------
#include "serverROA.h"
// #include <exception>         // For exception class
// #include <winsock.h>
// WORD versionWanted = MAKEWORD(1, 1);
// WSADATA wsaData;
// WSAStartup(versionWanted, &wsaData);
///////////////////////////////////////

#include <boost/lexical_cast.hpp>

#include <pxcimage.h>
#include <pxccapture.h>
#include <pxcprojection.h>
#include <pxcsensemanager.h>

#include <pcl/common/io.h>
#include <pcl/common/time.h>
//////////
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//////////
#include "real_sense_grabber.h"
#include "real_sense/real_sense_device_manager.h"
#include "buffers.h"
#include "io_exception.h"

using namespace pcl::io;
using namespace pcl::io::real_sense;

/* Helper function to convert a PXCPoint3DF32 point into a PCL point.
 * Takes care of unit conversion (PXC point coordinates are in millimeters)
 * and invalid points. */
template <typename T> inline void

convertPoint (const PXCPoint3DF32& src, T& tgt)
{
  static const float nan = std::numeric_limits<float>::quiet_NaN ();
  if (src.z == 0)
  {
    tgt.x = tgt.y = tgt.z = 0;
  }
  else
  {
    tgt.x = -src.x / 1000.0;
    tgt.y = src.y / 1000.0;
    tgt.z = src.z / 1000.0;
  }
}



inline uint64_t now_ms() {
  uint64_t timestamp = pcl::getTime () * 1.0e+3;
  return timestamp;
}

pcl::RealSenseGrabber::Mode::Mode ()
  : fps (0), depth_width (0), depth_height (0), color_width (0), color_height (0)
{
}

pcl::RealSenseGrabber::Mode::Mode (unsigned int f)
  : fps (f), depth_width (0), depth_height (0), color_width (0), color_height (0)
{
}

pcl::RealSenseGrabber::Mode::Mode (unsigned int dw, unsigned int dh)
  : fps (0), depth_width (dw), depth_height (dh), color_width (0), color_height (0)
{
}

pcl::RealSenseGrabber::Mode::Mode (unsigned int f, unsigned int dw, unsigned int dh)
  : fps (f), depth_width (dw), depth_height (dh), color_width (0), color_height (0)
{
}

pcl::RealSenseGrabber::Mode::Mode (unsigned int dw, unsigned int dh, unsigned int cw, unsigned int ch)
  : fps (0), depth_width (dw), depth_height (dh), color_width (cw), color_height (ch)
{
}

pcl::RealSenseGrabber::Mode::Mode (unsigned int f, unsigned int dw, unsigned int dh, unsigned int cw, unsigned int ch)
  : fps (f), depth_width (dw), depth_height (dh), color_width (cw), color_height (ch)
{
}

bool
pcl::RealSenseGrabber::Mode::operator== (const pcl::RealSenseGrabber::Mode& m) const
{
  return (this->fps == m.fps &&
          this->depth_width == m.depth_width &&
          this->depth_height == m.depth_height &&
          this->color_width == m.color_width &&
          this->color_height == m.color_height);
}

pcl::RealSenseGrabber::RealSenseGrabber (const std::string& device_id, const Mode& mode, bool strict)
  : Grabber ()
  , is_running_ (false)
  , confidence_threshold_ (6)
  , temporal_filtering_type_ (RealSense_None)
  , temporal_filtering_window_size_ (1)
  , mode_requested_ (mode)
  , strict_ (strict)
{
  if (device_id == "")
    device_ = RealSenseDeviceManager::getInstance ()->captureDevice ();
  else if (device_id[0] == '#')
    device_ = RealSenseDeviceManager::getInstance ()->captureDevice (boost::lexical_cast<int> (device_id.substr (1)) - 1);
  else
    device_ = RealSenseDeviceManager::getInstance ()->captureDevice (device_id);

  point_cloud_signal_ = createSignal<sig_cb_real_sense_point_cloud> ();
  point_cloud_rgba_signal_ = createSignal<sig_cb_real_sense_point_cloud_rgba> ();
}

pcl::RealSenseGrabber::~RealSenseGrabber () throw ()
{
  stop ();

  disconnect_all_slots<sig_cb_real_sense_point_cloud> ();
  disconnect_all_slots<sig_cb_real_sense_point_cloud_rgba> ();
}

void
pcl::RealSenseGrabber::start ()
{
  if (!is_running_)
  {
    need_xyz_ = num_slots<sig_cb_real_sense_point_cloud> () > 0;
    need_xyzrgba_ = num_slots<sig_cb_real_sense_point_cloud_rgba> () > 0;
    if (need_xyz_ || need_xyzrgba_)
    {
      selectMode ();
      PXCCapture::Device::StreamProfileSet profile;
      memset (&profile, 0, sizeof (profile));
      profile.depth.frameRate.max = mode_selected_.fps;
      profile.depth.frameRate.min = mode_selected_.fps;
      profile.depth.imageInfo.width = mode_selected_.depth_width;
      profile.depth.imageInfo.height = mode_selected_.depth_height;
      profile.depth.imageInfo.format = PXCImage::PIXEL_FORMAT_DEPTH;
      profile.depth.options = PXCCapture::Device::STREAM_OPTION_ANY;
      if (need_xyzrgba_)
      {
        profile.color.frameRate.max = mode_selected_.fps;
        profile.color.frameRate.min = mode_selected_.fps;
        profile.color.imageInfo.width = mode_selected_.color_width;
        profile.color.imageInfo.height = mode_selected_.color_height;
        profile.color.imageInfo.format = PXCImage::PIXEL_FORMAT_RGB32;
        profile.color.options = PXCCapture::Device::STREAM_OPTION_ANY;
      }
      device_->getPXCDevice ().SetStreamProfileSet (&profile);
      if (!device_->getPXCDevice ().IsStreamProfileSetValid (&profile))
        THROW_IO_EXCEPTION ("Invalid stream profile for PXC device");
      frequency_.reset ();
      is_running_ = true;
      thread_ = boost::thread (&RealSenseGrabber::run, this);
    }
  }
}

void
pcl::RealSenseGrabber::stop ()
{
  if (is_running_)
  {
    is_running_ = false;
    thread_.join ();
  }
}

bool
pcl::RealSenseGrabber::isRunning () const
{
  return (is_running_);
}

float
pcl::RealSenseGrabber::getFramesPerSecond () const
{
  boost::mutex::scoped_lock lock (fps_mutex_);
  return (frequency_.getFrequency ());
}

void
pcl::RealSenseGrabber::setConfidenceThreshold (unsigned int threshold)
{
  if (threshold > 15)
  {
    PCL_WARN ("[pcl::RealSenseGrabber::setConfidenceThreshold] Attempted to set threshold outside valid range (0-15)");
  }
  else
  {
    confidence_threshold_ = threshold;
    device_->getPXCDevice ().SetDepthConfidenceThreshold (confidence_threshold_);
  }
}

void
pcl::RealSenseGrabber::enableTemporalFiltering (TemporalFilteringType type, size_t window_size)
{
  if (temporal_filtering_type_ != type ||
      (type != RealSense_None && temporal_filtering_window_size_ != window_size))
  {
    temporal_filtering_type_ = type;
    temporal_filtering_window_size_ = window_size;
    if (is_running_)
    {
      stop ();
      start ();
    }
  }
}

void
pcl::RealSenseGrabber::disableTemporalFiltering ()
{
  enableTemporalFiltering (RealSense_None, 1);
}

const std::string&
pcl::RealSenseGrabber::getDeviceSerialNumber () const
{
  return (device_->getSerialNumber ());
}

std::vector<pcl::RealSenseGrabber::Mode>
pcl::RealSenseGrabber::getAvailableModes (bool only_depth) const
{
  std::vector<Mode> modes;
  PXCCapture::StreamType streams = only_depth
                                   ? PXCCapture::STREAM_TYPE_DEPTH
                                   : PXCCapture::STREAM_TYPE_DEPTH | PXCCapture::STREAM_TYPE_COLOR;
  for (int p = 0;; p++)
  {
    PXCCapture::Device::StreamProfileSet profiles = {};
    if (device_->getPXCDevice ().QueryStreamProfileSet (streams, p, &profiles) == PXC_STATUS_NO_ERROR)
    {
      if (!only_depth && profiles.depth.frameRate.max != profiles.color.frameRate.max)
        continue; // we need both streams to have the same framerate
      Mode mode;
      mode.fps = profiles.depth.frameRate.max;
      mode.depth_width = profiles.depth.imageInfo.width;
      mode.depth_height = profiles.depth.imageInfo.height;
      mode.color_width = profiles.color.imageInfo.width;
      mode.color_height = profiles.color.imageInfo.height;
      bool duplicate = false;
      for (size_t i = 0; i < modes.size (); ++i)
        duplicate |= modes[i] == mode;
      if (!duplicate)
        modes.push_back (mode);
    }
    else
    {
      break;
    }
  }
  return modes;
}

void
pcl::RealSenseGrabber::setMode (const Mode& mode, bool strict)
{
  if (mode == mode_requested_ && strict == strict_)
    return;
  mode_requested_ = mode;
  strict_ = strict;
  if (is_running_)
  {
    stop ();
    start ();
  }
}



void pcl::RealSenseGrabber:: resetCloud( struct serverROA::individualCloud* subCloud) {
  memset (&subCloud, 0, sizeof (subCloud));//reset frame
}

void pcl::RealSenseGrabber:: copyCloud( serverROA::individualCloud* subCloud, pcl::PointXYZRGBA* cloud_row, uint32_t index1, uint32_t index2, uint32_t indexIF ) {
  subCloud[indexIF].index = index1;
  subCloud[indexIF].x = (float)cloud_row[index2].x;
  subCloud[indexIF].y = (float)cloud_row[index2].y;
  subCloud[indexIF].z = (float)cloud_row[index2].z;
  subCloud[indexIF].rgba = (uint32_t)cloud_row[index2].rgba;
}

void pcl::RealSenseGrabber::run ()//rrrrrrrrr
{

  static uint64_t timerSendPCD = now_ms();
  uint32_t index;

  /*--------SERVER DECLARATION OBJs:--------*/
  serverROA* server_TCP_ROA = new serverROA();

  /*--------REALSENSE DECLARATION OBJs:--------*/
  PXCProjection* projection = device_->getPXCDevice ().CreateProjection ();
  PXCCapture::Sample sample;
  std::vector<PXCPoint3DF32> vertices (SIZE);
  createDepthBuffer ();

  /*--------MIN MAX RANGEE COLORS---call inside filter function-----*/
  uint32_t minColorValue = (uint32_t)0xFFFFFFFF;
  uint32_t maxColorValue = (uint32_t)0;
  /*--------INDEX TO SEND CLOUDS :--------*/
  uint32_t indexROA = (uint32_t)0;
  uint32_t indexBODYP = (uint32_t)0;

  /*--------SERVER :--------*/
  while (is_running_)
  {

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr xyzrgba_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ROA_Matrix; //Robotic Arm Object
    pcl::PointCloud<pcl::PointXYZRGBA> cloudSave ; // <- send this
    cloudSave.points.resize (WIDTH * HEIGHT);
    pxcStatus status;

    if (need_xyzrgba_) {
      status = device_->getPXCDevice ().ReadStreams (PXCCapture::STREAM_TYPE_DEPTH | PXCCapture::STREAM_TYPE_COLOR, &sample);
    }
    else
      status = device_->getPXCDevice ().ReadStreams (PXCCapture::STREAM_TYPE_DEPTH, &sample);

    uint64_t timestamp = pcl::getTime () * 1.0e+6;

    switch (status)
    {
    case PXC_STATUS_NO_ERROR:
    {
      fps_mutex_.lock ();
      frequency_.event ();
      fps_mutex_.unlock ();

      /*--------GET IMAGE FROM REALSENSE CAMERA BUFFER :--------*/
      if (temporal_filtering_type_ != RealSense_None)
      {
        PXCImage::ImageData   data;
        sample.depth->AcquireAccess (PXCImage::ACCESS_READ, &data);
        std::vector<unsigned short> data_copy (SIZE);
        memcpy (data_copy.data (), data.planes[0], SIZE * sizeof (unsigned short));
        sample.depth->ReleaseAccess (&data);
        depth_buffer_->push (data_copy);
        sample.depth->AcquireAccess (PXCImage::ACCESS_WRITE, &data);
        unsigned short* d = reinterpret_cast<unsigned short*> (data.planes[0]);
        for (size_t i = 0; i < SIZE; i++)
          d[i] = (*depth_buffer_)[i];
        sample.depth->ReleaseAccess (&data);
      }

      projection->QueryVertices (sample.depth, vertices.data ());

      if (need_xyzrgba_)
      { ////set image adquisition
        PXCImage::ImageData data;
        PXCImage* mapped = projection->CreateColorImageMappedToDepth (sample.depth, sample.color);
        mapped->AcquireAccess (PXCImage::ACCESS_READ, &data);
        uint32_t* d = reinterpret_cast<uint32_t*> (data.planes[0]);
        //convert
        xyzrgba_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA> (WIDTH, HEIGHT));
        xyzrgba_cloud->header.stamp = timestamp;
        xyzrgba_cloud->is_dense = false;

        /*--------FILTER BY COLORS IN DISTANCE/DEPTH AND SENDING PACKETS TCP :--------*/
        resetCloud(ROA_individualCloud);
        resetCloud(BODYP_individualCloud);
        for (int i = 0; i < HEIGHT; i++)
        {
          PXCPoint3DF32* vertices_row = &vertices[i * WIDTH];
          //we are moving the pointer WIDTH points position each loop(i x Points/row)
          pcl::PointXYZRGBA* cloud_row = &xyzrgba_cloud->points[i * WIDTH];
          uint32_t* color_row = &d[i * data.pitches[0] / sizeof (uint32_t)];
          for (int j = 0; j < WIDTH; j++) {
            index = ( i * WIDTH ) + j;
            convertPoint (vertices_row[j], cloud_row[j]);
            memcpy (&cloud_row[j].rgba, &color_row[j], sizeof (uint32_t));

            /*--------SET COLORS IN FRAME BY DISTANCE LAYERS:--------*/
            uint32_t color = cloud_row[j].rgba;
            uint32_t OriginalColor = color;
            cloud_row[j].rgba = (uint32_t)2701131775;//white
            if (cloud_row[j].z < 0.7/* && !std::isnan(NAN)*/) {
              /*Call get_color_ranges here!#*/
              uint32_t ROAColor = (uint32_t)4279303952;//Green
              uint32_t BODYPColor = (uint32_t)4278190335;//blue
              uint32_t rangeColor = (uint32_t)4294902015;//fushia

              /*--------FILTER TO UPDATE ROA:--------*/
              if (filter_ROA(color)) {
                cloud_row[j].rgba = OriginalColor;
                copyCloud( ROA_individualCloud, cloud_row, index, j, indexROA);
                indexROA++;
              }

              /*--------FILTER TO UPDATE BODYP:--------*/
              // else if (filter_BODYP(color)) {
              //   cloud_row[j].rgba = BODYPColor;
              //   copyCloud( BODYP_individualCloud, cloud_row, index, j, indexBODYP);
              //   // send_cloud_point_pkt(index, j, server_TCP_ROA, BODYP_individualCloud, 2 );//ID 2 == BODYP
              //   indexBODYP++;
              // }
              else {/**/cloud_row[j].rgba = rangeColor;}

              /*Call get_color_ranges here!#*/
              /*Call get_color_ranges here!#*/

            }//END 0.7m detection
          }
        }//end Double nested for Array
        /*--------CLOSE THE FRAME TRANSMISION :--------*/
        /*Print get_color_ranges here!#*/
        mapped->ReleaseAccess (&data);
        mapped->Release ();
      }

      if (need_xyzrgba_) {point_cloud_rgba_signal_->operator () (xyzrgba_cloud);}

      /*--------CLOUD ADQUISITION/PROCESIONG END:--------*/

      /*--------START SEND over TCP:--------*/

      /*--------TIMER FOR TCP TRANSMISION: (TODO: implement this on the THREAD class)--------*/
      if (now_ms() - timerSendPCD > 7000) {
        std::cout << "::Difference last cicle ms:" << (now_ms() - timerSendPCD) << std::endl;
        timerSendPCD = now_ms();

        /*--------SEND ROA UPDATED CLOUD:--------*/
        do {
          send_start_pkt(server_TCP_ROA, 1);
        } while (server_TCP_ROA->get_start_confirmation());

        for (int k = 0; k <= indexROA; k++) {
          // do {
            send_cloud_pkt(server_TCP_ROA, ROA_individualCloud, k);//ID 1 == ROA
          // } while (server_TCP_ROA->get_point_confirmation((int)ROA_individualCloud[k].index));

        }

        do {
          send_end_pkt(server_TCP_ROA, 1);
        } while (server_TCP_ROA->get_end_confirmation());
        printf("Total Points sent: %d\n", indexROA );
        indexROA = 0;

        /*--------SEND BODYP UPDATED CLOUD:--------*/
        // send_start_pkt(server_TCP_ROA,2);
        // send_cloud_pkt(server_TCP_ROA, BODYP_individualCloud, k, indexBODYP);//ID 2 == BODYP
        // send_end_pkt(server_TCP_ROA,2);

      }//end timer to send
      break;
    }
    case PXC_STATUS_DEVICE_LOST:
      THROW_IO_EXCEPTION ("failed to read data stream from PXC device: device lost");
    case PXC_STATUS_ALLOC_FAILED:
      THROW_IO_EXCEPTION ("failed to read data stream from PXC device: alloc failed");
    }
    sample.ReleaseImages ();
  }
  projection->Release ();
  RealSenseDevice::reset (device_);
}



void pcl::RealSenseGrabber::send_start_pkt(serverROA* server_TCP_ROA, int ID) {
  server_TCP_ROA->send_start_pkt(ID);

}
void pcl::RealSenseGrabber::send_end_pkt(serverROA* server_TCP_ROA, int ID) {
  server_TCP_ROA->send_end_pkt(ID);
}

void pcl::RealSenseGrabber::send_cloud_pkt(serverROA* server_TCP_ROA, struct serverROA::individualCloud* SendCloud, uint32_t index) {
  server_TCP_ROA->sendIndividualPoint(SendCloud, index);
}

bool pcl::RealSenseGrabber::filter_ROA(uint32_t color) {
  /*RGB Parameters to filter ROA*/
  int maxRR = 200, maxGG = 255, maxBB = 255;
  int minRR = 0, minGG = 50 , minBB = 10;
  // int maxRR = 100, maxGG = 163, maxBB = 175; //small blue original
  // int minRR = 0, minGG = 63 , minBB = 132;   //small blue original
  int colorRR = ((color >> 16) & 0x0000FF);
  int colorGG = ((color >> 8) & 0x0000FF);
  int colorBB = ((color) & 0x0000FF);
  if ( (minRR < colorRR  &&  colorRR < maxRR) &&
       (minGG < colorGG  &&  colorGG < maxGG) &&
       (minBB < colorBB  &&  colorBB < maxBB) )
  {     return 1;}
  else {return 0;}
}

bool pcl::RealSenseGrabber::filter_BODYP(uint32_t color) {
  int maxRR = 100, maxGG = 163, maxBB = 175;
  int minRR = 0, minGG = 63 , minBB = 132;
  int colorRR = ((color >> 16) & 0x0000FF);
  int colorGG = ((color >> 8) & 0x0000FF);
  int colorBB = ((color) & 0x0000FF);
  if ( (minRR < colorRR  &&  colorRR < maxRR) &&
       (minGG < colorGG  &&  colorGG < maxGG) &&
       (minBB < colorBB  &&  colorBB < maxBB) )
  {return 1;}
  else {return 0;}
}

bool pcl::RealSenseGrabber::get_color_ranges( uint32_t color, uint32_t *minColorValueOut, uint32_t* maxColorValueOut) {
  uint32_t minColorValue = (uint32_t)0xFFFFFFFF;
  uint32_t maxColorValue = (uint32_t)0;
  if (color > 0 && (color < minColorValue)) minColorValue = color;
  if (color > maxColorValue) maxColorValue = color;
  if ((minColorValue == (uint32_t)0xFFFFFFFF) && (maxColorValue == 0))
  {return 0;}
  else {
    if (minColorValue != (uint32_t)0xFFFFFFFF) {
      printf("minColorValue: %u RR:%u GG:%u BB:%u \n", minColorValue, ((minColorValue >> 16) & 0x0000FF), ((minColorValue >> 8) & 0x0000FF), ((minColorValue) & 0x0000FF)  );
    }
    if (maxColorValue != 0) {
      printf("maxColorValue: %u RR:%u GG:%u BB:%u \n", maxColorValue, ((maxColorValue >> 16) & 0x0000FF), ((maxColorValue >> 8) & 0x0000FF), ((maxColorValue) & 0x0000FF)  );
    }
    *minColorValueOut = minColorValue;
    *maxColorValueOut = maxColorValue;
    return 1;
  }
}

float pcl::RealSenseGrabber::computeModeScore (const Mode & mode) {
  const float FPS_WEIGHT = 100000;
  const float DEPTH_WEIGHT = 1000;
  const float COLOR_WEIGHT = 1;
  int f = mode.fps - mode_requested_.fps;
  int dw = mode.depth_width - mode_requested_.depth_width;
  int dh = mode.depth_height - mode_requested_.depth_height;
  int cw = mode.color_width - mode_requested_.color_width;
  int ch = mode.color_height - mode_requested_.color_height;
  float penalty;
  penalty  = std::abs (FPS_WEIGHT * f * (mode_requested_.fps != 0));
  penalty += std::abs (DEPTH_WEIGHT * dw * (mode_requested_.depth_width != 0));
  penalty += std::abs (DEPTH_WEIGHT * dh * (mode_requested_.depth_height != 0));
  penalty += std::abs (COLOR_WEIGHT * cw * (mode_requested_.color_width != 0 && need_xyzrgba_));
  penalty += std::abs (COLOR_WEIGHT * ch * (mode_requested_.color_height != 0 && need_xyzrgba_));
  return penalty;
}

void pcl::RealSenseGrabber::selectMode () {
  if (mode_requested_ == Mode ())
    mode_requested_ = Mode (30, 640, 480, 640, 480);
  float best_score = (std::numeric_limits<float>::max) ();
  std::vector<Mode> modes = getAvailableModes (!need_xyzrgba_);
  for (size_t i = 0; i < modes.size (); ++i)
  {
    Mode mode = modes[i];
    float score = computeModeScore (mode);
    if (score < best_score)
    {
      best_score = score;
      mode_selected_ = mode;
    }
  }
  if (strict_ && best_score > 0)
    THROW_IO_EXCEPTION ("PXC device does not support requested mode");
}

void pcl::RealSenseGrabber::createDepthBuffer () {
  size_t size = mode_selected_.depth_width * mode_selected_.depth_height;
  switch (temporal_filtering_type_)
  {
  case RealSense_None:
  {
    depth_buffer_.reset (new pcl::io::SingleBuffer<unsigned short> (size));
    break;
  }
  case RealSense_Median:
  {
    depth_buffer_.reset (new pcl::io::MedianBuffer<unsigned short> (size, temporal_filtering_window_size_));
    break;
  }
  case RealSense_Average:
  {
    depth_buffer_.reset (new pcl::io::AverageBuffer<unsigned short> (size, temporal_filtering_window_size_));
    break;
  }
  }
}
