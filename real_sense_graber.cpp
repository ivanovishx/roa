/*void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
  {

    uint32_t color=*(reinterpret_cast<const int *>(&(cloud->points [(cloud->width >> 1) * (cloud->height + 1)].rgb)));

    std::cout<<"distance of center pixel : "<<cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z<<" m."<<std::endl;
    std::cout<<"the RGB values : "<<((color>>16)&0x0000FF)<<" "<<((color>>8)&0x0000FF)<<" "<<((color)&0x0000FF)<<std::endl;
      count = 0;

  if (!viewer.wasStopped())
         viewer.showCloud (cloud);
  }
*/


#include <iostream>

using namespace std;
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
    tgt.x = tgt.y = tgt.z = nan;
  }
  else
  {
    tgt.x = -src.x / 1000.0;
    tgt.y = src.y / 1000.0;
    tgt.z = src.z / 1000.0;
    // tgt.z = src.z / 1000.0;
  }
}


inline uint64_t now_ms(){
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

// void
// pcl::RealSenseGrabber::splitColorsRGBA(int RGBASource)

void //ssssssss
pcl::RealSenseGrabber::sendPCD ()
{
   pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 5;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

}


void
pcl::RealSenseGrabber::run ()//rrrrrrrrr
{
  const int WIDTH = mode_selected_.depth_width;
  const int HEIGHT = mode_selected_.depth_height;
  const int SIZE = WIDTH * HEIGHT;
  static uint64_t timerSendPCD = now_ms();

  PXCProjection* projection = device_->getPXCDevice ().CreateProjection ();
  PXCCapture::Sample sample;
  std::vector<PXCPoint3DF32> vertices (SIZE);
  createDepthBuffer ();
std::cout<<"Runing" <<std::endl;
  while (is_running_)
  { //ddddddddddd
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr xyzrgba_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ROA_Matrix; //Robotic Arm Object
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr BODYP_Matrix; //Body pacient Object
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr* temp_Matrix; //Body pacient Object
    bool ROA_selected_px = false;
    //
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSave (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> cloudSave ;
    pcl::PointCloud<pcl::PointXYZ> cloudSaveB ;
    cloudSave.width = 1024;
    cloudSave.height = 1024;

    cloudSaveB.width = 1024;
    cloudSaveB.width = 1024;
    //

    pxcStatus status;
    if (need_xyzrgba_) {
      // status = device_->getPXCDevice ().ReadStreams (PXCCapture::STREAM_TYPE_DEPTH, &sample);
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

      /* We preform the following steps to convert received data into point clouds:
       *
       *   1. Push depth image to the depth buffer
       *   2. Pull filtered depth image from the depth buffer
       *   3. Project (filtered) depth image into 3D
       *   4. Fill XYZ point cloud with computed points
       *   5. Fill XYZRGBA point cloud with computed points
       *   7. Project color image into 3D
       *   6. Assign colors to points in XYZRGBA point cloud
       *
       * Steps 1-2 are skipped if temporal filtering is disabled.
       * Step 4 is skipped if there are no subscribers for XYZ clouds.
       * Steps 5-7 are skipped if there are no subscribers for XYZRGBA clouds. */

      if (temporal_filtering_type_ != RealSense_None)
      {
        PXCImage::ImageData data;
        sample.depth->AcquireAccess (PXCImage::ACCESS_READ, &data);
        std::vector<unsigned short> data_copy (SIZE);
        memcpy (data_copy.data (), data.planes[0], SIZE * sizeof (unsigned short));
        sample.depth->ReleaseAccess (&data);

        depth_buffer_->push (data_copy);

        sample.depth->AcquireAccess (PXCImage::ACCESS_WRITE, &data);
        unsigned short* d = reinterpret_cast<unsigned short*> (data.planes[0]);
        for (size_t i = 0; i < SIZE; i++)
          // d[i] = 251;

          d[i] = (*depth_buffer_)[i];
        sample.depth->ReleaseAccess (&data);
      }

      projection->QueryVertices (sample.depth, vertices.data ());

      if (need_xyz_)
      {
        xyz_cloud.reset (new pcl::PointCloud<pcl::PointXYZ> (WIDTH, HEIGHT));
        xyz_cloud->header.stamp = timestamp;
        xyz_cloud->is_dense = false;
        for (int i = 0; i < SIZE; i++)
          // convertPoint (251, xyz_cloud->points[i]);
          convertPoint (vertices[i], xyz_cloud->points[i]); //<--- converts from IntelSDK to PCL
      }

      if (need_xyzrgba_)
      {
        PXCImage::ImageData data;
        PXCImage* mapped = projection->CreateColorImageMappedToDepth (sample.depth, sample.color);
        mapped->AcquireAccess (PXCImage::ACCESS_READ, &data);
        uint32_t* d = reinterpret_cast<uint32_t*> (data.planes[0]);
        if (need_xyz_)
        {
          // We can fill XYZ coordinates more efficiently using pcl::copyPointCloud,
          // given that they were already computed for XYZ point cloud.
          xyzrgba_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA>);
          pcl::copyPointCloud (*xyz_cloud, *xyzrgba_cloud);
          xyzrgba_cloud->is_dense = false;
          for (int i = 0; i < HEIGHT; i++)
          {
            pcl::PointXYZRGBA* cloud_row = &xyzrgba_cloud->points[i * WIDTH];

            // uint32_t ja = 120;
            // uint32_t* color_row = &ja;

            uint32_t* color_row = &d[i * data.pitches[0] / sizeof (uint32_t)];
            for (int j = 0; j < WIDTH; j++)
              memcpy (&cloud_row[j].rgba, &color_row[j], sizeof (uint32_t));
          }
        }
        else
        {
          xyzrgba_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA> (WIDTH, HEIGHT));
          xyzrgba_cloud->header.stamp = timestamp;
          xyzrgba_cloud->is_dense = false;
          for (int i = 0; i < HEIGHT; i++)
          {
            PXCPoint3DF32* vertices_row = &vertices[i * WIDTH];
            pcl::PointXYZRGBA* cloud_row = &xyzrgba_cloud->points[i * WIDTH];


            uint32_t ja = 120;
            // uint32_t* color_row = &ja;
            // uint32_t* color_row = &d[i * (data.pitches[0]/2) / sizeof (uint32_t)];
            uint32_t* color_row = &d[i * data.pitches[0] / sizeof (uint32_t)]; // <---------- position of color map and XYZ position
            for (int j = 0; j < WIDTH; j++) {
              convertPoint (vertices_row[j], cloud_row[j]);
              memcpy (&cloud_row[j].rgba, &color_row[j], sizeof (uint32_t));
              uint32_t color = cloud_row[j].rgba;
              uint32_t OriginalColor = color;
              cloud_row[j].rgba = (uint32_t)2701131775;

              if (cloud_row[j].z < 0.7/* && !std::isnan(NAN)*/) {
                cloud_row[j].rgba = (uint32_t)4294902015;//fushia

                int ColorMaxFilter = 120;//130
                int ColorMinFilter = 90;//80
                /*BLUE*/
                if (((color >> 16) & 0x0000FF) < ColorMinFilter && ((color >> 8) & 0x0000FF) <  ColorMinFilter && ((color) & 0x0000FF) >= ColorMaxFilter)
                {

                  cloud_row[j].rgba = (uint32_t)4278190335; // FF0000FF //aarrggbb //blue FULL
                  cloud_row[j].rgba = OriginalColor;
                  // ROA_Matrix[j] = cloud_row[j];
                  ROA_selected_px = true;
                }
                else {
                  ROA_selected_px = false;
                }
//xxxxxx
                if (ROA_selected_px) {

                  if(now_ms()-timerSendPCD> 1000){
                  std::cout<<"::Difference last cicle ms:"<<(now_ms()-timerSendPCD)<<std::endl;
                  timerSendPCD = now_ms();
                  // sendPCD();

                  // std::cout<<" cloud_a.points.size (): "<<  xyz_cloud.points.size () <<std::endl;
                  // ROA_Matrix[j] = &cloud_row[j];
                  // ROA_Matrix[j] = xyzrgba_cloud->points[i * WIDTH];

                  // &ROA_Matrix->points[i * WIDTH] =  &xyzrgba_cloud->points[i * WIDTH];
                  // temp_Matrix.points = *temp_row_Matrix;
                  //OK pcl::PointXYZRGBA* temp_row_Matrix = &xyzrgba_cloud->points[i * WIDTH];
                  /*samples*/
                  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRead (new pcl::PointCloud<pcl::PointXYZ>);
                  // printf("width:%d\n", cloudSave.width );

/*ERROR IN MEMORY?  */    
                  // cloudSave[j].points.x=0;// = cloud_row[j].x;
                  // cloudSave.points[j].x=0;
                  // cloudSave.points[j].y = cloud_row[j].y;
                  // cloudSave.points[j].z = cloud_row[j].z;
                  // cloudSave.points[j].rgba = cloud_row[j].rgba;

                  // &ROA_Matrix->points[i * WIDTH] = *temp_Matrix;
                  // cloudSave.points[j].y = cloudSaveB.points[j].y;
                  std::cout<<" J:"<< j <<std::endl;

                  }
                }
                else {
/*ERROR IN MEMORY?     cloudSave.points[j].x = 0;
                  cloudSave.points[j].y = 0;
                  cloudSave.points[j].z = 0;
                 // cloudSave.points[j].rgba = 4278255360;*/
                }
              }
            }
          }



          // SAVE START
          // pcl::io::savePCDFileASCII ("test_pcd.pcd", cloudSave);
          // std::cerr << "Saved " << cloudSave.points.size () << " data points to test_pcd.pcd." << std::endl;
          // for (size_t k = 0; k < cloudSave.points.size (); ++k)
          // { std::cerr << "    " << cloudSave.points[k].x << " " << cloudSave.points[k].y << " " << cloudSave.points[k].z << std::endl;}
          
          // SAVE END
        }
        mapped->ReleaseAccess (&data);
        mapped->Release ();
      }

      if (need_xyzrgba_)
        // xyzrgba_cloud->is_dense = false;
        point_cloud_rgba_signal_->operator () (xyzrgba_cloud);
      // point_cloud_rgba_signal_->operator () (xyzrgba_cloud);

      if (need_xyz_)
        point_cloud_signal_->operator () (xyz_cloud);
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

float
pcl::RealSenseGrabber::computeModeScore (const Mode& mode)
{
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

void
pcl::RealSenseGrabber::selectMode ()
{
  if (mode_requested_ == Mode ())
    mode_requested_ = Mode (30, 640, 480, 640, 480);
  float best_score = std::numeric_limits<float>::max ();
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

void
pcl::RealSenseGrabber::createDepthBuffer ()
{
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
