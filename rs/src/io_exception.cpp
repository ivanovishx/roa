#include "pcl/io/io_exception.h"
#include <sstream>

pcl::io::IOException::IOException (const std::string& function_name, const std::string& file_name, unsigned line_number, const std::string& message)
  : function_name_ (function_name)
  , file_name_ (file_name)
  , line_number_ (line_number)
  , message_ (message)
{
  std::stringstream sstream;
  sstream << function_name_ << " @ " << file_name_ << " @ " << line_number_ << " : " << message_;
  message_long_ = sstream.str ();
}

pcl::io::IOException::~IOException () throw ()
{
}

pcl::io::IOException&
pcl::io::IOException::operator = (const IOException& exception)
{
  message_ = exception.message_;
  return (*this);
}

const char*
pcl::io::IOException::what () const throw ()
{
  return (message_long_.c_str ());
}

const std::string&
pcl::io::IOException::getFunctionName () const
{
  return (function_name_);
}

const std::string&
pcl::io::IOException::getFileName () const
{
  return (file_name_);
}

unsigned
pcl::io::IOException::getLineNumber () const
{
  return (line_number_);
}
