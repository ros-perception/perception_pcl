#ifndef pcl_ROS_POINT_CLOUD_H_
#define pcl_ROS_POINT_CLOUD_H_

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/for_each_type.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <boost/ref.hpp>

namespace pcl 
{
  namespace detail 
  {
    template<typename Stream, typename PointT>
    struct FieldStreamer
    {
      FieldStreamer(Stream& stream) : stream_(stream) {}

      template<typename U> void operator() ()
      {
        const char* name = traits::name<PointT, U>::value;
        uint32_t name_length = strlen(name);
        stream_.next(name_length);
        if (name_length > 0)
          memcpy(stream_.advance(name_length), name, name_length);

        uint32_t offset = traits::offset<PointT, U>::value;
        stream_.next(offset);

        uint8_t datatype = traits::datatype<PointT, U>::value;
        stream_.next(datatype);

        uint32_t count = traits::datatype<PointT, U>::size;
        stream_.next(count);
      }

      Stream& stream_;
    };

    template<typename PointT>
    struct FieldsLength
    {
      FieldsLength() : length(0) {}

      template<typename U> void operator() ()
      {
        uint32_t name_length = strlen(traits::name<PointT, U>::value);
        length += name_length + 13;
      }

      uint32_t length;
    };
  } // namespace pcl::detail
} // namespace pcl

namespace message_filters
{
  template<typename T>
  struct DefaultMessageCreator<pcl::PointCloud<T> >
  {
    boost::shared_ptr<pcl::MsgFieldMap> mapping_;

    DefaultMessageCreator()
      : mapping_( boost::make_shared<pcl::MsgFieldMap>() )
    {
    }
    
    std::shared_ptr<pcl::PointCloud<T> > operator() ()
    {
      std::shared_ptr<pcl::PointCloud<T> > msg (new pcl::PointCloud<T> ());
      pcl::detail::getMapping(*msg) = mapping_;
      return msg;
    }
  };

  namespace message_traits 
  {
    // pcl point clouds message don't have a ROS compatible header
    // the specialized meta functions below (TimeStamp and FrameId)
    // can be used to get the header data.
    template<typename T> struct HasHeader<pcl::PointCloud<T> > : std::false_type {};

    template<typename T>
    struct TimeStamp<pcl::PointCloud<T> >
    {
      // This specialization could be dangerous, but it's the best I can do.
      // If this TimeStamp struct is destroyed before they are done with the
      // pointer returned by the first functions may go out of scope, but there
      // isn't a lot I can do about that. This is a good reason to refuse to
      // returning pointers like this...
      static rclcpp::Time* pointer(typename pcl::PointCloud<T> &m) {
        header_.reset(new std_msgs::msg::Header());
        pcl_conversions::fromPCL(m.header, *(header_));
        return &(header_->stamp);
      }
      static rclcpp::Time const* pointer(const typename pcl::PointCloud<T>& m) {
        header_const_.reset(new std_msgs::msg::Header());
        pcl_conversions::fromPCL(m.header, *(header_const_));
        return &(header_const_->stamp);
      }
      static rclcpp::Time value(const typename pcl::PointCloud<T>& m) {
        return pcl_conversions::fromPCL(m.header).stamp;
      }
    private:
      static std::shared_ptr<std_msgs::msg::Header> header_;
      static std::shared_ptr<std_msgs::msg::Header> header_const_;
    };

    template<typename T>
    struct FrameId<pcl::PointCloud<T> >
    {
      static std::string* pointer(pcl::PointCloud<T>& m) { return &m.header.frame_id; }
      static std::string const* pointer(const pcl::PointCloud<T>& m) { return &m.header.frame_id; }
      static std::string value(const pcl::PointCloud<T>& m) { return m.header.frame_id; }
    };

  } // namespace message_filters::message_traits


  /// @todo Printer specialization in message_operations

} // namespace rclcpp

#endif
