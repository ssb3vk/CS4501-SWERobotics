// Generated by gencpp from file environment_controller/use_key.msg
// DO NOT EDIT!


#ifndef ENVIRONMENT_CONTROLLER_MESSAGE_USE_KEY_H
#define ENVIRONMENT_CONTROLLER_MESSAGE_USE_KEY_H

#include <ros/service_traits.h>


#include <environment_controller/use_keyRequest.h>
#include <environment_controller/use_keyResponse.h>


namespace environment_controller
{

struct use_key
{

typedef use_keyRequest Request;
typedef use_keyResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct use_key
} // namespace environment_controller


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::environment_controller::use_key > {
  static const char* value()
  {
    return "31c1bb88ea2fc30c8bbcc2144b52d6f7";
  }

  static const char* value(const ::environment_controller::use_key&) { return value(); }
};

template<>
struct DataType< ::environment_controller::use_key > {
  static const char* value()
  {
    return "environment_controller/use_key";
  }

  static const char* value(const ::environment_controller::use_key&) { return value(); }
};


// service_traits::MD5Sum< ::environment_controller::use_keyRequest> should match
// service_traits::MD5Sum< ::environment_controller::use_key >
template<>
struct MD5Sum< ::environment_controller::use_keyRequest>
{
  static const char* value()
  {
    return MD5Sum< ::environment_controller::use_key >::value();
  }
  static const char* value(const ::environment_controller::use_keyRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::environment_controller::use_keyRequest> should match
// service_traits::DataType< ::environment_controller::use_key >
template<>
struct DataType< ::environment_controller::use_keyRequest>
{
  static const char* value()
  {
    return DataType< ::environment_controller::use_key >::value();
  }
  static const char* value(const ::environment_controller::use_keyRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::environment_controller::use_keyResponse> should match
// service_traits::MD5Sum< ::environment_controller::use_key >
template<>
struct MD5Sum< ::environment_controller::use_keyResponse>
{
  static const char* value()
  {
    return MD5Sum< ::environment_controller::use_key >::value();
  }
  static const char* value(const ::environment_controller::use_keyResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::environment_controller::use_keyResponse> should match
// service_traits::DataType< ::environment_controller::use_key >
template<>
struct DataType< ::environment_controller::use_keyResponse>
{
  static const char* value()
  {
    return DataType< ::environment_controller::use_key >::value();
  }
  static const char* value(const ::environment_controller::use_keyResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ENVIRONMENT_CONTROLLER_MESSAGE_USE_KEY_H
