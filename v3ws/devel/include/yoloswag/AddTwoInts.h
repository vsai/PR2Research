/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by gensrv_cpp from file /home/vishalsai/Documents/PR2Research/pr2go/v3ws/src/yoloswag/srv/AddTwoInts.srv
 *
 */


#ifndef YOLOSWAG_MESSAGE_ADDTWOINTS_H
#define YOLOSWAG_MESSAGE_ADDTWOINTS_H

#include <ros/service_traits.h>


#include <yoloswag/AddTwoIntsRequest.h>
#include <yoloswag/AddTwoIntsResponse.h>


namespace yoloswag
{

struct AddTwoInts
{

typedef AddTwoIntsRequest Request;
typedef AddTwoIntsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct AddTwoInts
} // namespace yoloswag


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::yoloswag::AddTwoInts > {
  static const char* value()
  {
    return "6a2e34150c00229791cc89ff309fff21";
  }

  static const char* value(const ::yoloswag::AddTwoInts&) { return value(); }
};

template<>
struct DataType< ::yoloswag::AddTwoInts > {
  static const char* value()
  {
    return "yoloswag/AddTwoInts";
  }

  static const char* value(const ::yoloswag::AddTwoInts&) { return value(); }
};


// service_traits::MD5Sum< ::yoloswag::AddTwoIntsRequest> should match 
// service_traits::MD5Sum< ::yoloswag::AddTwoInts > 
template<>
struct MD5Sum< ::yoloswag::AddTwoIntsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::yoloswag::AddTwoInts >::value();
  }
  static const char* value(const ::yoloswag::AddTwoIntsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::yoloswag::AddTwoIntsRequest> should match 
// service_traits::DataType< ::yoloswag::AddTwoInts > 
template<>
struct DataType< ::yoloswag::AddTwoIntsRequest>
{
  static const char* value()
  {
    return DataType< ::yoloswag::AddTwoInts >::value();
  }
  static const char* value(const ::yoloswag::AddTwoIntsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::yoloswag::AddTwoIntsResponse> should match 
// service_traits::MD5Sum< ::yoloswag::AddTwoInts > 
template<>
struct MD5Sum< ::yoloswag::AddTwoIntsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::yoloswag::AddTwoInts >::value();
  }
  static const char* value(const ::yoloswag::AddTwoIntsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::yoloswag::AddTwoIntsResponse> should match 
// service_traits::DataType< ::yoloswag::AddTwoInts > 
template<>
struct DataType< ::yoloswag::AddTwoIntsResponse>
{
  static const char* value()
  {
    return DataType< ::yoloswag::AddTwoInts >::value();
  }
  static const char* value(const ::yoloswag::AddTwoIntsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // YOLOSWAG_MESSAGE_ADDTWOINTS_H
