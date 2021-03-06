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
 * Auto-generated by gensrv_cpp from file /home/vishalsai/Documents/PR2Research/pr2go/v3ws/src/yoloswag/srv/RecordAudio.srv
 *
 */


#ifndef YOLOSWAG_MESSAGE_RECORDAUDIO_H
#define YOLOSWAG_MESSAGE_RECORDAUDIO_H

#include <ros/service_traits.h>


#include <yoloswag/RecordAudioRequest.h>
#include <yoloswag/RecordAudioResponse.h>


namespace yoloswag
{

struct RecordAudio
{

typedef RecordAudioRequest Request;
typedef RecordAudioResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct RecordAudio
} // namespace yoloswag


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::yoloswag::RecordAudio > {
  static const char* value()
  {
    return "19618e2dbb865e4c55eb07772ccdfd38";
  }

  static const char* value(const ::yoloswag::RecordAudio&) { return value(); }
};

template<>
struct DataType< ::yoloswag::RecordAudio > {
  static const char* value()
  {
    return "yoloswag/RecordAudio";
  }

  static const char* value(const ::yoloswag::RecordAudio&) { return value(); }
};


// service_traits::MD5Sum< ::yoloswag::RecordAudioRequest> should match 
// service_traits::MD5Sum< ::yoloswag::RecordAudio > 
template<>
struct MD5Sum< ::yoloswag::RecordAudioRequest>
{
  static const char* value()
  {
    return MD5Sum< ::yoloswag::RecordAudio >::value();
  }
  static const char* value(const ::yoloswag::RecordAudioRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::yoloswag::RecordAudioRequest> should match 
// service_traits::DataType< ::yoloswag::RecordAudio > 
template<>
struct DataType< ::yoloswag::RecordAudioRequest>
{
  static const char* value()
  {
    return DataType< ::yoloswag::RecordAudio >::value();
  }
  static const char* value(const ::yoloswag::RecordAudioRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::yoloswag::RecordAudioResponse> should match 
// service_traits::MD5Sum< ::yoloswag::RecordAudio > 
template<>
struct MD5Sum< ::yoloswag::RecordAudioResponse>
{
  static const char* value()
  {
    return MD5Sum< ::yoloswag::RecordAudio >::value();
  }
  static const char* value(const ::yoloswag::RecordAudioResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::yoloswag::RecordAudioResponse> should match 
// service_traits::DataType< ::yoloswag::RecordAudio > 
template<>
struct DataType< ::yoloswag::RecordAudioResponse>
{
  static const char* value()
  {
    return DataType< ::yoloswag::RecordAudio >::value();
  }
  static const char* value(const ::yoloswag::RecordAudioResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // YOLOSWAG_MESSAGE_RECORDAUDIO_H
