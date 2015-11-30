/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) TODO FILL IN YEAR HERE \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   ROS package name: sick_3vistort_driver
 *
 * \author
 *   Author: Joshua Hampp
 *
 * \date Date of creation: 05/21/2015
 *
 * \brief
 *   TCP and UDP (server) connections with boost::asio
 *   driver for the KUKA RSI XML protocol
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
 
 #pragma once

#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/console.h>	//for ROS_ERROR, ROS_INFO, ...

//! abstract connection handler to process incoming data (independent of protocol)
class Any_Session
{
public:
  /// incoming data with "data" of size "size" and handler to write back ("writer")
  typedef boost::signals2::signal<void (const char *data, const size_t size, Any_Session *writer)> SIG_ON_DATA;
  
  /// every session needs a data handler
  Any_Session(SIG_ON_DATA &cb)
    : on_data_(cb)
  {
  }
  
  virtual ~Any_Session() {}
	
  /// write a string to the network connection
  virtual void write(const std::string &buffer)=0;
  
protected:

	SIG_ON_DATA &on_data_;	///< signal handler for incoming data
};


//! implementation connection handler for TCP
class TCP_Session : public Any_Session
{
public:
  
  TCP_Session(boost::asio::io_service& io_service, SIG_ON_DATA &cb)
    : Any_Session(cb), socket_(io_service)
  {
  }
  
  virtual ~TCP_Session() {
  }

  /// getter for socket connection
  boost::asio::ip::tcp::socket& socket()
  {
    return socket_;
  }

  virtual bool connect(const std::string &path, const std::string &service)
  {
	  boost::asio::ip::tcp::resolver resolver(socket_.get_io_service());
	  boost::asio::ip::tcp::resolver::query query(path, service);
	  boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
	  boost::asio::ip::tcp::resolver::iterator end;
	  
	  while (endpoint_iterator != end) {
		  if(connect(*endpoint_iterator))
			return true;
	  }
	  
	  return false;
  }
  
  virtual bool connect(const boost::asio::ip::tcp::resolver::endpoint_type &ep)
  {
    close();
    
    boost::system::error_code error = boost::asio::error::host_not_found;
    socket_.connect(ep, error);
    if (error) {
		ROS_ERROR("connecting failed");
      return false;//throw boost::system::system_error(error);
    }
    
    socket_.async_read_some(boost::asio::buffer(data_, max_length),
        boost::bind(&TCP_Session::handle_read, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
     return true;
  }
  
  virtual void close() {
    if(socket_.is_open()) socket_.close();
    boost::mutex::scoped_lock lock(mtx_);
  }
  
protected:
	
	/// write async. to TCP socket (implement abstract)
	virtual void write(const std::string &buffer) {
      boost::asio::async_write(socket_,
          boost::asio::buffer( buffer ),
          boost::bind(&TCP_Session::handle_write, this,
            boost::asio::placeholders::error));
	}
	virtual void write(const std::vector<char> &buffer) {
      boost::asio::async_write(socket_,
          boost::asio::buffer( buffer ),
          boost::bind(&TCP_Session::handle_write, this,
            boost::asio::placeholders::error));
	}
	
	/// write async. to TCP socket
	virtual void write(const boost::asio::mutable_buffers_1 &buffer) {
      boost::asio::async_write(socket_,
          buffer,
          boost::bind(&TCP_Session::handle_write, this,
            boost::asio::placeholders::error));
	}

private:

  /// called on incoming data --> decides if an error occured or calls handler
  void handle_read(const boost::system::error_code& error,
      size_t bytes_transferred)
  {
    boost::mutex::scoped_lock lock(mtx_);
      
      if (!error)
    {
		on_data_(data_, bytes_transferred, this);
    }
    else
    {
		ROS_ERROR("error while reading from socket");
      delete this;
    }
    
    if(socket_.is_open())
		socket_.async_read_some(boost::asio::buffer(data_, max_length),
			boost::bind(&TCP_Session::handle_read, this,
			  boost::asio::placeholders::error,
			  boost::asio::placeholders::bytes_transferred));
  }

  /// called if data is written (error handling)
  void handle_write(const boost::system::error_code& error)
  {
    boost::mutex::scoped_lock lock(mtx_);
    
    if (error)
    {
		ROS_ERROR("error while writing to socket");
      delete this;
    }
  }

protected:
  boost::asio::ip::tcp::socket socket_;	///< TCP socket
  enum { max_length = 4096 };	///< max length of incoming data (shoul be enough?)
  char data_[max_length];		///< data buffer of max. length
  boost::mutex mtx_;			///< lock operations
};
