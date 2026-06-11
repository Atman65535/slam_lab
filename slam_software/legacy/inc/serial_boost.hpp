#include <boost/asio.hpp>
#include <boost/asio/basic_readable_pipe.hpp>
#include <boost/asio/basic_streambuf.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/serial_port_base.hpp>
#include <boost/system/detail/error_code.hpp>
#include <cerrno>
#include <cstddef>
#include <iostream>
#include <string>

class Serial {
public:
	Serial(const std::string port_name, int baudrate)
	:port(boost::asio::serial_port(io, port_name)) {
		port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
		port.set_option(boost::asio::serial_port_base::character_size(8));
		// port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none))
	}
	size_t read(char* buf, size_t length) {
		boost::system::error_code e;
		size_t read_len = boost::asio::read(port, boost::asio::buffer(buf, length), e);
		if (e) {
			std::cerr << "error reading" << e.message() << std::endl;
		}
		return read_len;
	}

private:
	boost::asio::io_context io;
	boost::asio::serial_port port;
};
