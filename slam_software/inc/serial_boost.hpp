#pragma once

#include <iostream>
#include <boost/asio.hpp> // might after boost 1.83.0
#include <nlohmann/json.hpp>

class Serial {
public:
    Serial (const nlohmann::json serial_cfg)
    :port(boost::asio::serial_port(io, serial_cfg["port_name"].get<std::string>())) {
        uint32_t baud_rate = serial_cfg["baud_rate"];
        bool valid = false;
        for (auto i : baud_rates){
            if (baud_rate == i)
                valid = true;
        }
        if (!valid) {
            throw std::runtime_error("non valid baudrate.");
        }
            
        port.set_option(boost::asio::serial_port_base::baud_rate(serial_cfg["baud_rate"]));

        if (serial_cfg["parity"] == "None") {
            port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        }
        /*TODO More configurations could be added.*/
    }

    size_t read(char* buf, size_t length) {
        size_t read_len = boost::asio::read(port, boost::asio::buffer(buf, length));
        if (read_len != length) {
            std::cout << "error serial read" << std::endl;
        }
        return read_len;
    }

private:
    boost::asio::io_context io;
    boost::asio::serial_port port;
    uint32_t baud_rates[10] = {2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
};
